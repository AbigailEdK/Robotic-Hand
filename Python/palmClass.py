# region Imports
# === LIBRARIES ===
import numpy as np
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import csv
import re
import os
import datetime
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons, Button
from scipy.signal import savgol_filter, medfilt
from scipy import ndimage

# === UTILS ===
from madgwickUtils import scipyToMadgwick, madgwickToScipy, quatToEulers

# endregion

class Palm:
    def __init__(self, sample_rate=100, debug=True):
        self.sample_rate = sample_rate
        self.debug = debug

        # === SMOOTHING PARAMETERS ===
        self.accAlpha = 0.1
        self.gyroAlpha = 0.1
        self.magAlpha = 0.1
        self.prevAcc = np.zeros(3)
        self.prevGyro = np.zeros(3)
        self.prevMag = np.zeros(3)
        
        # === MADGWICK PARAMETERS ===
        self.beta = 0.1
        self.zeta = 0.1

        # === CALIBRATION PARAMETERS ===
        self.isCalibrated = False
        self.CalibrationFrame = np.array([0, 0, 0, 1], dtype=np.float64)  # Quaternion
        
        # === MADGWICK FILTER ===
        self.madgwick = Madgwick(beta=self.beta, frequency=sample_rate)
        self.currentWorldOrientation = np.array([0, 0, 0, 1], dtype=np.float64)  # Quaternion
        self.currentCalibratedOrientation = np.array([0, 0, 0, 1], dtype=np.float64)  # Quaternion
        self.previousCalibratedOrientation = np.array([0, 0, 0, 1], dtype=np.float64)  # Quaternion

        # === JOINT ANGLES ===
        self.wrist_joints = {
            39: "right_wrist_rotation",
            40: "right_wrist_flexion"
        }

        # === DATA LOGGING ===
        self.rawDataList = []
        self.smoothedDataList = []
        self.jointAnglesList = []

# region Calibration
    def calibrate(self, sample_types, flatSamples, flatTimes, rightSamples, rightTimes, leftSamples, leftTimes, upsideDownSamples, upsideDownTimes, fistSamples, fistTimes):
        """
        Parameters
        ----------
        calibration_positions : list of str
            List of position names corresponding to calibration data.
        calibration_data : np.ndarray
            Nx7 array where each row is [ax, ay, az, gx, gy, gz, mx, my, mz].
        calibration_times : list of float 
            List of timestamps corresponding to each calibration data point.
        
        Raises
        ------
        ValueError
            If the lengths of calibration_positions and calibration_times do not match.
        """

        if "flat" in sample_types: 
            self._processSamples(flatSamples, flatTimes)
            if self.debug: print("Flat position recorded.")
            self._setAnchorFrame()
            self._resetFilter()
            self._resetSmoothing()

        if "right" in sample_types: 
            self._processSamples(rightSamples, rightTimes)
            if self.debug: print("Right tilt position recorded.")
            self.rightTiltQuat = self.currentWorldOrientation.copy()
            self._resetFilter()
            self._resetSmoothing()

        if "left" in sample_types: 
            self._processSamples(leftSamples, leftTimes)
            if self.debug: print("Left tilt position recorded.")
            self.leftTiltQuat = self.currentWorldOrientation.copy()
            self._resetFilter()
            self._resetSmoothing()

        if "upside_down" in sample_types: 
            self._processSamples(upsideDownSamples, upsideDownTimes)
            if self.debug: print("Upside down position recorded.")
            self.upsidedownTiltQuat = self.currentWorldOrientation.copy()
            self._resetFilter()
            self._resetSmoothing()

        if "fist" in sample_types: 
            self._processSamples(fistSamples, fistTimes)
            if self.debug: print("Fist position recorded.")
            self.fistQuat = self.currentWorldOrientation.copy()
            self._resetFilter()
            self._resetSmoothing()
        
        self.isCalibrated = True

    def _setAnchorFrame(self):
        self.CalibrationFrame = self.currentWorldOrientation

    def _processSamples(self, samples, timestamps):
        for i, frame in enumerate(samples):
            if i == 0:
                dt = 0.0
            else:
                dt = timestamps[i] - timestamps[i - 1]
                if dt <= 0:
                    continue  # skip invalid intervals

            self.madgwick.Dt = dt
            self._processFrame(frame)
    
    def _processFrame(self, frame):
        # > Smooth incoming raw data
        rawDataSmoothed = self._smoothRawData(frame)

        # > Update filter with smoothed data
        self._updateMadgwick(rawDataSmoothed)
 # endregion

 # region Madgwick
    def _resetFilter(self):
        self.madgwick = Madgwick(beta=self.beta, frequency=self.sample_rate)

    def _updateMadgwick(self, rawData):
        # > Convert to madgwick format
        self.currentWorldOrientation = scipyToMadgwick(self.currentWorldOrientation)

        # > Update filter
        acc, gyr, mag = rawData[0:3], rawData[3:6], rawData[6:9]
        self.currentWorldOrientation = self.madgwick.updateMARG(self.currentWorldOrientation, gyr=gyr, acc=acc, mag=mag)

        # > Convert back to scipy format
        self.currentWorldOrientation = madgwickToScipy(self.currentWorldOrientation)

        # > Transform to calibrated frame
        self.previousCalibratedOrientation = self.currentCalibratedOrientation
        self.currentCalibratedOrientation = self._toCalibratedFrame(self.currentWorldOrientation)

    def _toCalibratedFrame(self, worldOrientation):
        if not self.isCalibrated:
            return worldOrientation

        r_world = R.from_quat(worldOrientation)
        r_calibration = R.from_quat(self.CalibrationFrame)

        r_calibrated = r_calibration.inv() * r_world
        calibratedQuat = r_calibrated.as_quat()

        return calibratedQuat
    
    def update_sample_rate(self, sample_rate):
        self.sample_rate = sample_rate
        self.madgwick.frequency = sample_rate
# endregion

# region Joint Angles
    def getJointAngles(self, rawData):
        # > Smooth incoming raw data
        self.rawDataList.append(rawData)
        rawDataSmoothed = self._smoothRawData(rawData)
        self.smoothedDataList.append(rawDataSmoothed)

        # > Update filter with smoothed data
        self._updateMadgwick(rawDataSmoothed)

        # > Compute joint angles
        joint_angles = self._computeJointAngles()

        return joint_angles

    def _computeJointAngles(self):
        slerpedQuat = self._slerpQuats(self.currentCalibratedOrientation, self.previousCalibratedOrientation, 0.5)
        
        wrist_rotation, wrist_flexion, _ = quatToEulers(slerpedQuat)
        wrist_rotation, wrist_flexion = self._deadband(wrist_rotation, wrist_flexion)
        wrist_rotation, wrist_flexion = self._clipToRanges(wrist_rotation, wrist_flexion)

        joint_angles = {
            39: wrist_rotation,
            40: wrist_flexion
        }

        return joint_angles
    
    def _deadband(self, wrist_rotation, wrist_flexion, threshold=2/180*np.pi): # 2 degrees in radians
        if abs(wrist_rotation) < threshold:
            wrist_rotation = 0.0
        if abs(wrist_flexion) < threshold:
            wrist_flexion = 0.0
        return wrist_rotation, wrist_flexion
    
    def _clipToRanges(self, wrist_rotation, wrist_flexion):
        # Define joint limits (in radians)
        wrist_rotation_min, wrist_rotation_max = -3.14159, 1.570796  # -180 to +90 degrees
        wrist_flexion_min,  wrist_flexion_max  = -1.570796, 1.570796 # -90 to +90 degrees

        # Clip values to joint limits
        wrist_rotation = np.clip(wrist_rotation, wrist_rotation_min, wrist_rotation_max)
        wrist_flexion = np.clip(wrist_flexion, wrist_flexion_min, wrist_flexion_max)

        return wrist_rotation, wrist_flexion
# endregion

# region Smoothing
    def _smoothRawData(self, rawData):
        rawAcc = rawData[0:3]
        rawGyro = rawData[3:6]
        rawMag = rawData[6:9]

        # Apply EMA smoothing to each component
        smoothedAcc = self._ema(rawAcc, self.prevAcc, self.accAlpha)
        smoothedGyro = self._ema(rawGyro, self.prevGyro, self.gyroAlpha)
        smoothedMag = self._ema(rawMag, self.prevMag, self.magAlpha)
        
        # Update previous values for next iteration
        self.prevAcc = smoothedAcc.copy()
        self.prevGyro = smoothedGyro.copy()
        self.prevMag = smoothedMag.copy()

        # Combine smoothed data
        smoothedData = np.concatenate([smoothedAcc, smoothedGyro, smoothedMag])
        
        return smoothedData
    
    def _ema(self, current_data, previous_data, alpha):
            return alpha * current_data + (1 - alpha) * previous_data
    
    def _resetSmoothing(self):
        self.prevAcc = np.zeros(3)
        self.prevGyro = np.zeros(3)
        self.prevMag = np.zeros(3)

    def _slerpQuats(self, q1, q2, t):
        r1 = R.from_quat(q1)
        r2 = R.from_quat(q2)
        r_interp = R.slerp(0, 1, [r1, r2])(t)
        return r_interp.as_quat()

# endregion

# region Data Logging
    def saveDataToCSV(self, filename):
        """Save data and joint angles to a CSV file."""
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            # Header
            writer.writerow(["raw_Ax", "raw_Ay", "raw_Az", "raw_Gx", "raw_Gy", "raw_Gz", "raw_Mx", "raw_My", "raw_Mz", "smoothed_Ax", "smoothed_Ay", "smoothed_Az", "smoothed_Gx", "smoothed_Gy", "smoothed_Gz", "smoothed_Mx", "smoothed_My", "smoothed_Mz", "Wrist_Rotation", "Wrist_Flexion"])
            # Write data
            for rawData, smoothedData, jointAngles in zip(self.rawDataList, self.smoothedDataList, self.jointAnglesList):
                row = list(rawData) + list(smoothedData) + [jointAngles[39], jointAngles[40]]
                writer.writerow(row)
        print(f"Data saved to {filename}.")

    def plotDataFromCSV(self, filename):
        """Plot data and joint angles from a CSV file."""
        with open(filename, mode='r') as f:
            reader = csv.DictReader(f)
            raw_Ax, raw_Ay, raw_Az = [], [], []
            raw_Gx, raw_Gy, raw_Gz = [], [], []
            raw_Mx, raw_My, raw_Mz = [], [], []
            smoothed_Ax, smoothed_Ay, smoothed_Az = [], [], []
            smoothed_Gx, smoothed_Gy, smoothed_Gz = [], [], []
            smoothed_Mx, smoothed_My, smoothed_Mz = [], [], []
            wrist_rotation, wrist_flexion = [], []

            for row in reader:
                raw_Ax.append(float(row["raw_Ax"]))
                raw_Ay.append(float(row["raw_Ay"]))
                raw_Az.append(float(row["raw_Az"]))
                raw_Gx.append(float(row["raw_Gx"]))
                raw_Gy.append(float(row["raw_Gy"]))
                raw_Gz.append(float(row["raw_Gz"]))
                raw_Mx.append(float(row["raw_Mx"]))
                raw_My.append(float(row["raw_My"]))
                raw_Mz.append(float(row["raw_Mz"]))
                smoothed_Ax.append(float(row["smoothed_Ax"]))
                smoothed_Ay.append(float(row["smoothed_Ay"]))
                smoothed_Az.append(float(row["smoothed_Az"]))
                smoothed_Gx.append(float(row["smoothed_Gx"]))
                smoothed_Gy.append(float(row["smoothed_Gy"]))
                smoothed_Gz.append(float(row["smoothed_Gz"]))
                smoothed_Mx.append(float(row["smoothed_Mx"]))
                smoothed_My.append(float(row["smoothed_My"]))
                smoothed_Mz.append(float(row["smoothed_Mz"]))
                wrist_rotation.append(float(row["Wrist_Rotation"]))
                wrist_flexion.append(float(row["Wrist_Flexion"]))

        # === Plot raw sensor data ===
        plt.figure(figsize=(15, 12))

        # Accelerometer data
        plt.subplot(3, 3, 1)
        plt.plot(raw_Ax, label="Raw Ax")
        plt.plot(smoothed_Ax, label="Smoothed Ax")
        plt.legend()
        plt.title("Accelerometer X-axis")

        plt.subplot(3, 3, 4)
        plt.plot(raw_Ay, label="Raw Ay")
        plt.plot(smoothed_Ay, label="Smoothed Ay")
        plt.legend()
        plt.title("Accelerometer Y-axis")

        plt.subplot(3, 3, 7)
        plt.plot(raw_Az, label="Raw Az")
        plt.plot(smoothed_Az, label="Smoothed Az")
        plt.legend()
        plt.title("Accelerometer Z-axis")

        # Gyroscope data
        plt.subplot(3, 3, 2)
        plt.plot(raw_Gx, label="Raw Gx")
        plt.plot(smoothed_Gx, label="Smoothed Gx")
        plt.legend()
        plt.title("Gyroscope X-axis")

        plt.subplot(3, 3, 5)
        plt.plot(raw_Gy, label="Raw Gy")
        plt.plot(smoothed_Gy, label="Smoothed Gy")
        plt.legend()
        plt.title("Gyroscope Y-axis")

        plt.subplot(3, 3, 8)
        plt.plot(raw_Gz, label="Raw Gz")
        plt.plot(smoothed_Gz, label="Smoothed Gz")
        plt.legend()
        plt.title("Gyroscope Z-axis")

        # Magnetometer data
        plt.subplot(3, 3, 3)
        plt.plot(raw_Mx, label="Raw Mx")
        plt.plot(smoothed_Mx, label="Smoothed Mx")
        plt.legend()
        plt.title("Magnetometer X-axis")

        plt.subplot(3, 3, 6)
        plt.plot(raw_My, label="Raw My")
        plt.plot(smoothed_My, label="Smoothed My")
        plt.legend()
        plt.title("Magnetometer Y-axis")

        plt.subplot(3, 3, 9)
        plt.plot(raw_Mz, label="Raw Mz")
        plt.plot(smoothed_Mz, label="Smoothed Mz")
        plt.legend()
        plt.title("Magnetometer Z-axis")

        plt.tight_layout()
        plt.show()

        # === Plot joint angles ===
        plt.figure(figsize=(12, 6))
        
        # Plot wrist rotation angles
        plt.subplot(2, 1, 1)
        plt.plot(wrist_rotation, label="Wrist Rotation", color="blue")
        plt.legend()
        plt.title("Wrist Rotation Angle")
        plt.xlabel("Sample")
        plt.ylabel("Angle (radians)")
        
        # Plot wrist flexion angles
        plt.subplot(2, 1, 2)
        plt.plot(wrist_flexion, label="Wrist Flexion", color="green")
        plt.legend()
        plt.title("Wrist Flexion Angle")
        plt.xlabel("Sample")
        plt.ylabel("Angle (radians)")
        
        plt.tight_layout()
        plt.show()
    # endregion