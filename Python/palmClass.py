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
from packetProcessing import unwrapFrame, ema

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

# region Calibration
    def calibrate(self, samples, timestamps):
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
        
        self._processSamples(samples, timestamps)
        self._setAnchorFrame()

        self._resetFilter()
        self._resetSmoothing()

    def _setAnchorFrame(self):
        self.CalibrationFrame = self.currentWorldOrientation
        self.isCalibrated = True

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
    def palmJointAngles(self, rawData):
        # > Smooth incoming raw data
        rawDataSmoothed = self._smoothRawData(rawData)

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
        if self.RawSmoothingFunction is None:
            return rawData

        rawAcc = rawData[0:3]
        rawGyro = rawData[3:6]
        rawMag = rawData[6:9]

        # Apply EMA smoothing to each component
        smoothedAcc = ema(rawAcc, self.prevAcc, self.accAlpha)
        smoothedGyro = ema(rawGyro, self.prevGyro, self.gyroAlpha)
        smoothedMag = ema(rawMag, self.prevMag, self.magAlpha)
        
        # Update previous values for next iteration
        self.prevAcc = smoothedAcc.copy()
        self.prevGyro = smoothedGyro.copy()
        self.prevMag = smoothedMag.copy()

        # Combine smoothed data
        smoothedData = np.concatenate([smoothedAcc, smoothedGyro, smoothedMag])
        
        return smoothedData
    
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