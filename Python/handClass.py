import numpy as np
from palmClass import Palm
# from fingerClass import Finger

class Hand:
    def __init__(self, sample_rate=100):
        self.palm = Palm(sample_rate=sample_rate)
        # self.thumb = Finger(sample_rate=sample_rate, beta=0.1, smoothing_factor=0.1)
        # self.index = Finger(sample_rate=sample_rate, beta=0.1, smoothing_factor=0.1)
        # self.middle = Finger(sample_rate=sample_rate, beta=0.1, smoothing_factor=0.1)
        # self.ring = Finger(sample_rate=sample_rate, beta=0.1, smoothing_factor=0.1)
        # self.pinky = Finger(sample_rate=sample_rate, beta=0.1, smoothing_factor=0.1)

    def updateSampleRate(self, sample_rate):
        self.palm.update_sample_rate(sample_rate)
        # self.thumb.update_sample_rate(sample_rate)
        # self.index.update_sample_rate(sample_rate)
        # self.middle.update_sample_rate(sample_rate)
        # self.ring.update_sample_rate(sample_rate)
        # self.pinky.update_sample_rate(sample_rate)
    
    def unwrapFrame(self, frame):
        P = frame[0:9]
        fingers = frame[9:].reshape(5, 6)

        return P, fingers
    
    def calibrate(self, sample_types, flatSamples, flatTimes, rightSamples, rightTimes, leftSamples, leftTimes, upsideDownSamples, upsideDownTimes, fistSamples, fistTimes):
        self.palm.calibrate(sample_types, flatSamples, flatTimes, rightSamples, rightTimes, leftSamples, leftTimes, upsideDownSamples, upsideDownTimes, fistSamples, fistTimes)
        # self.thumb.calibrate(sample_types, flatSamples, flatTimes, rightSamples, rightTimes, leftSamples, leftTimes, upsideDownSamples, upsideDownTimes, fistSamples, fistTimes)
        # self.index.calibrate(sample_types, flatSamples, flatTimes, rightSamples, rightTimes, leftSamples, leftTimes, upsideDownSamples, upsideDownTimes, fistSamples, fistTimes)
        # self.middle.calibrate(sample_types, flatSamples, flatTimes, rightSamples, rightTimes, leftSamples, leftTimes, upsideDownSamples, upsideDownTimes, fistSamples, fistTimes)
        # self.ring.calibrate(sample_types, flatSamples, flatTimes, rightSamples, rightTimes, leftSamples, leftTimes, upsideDownSamples, upsideDownTimes, fistSamples, fistTimes)
        # self.pinkie.calibrate(sample_types, flatSamples, flatTimes, rightSamples, rightTimes, leftSamples, leftTimes, upsideDownSamples, upsideDownTimes, fistSamples, fistTimes)

    def getJointAngles(self, frame):
        P, fingers = self.unwrapFrame(frame)

        palm_angles = self.palm.getJointAngles(P)
        # thumb_angles = self.thumb.getJointAngles(fingers[0])
        # index_angles = self.index.getJointAngles(fingers[1])
        # middle_angles = self.middle.getJointAngles(fingers[2])
        # ring_angles = self.ring.getJointAngles(fingers[3])
        # pinky_angles = self.pinky.getJointAngles(fingers[4])

        joint_angles = {}
        joint_angles.update(palm_angles)
        # joint_angles.update(thumb_angles)
        # joint_angles.update(index_angles)
        # joint_angles.update(middle_angles)
        # joint_angles.update(ring_angles)
        # joint_angles.update(pinky_angles)

        return joint_angles
    
    def savePalmDataToCSV(self, filename):
        self.palm.saveDataToCSV(filename)

    def plotPalmDataFromCSV(self, filename):
        self.palm.plotDataFromCSV(filename)
       