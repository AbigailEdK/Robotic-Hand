# receive frame
# unwrap frame
# send packets of data to palm and finger objects
# get angles out of palm and finger objects
# assemble angles into frame
# send frame to robotic hand

import numpy as np
from palmClass import Palm
# from fingerClass import Finger

class Hand:
    def __init__(self, sample_rate=100):
        self.palm = Palm(sample_rate=sample_rate, beta=0.1, smoothing_factor=0.1)
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

    def ema(self, current_data, previous_data, alpha):
            return alpha * current_data + (1 - alpha) * previous_data

    