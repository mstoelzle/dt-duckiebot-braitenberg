#!/usr/bin/env python

import os
import pwd
import sys
import cv2
import numpy as np

from pprint import pprint

class Test:

    def __init__(self):
        self.mode = 'color'

    def run(self):
        script = os.path.realpath(__file__)
        dir = os.path.dirname(script)

        raw_img_path = os.path.join(dir, 'test_img', 'test_'+self.mode+'.jpg')

        raw_img = cv2.imread(raw_img_path)

        # takes raw cv2 bgr image
        # returns observations_left and observations_right
        # observations_left describe features which lead the robot to turn left
        # observations_left describe features which lead the robot to turn right

        total_intensity = raw_img[:,:,0].sum()+raw_img[:,:,1].sum()+raw_img[:,:,2].sum()
        avg_spectral_intensity = total_intensity/3
        avg_spectral_intensity_third = avg_spectral_intensity/3

        h = raw_img.shape[0]
        w = raw_img.shape[1]
        print(avg_spectral_intensity/h/w)

        if self.mode == 'brightness':
            gray_img = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
            left_img = self.get_img_sector(gray_img, 'left')
            middle_img = self.get_img_sector(gray_img, 'middle')
            right_img = self.get_img_sector(gray_img, 'right')
            observation_left = left_img.sum()
            observation_middle = middle_img.sum()
            observation_right = right_img.sum()
        elif self.mode == 'color':
            left_img = self.get_img_sector(raw_img, 'left')
            middle_img = self.get_img_sector(raw_img, 'middle')
            right_img = self.get_img_sector(raw_img, 'right')

            left_green = left_img[:,:,1].sum()
            left_red = left_img[:,:,2].sum()
            middle_green = middle_img[:,:,1].sum()
            middle_red = middle_img[:,:,2].sum()
            right_green = right_img[:,:,1].sum()
            right_red = right_img[:,:,2].sum()

            observation_left = (left_green-left_red)/avg_spectral_intensity_third
            observation_middle = (middle_green-middle_red)/avg_spectral_intensity_third
            observation_right = (right_green-right_red)/avg_spectral_intensity_third

        else:
            print("unknown observation mode: "+str(self.mode))
            observation_left = 0
            observation_right = 0

        observation_delta = observation_left-observation_right
        observation_sum = observation_left+observation_middle+observation_right
        observation_delta_normalized = observation_delta/observation_sum
        observation_middle_normalized = observation_middle/observation_sum

        # option 1
        speed_l = (observation_middle + observation_left)/observation_sum
        speed_r = (observation_middle + observation_right)/observation_sum

        # option 2
        # neutral_speed = 1
        # speed_l = neutral_speed*observation_middle_normalized + 1*observation_delta_normalized
        # speed_r = neutral_speed*observation_middle_normalized - 1*observation_delta_normalized

        print("observation_left: "+str(observation_left))
        print("observation_middle: "+str(observation_middle))
        print("observation_right: "+str(observation_right))
        print("observation_left_normalized: "+str(observation_left/observation_sum))
        print("observation_middle_normalized: "+str(observation_middle_normalized))
        print("observation_right_normalized: "+str(observation_right/observation_sum))
        print("observation_delta_normalized: "+str(observation_delta_normalized))

        print("speed_left: "+str(speed_l))
        print("speed_right: "+str(speed_r))

    def get_img_sector(self, img, sector):
        h = img.shape[0]
        w = img.shape[1]
        # separation between first and second third
        w1 = int(np.floor(w/3.0))
        # separation between second and third third
        w2 = int(np.floor(2*w/3.0))

        if sector == 'left':
            sector_img = img[:, 0:w1-1:1]
        elif sector == 'right':
            sector_img = img[:, w2:w-1:1]
        else:
            # middle image
            sector_img = img[:, w1:w2-1:1]

        return sector_img


if __name__ == '__main__':
    # create the node
    test = Test()
    # run node
    test.run()
