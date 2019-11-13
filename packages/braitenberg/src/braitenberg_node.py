#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml

from cv_bridge import CvBridge, CvBridgeError

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped


class BraitenbergNode(DTROS):
    """Braitenberg Behaviour

    This node implements Braitenberg vehicle behavior on a Duckiebot.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node
            that ROS will use

    Configuration:
        ~gain (:obj:`float`): scaling factor applied to the desired
            velocity, taken from the robot-specific kinematics
            calibration
        ~trim (:obj:`float`): trimming factor that is typically used
            to offset differences in the behaviour of the left and
            right motors, it is recommended to use a value that results
            in the robot moving in a straight line when forward command
            is given, taken from the robot-specific kinematics calibration
        ~baseline (:obj:`float`): the distance between the two wheels
            of the robot, taken from the robot-specific kinematics
            calibration
        ~radius (:obj:`float`): radius of the wheel, taken from the
            robot-specific kinematics calibration
        ~k (:obj:`float`): motor constant, assumed equal for both
            motors, taken from the robot-specific kinematics calibration
        ~limit (:obj:`float`): limits the final commands sent to the
            motors, taken from the robot-specific kinematics calibration

    Subscriber:
        ~image/compressed (:obj:`CompressedImage`): The acquired camera
            images

    Publisher:
        ~wheels_cmd (:obj:`duckietown_msgs.msg.WheelsCmdStamped`): The
            wheel commands that the motors will execute

    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(BraitenbergNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        # mode of braitenberg, options are:
        # "color": attracted by green, repelled by red
        # "brightness": attracted by brightness
        self.mode = rospy.get_param("/robot_name")

        # Use the kinematics calibration for the gain and trim
        self.parameters['~gain'] = None
        self.parameters['~trim'] = None
        self.parameters['~baseline'] = None
        self.parameters['~radius'] = None
        self.parameters['~k'] = None
        self.parameters['~limit'] = None

        # Set parameters using a robot-specific yaml file if such exists
        self.readParamFromFile()
        self.updateParameters()

        # control gains
        self.kp_delta = 1
        self.kp_middle = 1

        # Wait for the automatic gain control
        # of the camera to settle, before we stop it
        rospy.sleep(2.0)
        rospy.set_param('/'+self.veh_name+'/camera_node/exposure_mode', 'off')

        # subscribe to camera stream
        # construct image subscriber
        self.sub_image_topic = '/'+self.veh_name+'/camera_node/image/compressed'
        self.sub_image = rospy.Subscriber(self.sub_image_topic, CompressedImage, self.callback_image)

        # publisher for wheels command message (WheelsCmdStamped)
        self.pub_wheels_cmd_topic = '~wheels_cmd'
        # TODO: maybe the following topics work better
        # self.pub_wheels_cmd_topic = '/'+self.veh_name+'/wheels_driver_node/wheels_cmd'
        # self.pub_wheels_cmd_topic = '/'+self.veh_name+'/wheels_driver/wheels_cmd'
        self.pub_wheels_cmd = rospy.Publisher(self.pub_wheels_cmd_topic, WheelsCmdStamped, queue_size=10)

        self.log("Initialized")

    def run(self):
        pass

    def callback_image(self, img_msg):
        rospy.loginfo("I received an image message")
        try:
            raw_img = self.bridge.compressed_imgmsg_to_cv2(img_msg)

            observation_left, observation_middle, observation_right = self.process_image(raw_img)

            rospy.loginfo("observation_left: "+str(observation_left))
            rospy.loginfo("observation_middle: "+str(observation_middle))
            rospy.loginfo("observation_right: "+str(observation_right))

            u_l_limited, u_r_limited = self.control(observation_left, observation_middle, observation_right)


            self.publish_wheels_cmd_msg(u_l_limited, u_r_limited)
        except CvBridgeError as e:
            print(e)

    def process_image(self, raw_img):
        # takes raw cv2 bgr image
        # returns observations_left and observations_right
        # observations_left describe features which lead the robot to turn left
        # observations_left describe features which lead the robot to turn right

        h = raw_img.shape[0]
        w = raw_img.shape[1]


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

        return observation_left, observation_middle, observation_right

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


    def control(self, observation_left, observation_middle, observation_right):
        # observations can be integrated brightness or amount of detected color
        # observations_left describe features which lead the robot to turn left
        # observations_left describe features which lead the robot to turn right

        observation_delta = observation_left-observation_right
        observation_sum = observation_left+observation_middle+observation_right
        observation_delta_normalized = observation_delta/observation_sum
        observation_middle_normalized = observation_middle/observation_sum

        # option 1
        speed_l = (observation_middle + observation_left)/observation_sum
        speed_r = (observation_middle + observation_right)/observation_sum

        # option 2
        # speed_l = self.kp_middle*observation_middle_normalized + self.kp_delta*observation_delta_normalized
        # speed_r = self.kp_middle*observation_middle_normalized - self.kp_delta*observation_delta_normalized

        rospy.loginfo("speed_l: "+str(speed_l))
        rospy.loginfo("speed_r: "+str(speed_r))

        return self.speedToCmd(speed_l, speed_r)

    def publish_wheels_cmd_msg(self, u_l_limited, u_r_limited):
        message = WheelsCmdStamped()
        message.vel_left = u_l_limited
        message.vel_right = u_r_limited
        self.pub_wheels_cmd.publish(message)
        print("published wheels_cmd message")

    def speedToCmd(self, speed_l, speed_r):
        """Applies the robot-specific gain and trim to the
        output velocities

        Applies the motor constant k to convert the deisred wheel speeds
        to wheel commands. Additionally, applies the gain and trim from
        the robot-specific kinematics configuration.

        Args:
            speed_l (:obj:`float`): Desired speed for the left
                wheel (e.g between 0 and 1)
            speed_r (:obj:`float`): Desired speed for the right
                wheel (e.g between 0 and 1)

        Returns:
            The respective left and right wheel commands that need to be
                packed in a `WheelsCmdStamped` message

        """

        # assuming same motor constants k for both motors
        k_r = self.parameters['~k']
        k_l = self.parameters['~k']

        # adjusting k by gain and trim
        k_r_inv = (self.parameters['~gain'] + self.parameters['~trim']) \
                  / k_r
        k_l_inv = (self.parameters['~gain'] - self.parameters['~trim']) \
                  / k_l

        # conversion from motor rotation rate to duty cycle
        u_r = speed_r * k_r_inv
        u_l = speed_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = self.trim(u_r,
                                -self.parameters['~limit'],
                                self.parameters['~limit'])
        u_l_limited = self.trim(u_l,
                                -self.parameters['~limit'],
                                self.parameters['~limit'])

        return u_l_limited, u_r_limited

    def readParamFromFile(self):
        """
        Reads the saved parameters from
        `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml` or
        uses the default values if the file doesn't exist. Adjsuts
        the ROS paramaters for the node with the new values.

        """
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use the default values from the config folder if a
        # robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.log("Kinematics calibration file %s does not "
                     "exist! Using the default file." % fname, type='warn')
            fname = self.getFilePath('default')

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["gain", "trim", "baseline", "k", "radius", "limit"]:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass

    def getFilePath(self, name):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.

        Args:
            name (:obj:`str`): the Duckiebot name

        Returns:
            :obj:`str`: the full path to the robot-specific
                calibration file

        """
        cali_file_folder = '/data/config/calibrations/kinematics/'
        cali_file = cali_file_folder + name + ".yaml"
        return cali_file

    def trim(self, value, low, high):
        """
        Trims a value to be between some bounds.

        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound

        Returns:
            the trimmed value
        """

        return max(min(value, high), low)

    def onShutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown."""

        # MAKE SURE THAT THE LAST WHEEL COMMAND YOU PUBLISH IS ZERO,
        # OTHERWISE YOUR DUCKIEBOT WILL CONTINUE MOVING AFTER
        # THE NODE IS STOPPED

        # stop robot
        self.publish_wheels_cmd_msg(0,0)

        super(BraitenbergNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    node = BraitenbergNode(node_name='braitenberg')
    # run
    node.run()
    # Keep it spinning to keep the node alive
    rospy.spin()