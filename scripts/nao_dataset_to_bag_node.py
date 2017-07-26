#!/usr/bin/env python

# ROS stuff
import rospy
import rosbag
import rospkg

# Transformations
import tf
import tf.transformations as tr

# Standard messages
from std_msgs.msg import Float32, Float32MultiArray

# Sensors
from sensor_msgs.msg import Imu, JointState, Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

# python
import os
from time import time, sleep
import cv2
import numpy as np
import struct

class DatasetToBag(object):
    def __init__(self):
        # Read parameters to configure the node
        self.dataset_number = self.read_parameter('~dataset_number', 9)
        self.write_images = self.read_parameter('~write_images', False)
        self.units_metric = self.read_parameter('~units_metric', 1000)
        self.publish_nao_gt = self.read_parameter('~publish_nao_gt', True)

        # create bag
        self.bag_filename = 'dataset_%d.bag' % self.dataset_number
        self.bag = rosbag.Bag(self.bag_filename, 'w')

        # CV bridge
        self.cv_bridge = CvBridge()

        # rospack
        self.rospack = rospkg.RosPack()

        # read data from dataset
        script_dir = self.rospack.get_path('nao_dataset_to_ros')#os.path.dirname(os.path.abspath(__file__))
        self.images_filename      = '%s/datasets/dataset_%d/camera_%d.strm' % (script_dir, self.dataset_number, self.dataset_number)
        self.groundtruth_filename = '%s/datasets/dataset_%d/ground_truth_%d.csv' % (script_dir, self.dataset_number, self.dataset_number)
        self.hardware_filename    = '%s/datasets/dataset_%d/hardware_%d.csv' % (script_dir, self.dataset_number, self.dataset_number)
        #rospy.loginfo(self.images_filename)
        #rospy.loginfo(self.groundtruth_filename)
        #rospy.loginfo(self.hardware_filename)

        # execute processes
        rospy.loginfo('Reading images ...')
        self.read_images()
        rospy.loginfo('Reading ground truth ...')
        self.read_ground_truth()
        rospy.loginfo('Reading sensors ...')
        self.read_sensors()

        rospy.loginfo('Closing rosbag')
        self.bag.close()


    def read_images(self):
        # read images
        file_image = open(self.images_filename, "rb")
        with file_image:
            # get header
            header = file_image.read(86)
            #print(header)

            # initial time normalization
            initial_time = 0
            is_first = True

            # get images
            while not rospy.is_shutdown():
                # read seconds
                seconds_bin = file_image.read(4)
                if len(seconds_bin) is 0:
                    break
                seconds = (struct.unpack('=l', seconds_bin)[0])

                # read microseconds
                microseconds_bin = file_image.read(4)
                if len(microseconds_bin) is 0:
                    break
                microseconds = (struct.unpack('=l', microseconds_bin)[0])

                # rebuild timestamp
                #timestamp = seconds + microseconds * 10**-6

                # read image
                image_bytes = file_image.read(153600)
                #print(image_bytes)
                if image_bytes is None:
                    break

                yuv = self.yuv422_to_yuv444(image_bytes)
                rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)

                # prepare message
                if is_first:
                    initial_time = rospy.Duration(secs=seconds, nsecs=microseconds*10**3)
                    is_first = False

                ros_image = self.cv_bridge.cv2_to_imgmsg(rgb, 'bgr8')
                ros_image.header.stamp.secs = seconds
                ros_image.header.stamp.nsecs = microseconds * 10**3
                ros_image.header.stamp = ros_image.header.stamp - initial_time
                #rospy.loginfo(ros_image.header.stamp.to_sec())

                # write to bag
                self.bag.write('/camera/image_raw', ros_image, ros_image.header.stamp)

                if self.write_images:
                    cv2.imwrite( ('%s_%s.bmp' % (seconds, microseconds)), rgb)

    def read_ground_truth(self):
        # initial time normalization
        initial_time = 0
        is_first = True

        # Read ground truth
        ground_truth = np.genfromtxt(self.groundtruth_filename, delimiter=',', comments='#', skip_footer=1)
        #rospy.loginfo(np.shape(ground_truth))
        for line in ground_truth:
            (timestamp, head_rx, head_ry, head_rz, head_x, head_y, head_z, torso_rx, torso_ry, torso_rz, torso_x, torso_y, torso_z) = line
            #rospy.loginfo(timestamp)

            # prepare message
            if is_first:
                initial_time = rospy.Duration().from_sec(timestamp)
                is_first = False

            ros_stamp = rospy.Time().from_sec(timestamp) - initial_time

            # fill head transformation
            T_head = TransformStamped()
            T_head.header.frame_id = "world"
            T_head.child_frame_id = "gt_head"
            T_head.header.stamp = ros_stamp
            angles_head = tr.quaternion_from_euler(head_rx, head_ry, head_rz)
            T_head.transform.rotation.x = angles_head[0]
            T_head.transform.rotation.y = angles_head[1]
            T_head.transform.rotation.z = angles_head[2]
            T_head.transform.rotation.w = angles_head[3]
            T_head.transform.translation.x = head_x / self.units_metric
            T_head.transform.translation.y = head_y / self.units_metric
            T_head.transform.translation.z = head_z / self.units_metric
            self.bag.write('/tf', TFMessage([T_head]), ros_stamp)

            # fill torso transformation
            T_torso = TransformStamped()
            T_torso.header.frame_id = "world"
            T_torso.child_frame_id = "gt_torso"
            T_torso.header.stamp = ros_stamp
            angles_torso = tr.quaternion_from_euler(torso_rx, torso_ry, torso_rz)
            T_torso.transform.rotation.x = angles_torso[0]
            T_torso.transform.rotation.y = angles_torso[1]
            T_torso.transform.rotation.z = angles_torso[2]
            T_torso.transform.rotation.w = angles_torso[3]
            T_torso.transform.translation.x = torso_x / self.units_metric
            T_torso.transform.translation.y = torso_y / self.units_metric
            T_torso.transform.translation.z = torso_z / self.units_metric
            self.bag.write('/tf', TFMessage([T_torso]), ros_stamp)

            if self.publish_nao_gt:
                T_nao_gt = TransformStamped()
                T_nao_gt.header.frame_id = "gt_torso"
                T_nao_gt.child_frame_id = "base_link"
                T_nao_gt.header.stamp = ros_stamp
                T_nao_gt.transform.rotation.x = 0.0
                T_nao_gt.transform.rotation.y = 0.0
                T_nao_gt.transform.rotation.z = 0.0
                T_nao_gt.transform.rotation.w = 1.0
                T_nao_gt.transform.translation.x = 0.0
                T_nao_gt.transform.translation.y = 0.0
                T_nao_gt.transform.translation.z = 0.0
                self.bag.write('/tf', TFMessage([T_nao_gt]), ros_stamp)

            #rospy.loginfo(T_head.header.stamp.to_sec())
            #rospy.loginfo(T_torso.header.stamp.to_sec())

    def read_sensors(self):
        # initial time normalization
        initial_time = 0
        is_first = True

        # Read ground truth
        hardware = np.genfromtxt(self.hardware_filename, delimiter=';', comments='#', skip_footer=1)
        #rospy.loginfo(np.shape(ground_truth))
        for line in hardware:
            (timestamp, \
            servo_enabled, \
            head_yaw, head_pitch, \
            l_shoulder_pitch, l_shoulder_roll, \
            l_elbow_yaw, l_elbow_roll, \
            l_hip_yaw_pitch, l_hip_roll, l_hip_pitch, \
            l_knee_pitch, \
            l_ankle_pitch, l_ankle_roll, \
            r_hip_yaw_pitch, r_hip_roll, r_hip_pitch, \
            r_knee_pitch, \
            r_ankle_pitch, r_ankle_roll, \
            r_shoulder_pitch, r_shoulder_roll, \
            r_elbow_yaw, r_elbow_roll, \
            accel_x, accel_y, accel_z, \
            gyro_x, gyro_y, gyro_ref, \
            l_fsr_fl, l_fsr_fr, l_fsr_rl, l_fsr_rr, \
            r_fsr_fl, r_fsr_fr, r_fsr_rl, r_fsr_rr, \
            us_distance_left, us_distance_right, \
            l_bumper_l, l_bumper_r, \
            r_bumper_l, r_bumper_r, \
            chest_button, \
            battery_charge, \
            time) = line

            # prepare message
            if is_first:
                initial_time = rospy.Duration().from_sec(timestamp)
                is_first = False

            # timestamp
            ros_stamp = rospy.Time().from_sec(timestamp) - initial_time

            # IMU
            ros_imu = Imu()
            ros_imu.header.stamp = ros_stamp
            ros_imu.header.frame_id = '/Imu'
            ros_imu.angular_velocity.x = accel_x
            ros_imu.angular_velocity.y = accel_y
            ros_imu.angular_velocity.z = accel_z
            ros_imu.linear_acceleration.x = accel_x
            ros_imu.linear_acceleration.y = accel_y
            ros_imu.linear_acceleration.z = accel_z
            self.bag.write('/imu/data_raw', ros_imu, ros_stamp)

            # JointState
            ros_joints = JointState()
            ros_joints.header.stamp = ros_stamp

            ros_joints.name = [None]*26
            ros_joints.position = [0]*26

            ros_joints.name[0]      = "HeadYaw"
            ros_joints.position[0]  = head_yaw
            ros_joints.name[1]      = "HeadPitch"
            ros_joints.position[1]  = head_pitch
            ros_joints.name[2]      = "LHipYawPitch"
            ros_joints.position[2]  = l_hip_yaw_pitch
            ros_joints.name[3]      = "LHipRoll"
            ros_joints.position[3]  = l_hip_roll
            ros_joints.name[4]      = "LHipPitch"
            ros_joints.position[4]  = l_hip_pitch
            ros_joints.name[5]      = "LKneePitch"
            ros_joints.position[5]  = l_knee_pitch
            ros_joints.name[6]      = "LAnklePitch"
            ros_joints.position[6]  = l_ankle_pitch
            ros_joints.name[7]      = "LAnkleRoll"
            ros_joints.position[7]  = l_ankle_roll
            ros_joints.name[8]      = "RHipYawPitch"
            ros_joints.position[8]  = r_hip_yaw_pitch
            ros_joints.name[9]      = "RHipRoll"
            ros_joints.position[9]  = r_hip_roll
            ros_joints.name[10]     = "RHipPitch"
            ros_joints.position[10] = r_hip_pitch
            ros_joints.name[11]     = "RKneePitch"
            ros_joints.position[11] = r_knee_pitch
            ros_joints.name[12]     = "RAnklePitch"
            ros_joints.position[12] = r_ankle_pitch
            ros_joints.name[13]     = "RAnkleRoll"
            ros_joints.position[13] = r_ankle_roll
            ros_joints.name[14]     = "LShoulderPitch"
            ros_joints.position[14] = l_shoulder_pitch
            ros_joints.name[15]     = "LShoulderRoll"
            ros_joints.position[15] = l_shoulder_roll
            ros_joints.name[16]     = "LElbowYaw"
            ros_joints.position[16] = l_elbow_yaw
            ros_joints.name[17]     = "LElbowRoll"
            ros_joints.position[17] = l_elbow_roll
            ros_joints.name[18]     = "LWristYaw"
            ros_joints.position[18] = 0
            ros_joints.name[19]     = "LHand"
            ros_joints.position[19] = 0
            ros_joints.name[20]     = "RShoulderPitch"
            ros_joints.position[20] = r_shoulder_pitch
            ros_joints.name[21]     = "RShoulderRoll"
            ros_joints.position[21] = r_shoulder_roll
            ros_joints.name[22]     = "RElbowYaw"
            ros_joints.position[22] = r_elbow_yaw
            ros_joints.name[23]     = "RElbowRoll"
            ros_joints.position[23] = r_elbow_roll
            ros_joints.name[24]     = "RWristYaw"
            ros_joints.position[24] = 0
            ros_joints.name[25]     = "RHand"
            ros_joints.position[25] = 0
            self.bag.write('/joint_states', ros_joints, ros_stamp)

            # FSR
            ros_l_fsr = Float32MultiArray()
            ros_l_fsr.data = [l_fsr_fl, l_fsr_fr, l_fsr_rl, l_fsr_rr]
            self.bag.write('/fsr/l', ros_l_fsr, ros_stamp)

            ros_r_fsr = Float32MultiArray()
            ros_r_fsr.data = [r_fsr_fl, r_fsr_fr, r_fsr_rl, r_fsr_rr]
            self.bag.write('/fsr/r', ros_r_fsr, ros_stamp)

            # Battery
            ros_battery = Float32()
            ros_battery.data = battery_charge
            self.bag.write('/battery', ros_battery, ros_stamp)

            # USs
            ros_us = Float32MultiArray()
            ros_us.data = [us_distance_left, us_distance_right]
            self.bag.write('/us', ros_us, ros_stamp)

    def read_parameter(self, name, default):
        """
        Get a parameter from the ROS parameter server. If it's not found, a
        warn is printed.
        @type name: string
        @param name: Parameter name
        @param default: Default value for the parameter. The type should be
        the same as the one expected for the parameter.
        @return: The resulting parameter
        """
        if not rospy.has_param(name):
            rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
        return rospy.get_param(name, default)

    def yuv422_to_yuv444(self, image, height=240, width=320, skip=True):
        """
        This method converts a YUV422 image into yuv422ToYUV444
        Loosely based on: https://gist.github.com/baali/3525330
        """
        arr = np.fromstring(image, dtype=np.uint8)
        y = arr[0::2]   # height * width
        u = arr[1::4]   # height * width / 2
        v = arr[3::4]   # height * width / 2

        #rospy.loginfo(np.shape(y))
        u_full = np.zeros(np.shape(y), dtype=np.uint8)
        v_full = np.zeros(np.shape(y), dtype=np.uint8)

        # fill matrices
        u_full[0::2] = u
        u_full[1::2] = u
        v_full[0::2] = v
        v_full[1::2] = v

        # reshape vectors
        y_channel = np.reshape(y, (height,width))
        u_channel = np.reshape(u_full, (height,width))
        v_channel = np.reshape(v_full, (height,width))

        #rgb = cv2.cvtColor(y_channel, cv2.COLOR_GRAY2BGR)
        #cv2.imwrite('test_gray.jpg', rgb)

        img_yuv = cv2.merge([y_channel, u_channel, v_channel])

        return img_yuv

if __name__ == '__main__':
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)
    dataset_to_ros = DatasetToBag()
    rospy.loginfo('Shuting down [%s] node' % node_name)


    ##with rosbag.Bag('output.bag', 'w') as outbag:
    #    for topic, msg, t in rosbag.Bag('input.bag').read_messages():
    #        # This also replaces tf timestamps under the assumption
    #        # that all transforms in the message share the same timestamp
    #        if topic == "/tf" and msg.transforms:
    #            outbag.write(topic, msg, msg.transforms[0].header.stamp)
    #        else:
    #            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
