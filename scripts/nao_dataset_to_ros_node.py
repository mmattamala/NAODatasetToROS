#!/usr/bin/env python

# ROS stuff
import rospy
import rosbag
import rospkg

# Transformations
import tf
import tf.transformations as tr

# Sensors
from sensor_msgs.msg import Imu, JointState, Image
from cv_bridge import CvBridge, CvBridgeError

# python
import os
from time import time, sleep
import cv2
import numpy as np
import struct

class DatasetPublisher(object):
  def __init__(self):
    # Read parameters to configure the node
    dataset_number = self.read_parameter('~dataset_number', 9)

    # image publisher
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)

    # joints publisher
    joints_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

    # imu publisher
    imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=1)

    # Setup TF listener and broadcaster
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()

    # read data from dataset
    rospack = rospkg.RosPack()
    script_dir = rospack.get_path('nao_dataset_to_ros')#os.path.dirname(os.path.abspath(__file__))
    images_filename      = 'datasets/dataset_%d/camera_%d.strm' % (dataset_number, dataset_number)
    groundtruth_filename = 'datasets/dataset_%d/ground_truth_%d.csv' % (dataset_number, dataset_number)
    hardware_filename    = 'datasets/dataset_%d/hardware_current_%d.csv' % (dataset_number, dataset_number)
    rospy.loginfo(images_filename)
    rospy.loginfo(groundtruth_filename)
    rospy.loginfo(hardware_filename)

    rospy.loginfo(script_dir)
    # read images
    file = open(script_dir + '/' +images_filename, "rb")
    with file:
      # get header
      header = file.read(86)
      print(header)

      # get images
      i = 0
      while not rospy.is_shutdown() and i < 20:
       print(i)

       # read seconds
       seconds = file.read(4)
       if len(seconds) is 0:
         break
       print(struct.unpack('=l', seconds))

       # read microseconds
       microseconds = file.read(4)
       if len(stuff) is 0:
          break
       print(struct.unpack('=l', microseconds))

       # read image
       image_bytes = file.read(153600)
       #print(image_bytes)
       if image_bytes is None:
         break
       yuv = self.yuv422_to_yuv444(image_bytes)
       rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
       cv2.imwrite('lower_%d.bmp' % i, rgb)
       i+=1
    #    # upper image
    #    image_bytes = file.read(153600)
    #    if image_bytes is None:
    #        break
    #    yuv = self.yuv422_to_yuv444(image_bytes)
    #    rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
    #    cv2.imwrite('upper_%d.jpg' % i, rgb)
    #    i = i+1

      ## Using PIL's fromstring format save image as jpg
      #yuv = self.yuv422_to_yuv444(file.read(153600))
      #print(yuv)
      #cv2.imshow('Color image', yuv)
      #cv2.waitKey(0)
      #cv2.destroyAllWindows()
      ##yuvstr = yuv.tostring()
      ##Img = Image.fromstring("YCbCr",(img[0],img[1]),yuvstr)
      ##Img.save("Image.jpg")


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

    rospy.loginfo(np.shape(y))
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

    rgb = cv2.cvtColor(y_channel, cv2.COLOR_GRAY2BGR)
    cv2.imwrite('test_gray.jpg', rgb)

    img_yuv = cv2.merge([y_channel, u_channel, v_channel])

    return img_yuv

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  dataset_to_ros = DatasetPublisher()
  rospy.loginfo('Shuting down [%s] node' % node_name)


##with rosbag.Bag('output.bag', 'w') as outbag:
#    for topic, msg, t in rosbag.Bag('input.bag').read_messages():
#        # This also replaces tf timestamps under the assumption
#        # that all transforms in the message share the same timestamp
#        if topic == "/tf" and msg.transforms:
#            outbag.write(topic, msg, msg.transforms[0].header.stamp)
#        else:
#            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
