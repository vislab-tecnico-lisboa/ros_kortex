#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('kortex_gazebo')
import sys
import os
import rospy
import cv2
import numpy as np
np.set_printoptions(threshold=np.inf)
import message_filters as mf
# import grasp_estimator
# import tensorflow as tf
# import glob
# import mayavi.mlab as mlab
# from visualization_utils import *
# import mayavi.mlab as mlab
# from grasp_data_reader import regularize_pc_point_count
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

def array_to_matrix_np(vec):

  # Instrinsics Matrix for 640x480 Resolution
  # M = np.array( [[ 463.889, 0.0, 320.0],[ 0.0, 463.889, 240.0],[ 0.0, 0.0, 1.0]] )
  # Instrinsics Matrix for 1280x720 Resolution
  if len(vec) == 9:
    M = np.array( [[ vec[0], vec[1], vec[2]],[ vec[3], vec[4], vec[5]],[ vec[6], vec[7], vec[8]]] )
  else:
    M = np.array( [[ 695.9951171875, 0.0, 640.0],[ 0.0, 695.9951171875, 360.0],[ 0.0, 0.0, 1.0]] )
  return M

def backproject(depth_cv, intrinsic_matrix, return_finite_depth=True, return_selection=False):

  depth = depth_cv.astype(np.float32, copy=True)

  # get intrinsic matrix
  K = intrinsic_matrix
  Kinv = np.linalg.inv(K)

  # compute the 3D points
  width = depth.shape[1]
  height = depth.shape[0]

  # construct the 2D points matrix
  x, y = np.meshgrid(np.arange(width), np.arange(height))
  ones = np.ones((height, width), dtype=np.float32)
  x2d = np.stack((x, y, ones), axis=2).reshape(width*height, 3)

  # backprojection
  R = np.dot(Kinv, x2d.transpose())

  # compute the 3D points
  X = np.multiply(np.tile(depth.reshape(1, width*height), (3, 1)), R)
  X = np.array(X).transpose()
  if return_finite_depth:
    selection = np.isfinite(X[:, 0])
    X = X[selection, :]

  if return_selection:
    return X, selection
        
  return X

def callback(depth_sub, info):

  bridge = CvBridge()
  try:
    cv_image = bridge.imgmsg_to_cv2(depth_sub, "16UC1")
  except CvBridgeError as e:
    print(e)

  # Recize the image from 1080x720 to 640x480
  resized_cv_image = cv2.resize(cv_image, (640, 480))

  cv2.imshow("Image window", resized_cv_image)
  cv2.waitKey(3)

  # Convert CV to 'ndarray'
  depth = np.asarray(resized_cv_image, dtype = np.float32)
  K = array_to_matrix_np(info.K)

  # Removing points that are farther than 1 meter or missing depth values
  depth[depth == 0] = np.nan
  # depth[depth > 1] = np.nan
  print(depth)

  pc, selection = backproject(depth, K, return_finite_depth=True, return_selection=True)  

def main(args):
  # Subscribe to Topics
  image_depth_sub = mf.Subscriber("my_gen3/depth/image_raw", Image)
  camera_info_sub = mf.Subscriber("my_gen3/depth/camera_info", CameraInfo)
  # Subscriptions Synchronizer
  ts = mf.TimeSynchronizer([image_depth_sub, camera_info_sub], 10)
  ts.registerCallback(callback)
  # Initialize Node
  rospy.init_node('image_converter', anonymous=True)
  # Process Execution
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)