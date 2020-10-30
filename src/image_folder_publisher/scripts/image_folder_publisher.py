#!/usr/bin/env python
from __future__ import print_function

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import rospy
from os.path import isfile, join
from os import listdir
import os
import sys
from sensor_msgs.msg import CameraInfo
import yaml
import roslib
import rospkg

roslib.load_manifest('image_folder_publisher')


class image_folder_publisher:
    def __init__(self):
        rospack = rospkg.RosPack()
        
        self.img_saved = True
        self.img_file_name = ""

        self.__app_name = "image_folder_publisher"
        print("path = {}", rospack.get_path(self.__app_name))

        self._cv_bridge = CvBridge()

        self._topic_name = rospy.get_param('~topic_name', '/image_raw')
        rospy.loginfo("[%s] (topic_name) Publishing Images to topic  %s",
                      self.__app_name, self._topic_name)

        self._image_publisher = rospy.Publisher(
            self._topic_name, Image, queue_size=1)

        self._seg_image_sub = rospy.Subscriber("/deeplab/segmentation_colored", Image, self.callback)

        
        self._topic_name_camera_info = rospy.get_param('~topic_name_camera_info', '/camera_info')
        rospy.loginfo("[%s] (topic_name) Publishing camera info to topic  %s",
                      self.__app_name, self._topic_name_camera_info)

        self._cam_info_publisher = rospy.Publisher(
            self._topic_name_camera_info, CameraInfo, queue_size=1)

        self._rate = rospy.get_param('~publish_rate', 1)
        rospy.loginfo(
            "[%s] (publish_rate) Publish rate set to %s hz", self.__app_name, self._rate)

        self._sort_files = rospy.get_param('~sort_files', True)
        rospy.loginfo("[%s] (sort_files) Sort Files: %r",
                      self.__app_name, self._sort_files)

        self._frame_id = rospy.get_param('~frame_id', 'camera')
        rospy.loginfo("[%s] (frame_id) Frame ID set to  %s",
                      self.__app_name, self._frame_id)
        
        self._save_img_width = rospy.get_param('~save_img_width', '1920')
        rospy.loginfo("[%s] (save_img_width) set to  %s",
                      self.__app_name, self._save_img_width)
        
        self._save_img_height = rospy.get_param('~save_img_height', '1080')
        rospy.loginfo("[%s] (save_img_height) set to  %s",
                      self.__app_name, self._save_img_height)

        self._loop = rospy.get_param('~loop', 1)
        rospy.loginfo(
            "[%s] (loop) Loop  %d time(s) (set it -1 for infinite)", self.__app_name, self._loop)

        self._camera_info_path = rospy.get_param(
            '~cam_info', join(rospack.get_path(self.__app_name), 'yamls/realsenseRGB.yaml'))
        rospy.loginfo("[%s] (cam_info) Load camera info from %s", self.__app_name, self._camera_info_path)
        self._camera_info = self.yaml_to_CameraInfo(self._camera_info_path)

        self._image_folder = rospy.get_param('~input_image_folder', '')
        if self._image_folder == '' or not os.path.exists(self._image_folder) or not os.path.isdir(self._image_folder):
            rospy.logfatal(
                "[%s] (input_image_folder) Invalid Image folder", self.__app_name)
            sys.exit(0)
        rospy.loginfo("[%s] Reading images from %s",
                      self.__app_name, self._image_folder)
        
        self._output_image_folder = rospy.get_param('~output_image_folder', '')
        if not os.path.exists(self._output_image_folder):
            os.mkdir(self._output_image_folder)
            print("create {}".format(self._output_image_folder))
        else:
            print("{} already exist".format(self._output_image_folder))
        
        rospy.loginfo("[%s] Reading images from %s",
                      self.__app_name, self._image_folder)

    def callback(self, img):
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
        dim = (self._save_img_width, self._save_img_height)
        resized = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)
        save_path = join(self._output_image_folder, self.img_file_name)
        cv2.imwrite(save_path, resized)
        self.img_saved = True
        rospy.loginfo("Saved %s", save_path)


    def run(self):
        ros_rate = rospy.Rate(self._rate)
        files_in_dir = [f for f in listdir(
            self._image_folder) if isfile(join(self._image_folder, f))]
        if self._sort_files:
            files_in_dir.sort()
        try:
            while self._loop != 0:
                for f in files_in_dir:
                    if not rospy.is_shutdown():
                        if isfile(join(self._image_folder, f)):
                            self.img_file_name = f
                            cv_image = cv2.imread(join(self._image_folder, f))
                            if cv_image is not None:
                                ros_msg = self._cv_bridge.cv2_to_imgmsg(
                                    cv_image, "bgr8")
                                ros_msg.header.frame_id = self._frame_id
                                ros_msg.header.stamp = rospy.Time.now()
                                self._image_publisher.publish(ros_msg)
                                self._cam_info_publisher.publish(self._camera_info)
                                #rospy.loginfo("[%s] Published %s", self.__app_name, join(
                                #    self._image_folder, f))
                            else:
                                rospy.loginfo("[%s] Invalid image file %s", self.__app_name, join(
                                    self._image_folder, f))
                            ros_rate.sleep()
                    else:
                        return
                self._loop = self._loop - 1
        except CvBridgeError as e:
            rospy.logerr(e)

    def yaml_to_CameraInfo(self, yaml_fname):
        # Load data from file
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.load(file_handle)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        camName = calib_data["camera_name"]
        return camera_info_msg


def main(args):
    rospy.init_node('image_folder_publisher', anonymous=True)

    image_publisher = image_folder_publisher()
    image_publisher.run()


if __name__ == '__main__':
    main(sys.argv)
