#!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
 
class DepthCropToPointCloud:
    def __init__(self):
        rospy.init_node('depth_crop_to_pointcloud')
 
        # Set up parameters for cropping
        self.crop_x = rospy.get_param('~crop_x', 100)
        self.crop_y = rospy.get_param('~crop_y', 50)
        self.crop_width = rospy.get_param('~crop_width', 200)
        self.crop_height = rospy.get_param('~crop_height', 200)
 
        self.bridge = CvBridge()
 
        # Subscribers
        self.sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.sub_info = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.info_callback)
 
        # Publisher
        self.pub_cloud = rospy.Publisher("/camera/depth/points_cropped", PointCloud2, queue_size=1)
 
        # Camera intrinsics
        self.camera_model_ready = False
 
    def info_callback(self, msg):
        # Store original intrinsics
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        self.camera_model_ready = True
        rospy.loginfo_once("Camera intrinsics received.")
 
    def depth_callback(self, msg):
        if not self.camera_model_ready:
            rospy.logwarn_throttle(5.0, "Waiting for camera_info...")
            return
 
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"CV bridge error: {e}")
            return
 
        height, width = depth_image.shape
 
        # Check bounds
        if (self.crop_x + self.crop_width > width) or (self.crop_y + self.crop_height > height):
            rospy.logerr("Crop area exceeds image bounds!")
            return
 
        # Crop the image
        depth_cropped = depth_image[
            self.crop_y : self.crop_y + self.crop_height,
            self.crop_x : self.crop_x + self.crop_width
        ]
 
        # Adjust intrinsics for the cropped image
        fx = self.fx
        fy = self.fy
        cx = self.cx - self.crop_x
        cy = self.cy - self.crop_y
 
        # Generate (u, v) pixel grids
        crop_h, crop_w = depth_cropped.shape
        u, v = np.meshgrid(np.arange(crop_w), np.arange(crop_h))
        u = u.flatten()
        v = v.flatten()
        z = depth_cropped.flatten()
 
        valid = z > 0  # Filter out invalid depth
 
        u = u[valid]
        v = v[valid]
        z = z[valid]
 
        # Project to 3D
        x = (u - cx) * (z/1000) / fx
        y = (v - cy) * (z/1000) / fy
 
        points = np.stack((x, y, z/1000), axis=-1)
 
        # Create PointCloud2
        header = msg.header
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
 
        cloud_msg = point_cloud2.create_cloud(header, fields, points)
        self.pub_cloud.publish(cloud_msg)
 
if __name__ == '__main__':
    try:
        node = DepthCropToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass