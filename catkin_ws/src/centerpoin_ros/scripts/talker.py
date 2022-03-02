#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
from cv_bridge import CvBridge
import cv2
import struct
import os
import numpy as np
import open3d as o3d
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)

def bin_to_pcd(binFileName):
    size_float = 4
    list_pcd = []
    with open(binFileName, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z, intensity = struct.unpack("ffff", byte)
            list_pcd.append([x, y, z])
            byte = f.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_pcd)
    return pcd

def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="lidar"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    # if not open3d_cloud.colors: # XYZ only
    cloud_data=points
    # else: # XYZ + RGB
    #     fields=FIELDS_XYZRGB
    #     # -- Change rgb color from "three float" to "one 24-byte int"
    #     # 0x00FFFFFF is white, 0x00000000 is black.
    #     colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
    #     colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
    #     cloud_data=np.c_[points, colors]
    
    # create ros_cloud
    return pc2.create_cloud_xyz32(header, cloud_data)



class Node(object):
    def __init__(self):
        self.image = cv2.imread('/home/tony/Desktop/ROS_centerpoint/samples/CAM_BACK/n008-2018-08-01-15-16-36-0400__CAM_BACK__1533151603537558.jpg')
        self.lidar = bin_to_pcd('/home/tony/Desktop/ROS_centerpoint/samples/LIDAR_TOP/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151603547590.pcd.bin')
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub_image = rospy.Publisher('/test/image', Image,queue_size=1)
        self.pub_lidar = rospy.Publisher('/test/lidar', PointCloud2,queue_size=1)

        # Subscribers
        # rospy.Subscriber("/camera/image_color",Image,self.callback)

    def start(self):
        rospy.loginfo("Start publishing")
        #rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image and lidar')
            if self.image is not None:
                self.pub_image.publish(self.br.cv2_to_imgmsg(self.image,"bgr8"))
                self.pub_lidar.publish(convertCloudFromOpen3dToRos(self.lidar))
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = Node()
    my_node.start()