#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

import numpy as np
from scipy.linalg import lstsq

from gpd.msg import CloudIndexed
from std_msgs.msg import Header, Int64
from geometry_msgs.msg import Point

cloud = [] # global variable to store the point cloud

def cloudCallback(msg):
    # Get the point cloud msg from preprocessing topic
    global cloud
    if len(cloud) == 0:
        for p in point_cloud2.read_points(msg,skip_nans=True):
            cloud.append([p[0], p[1], p[2]])
    cloud = np.asarray(cloud)
    X = cloud
    print(cloud)

    # Create CloudIndexed msg from remaining point
    msg = CloudIndexed()
    header = Header()
    header.frame_id = "/kinect2_link"
    header.stamp = rospy.Time.now()
    msg.cloud_sources.cloud = point_cloud2.create_cloud_xyz32(header, cloud.tolist())
    msg.cloud_sources.view_points.append(Point(0,0,0))
    msg.cloud_sources.camera_source.append(Int64(0))
    for i in X[0]:
        msg.indices.append(Int64(i))    

    # Hit key to publish topic
    s = raw_input('Hit [ENTER] to publish')
    pub.publish(msg)
    rospy.sleep(1)
    print 'Published cloud with', len(msg.indices), 'indices'

# Create a ROS node.
rospy.init_node('select_grasp')

# Subscribe to the ROS topic that contains the grasps.
cloud_sub = rospy.Subscriber('/segmentation_output', PointCloud2, cloudCallback)
pub = rospy.Publisher('/cloud_indexed', CloudIndexed, queue_size=1)

# Wait for point cloud to arrive.
while len(cloud) == 0:
    rospy.sleep(0.01)

rospy.spin()
