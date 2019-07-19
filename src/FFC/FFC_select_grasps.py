#!/usr/bin/env python

from geometry_msgs.msg import Point
from std_msgs.msg import Header, Int64
from gpd.msg import CloudIndexed
from scipy.linalg import lstsq
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np

cloud = []  # global variable to store the point cloud

def cloudCallback(msg):
    global cloud
    if len(cloud) == 0:
        for p in point_cloud2.read_points(msg):
            cloud.append([p[0], p[1], p[2]])

# Create a ROS node.
rospy.init_node('select_grasp')

# Subscribe to the ROS topic that contains the grasps.
cloud_sub = rospy.Subscriber('/voxel_output', PointCloud2, cloudCallback)

while not rospy.is_shutdown():
    # Wait for point cloud to arrive.
    while len(cloud) == 0:
        rospy.sleep(0.01)

    cloud = np.asarray(cloud)
    A = np.c_[cloud[:, 0], cloud[:, 1], np.ones(cloud.shape[0])]
    b = cloud[:, 2]
    C, _, _, _ = lstsq(A, b) # coefficients of the form: a*x + b*y + c*z + d = 0.
    a, b, c, d = C[0], C[1], -1., C[2]
    dist = ((a*cloud[:, 0] + b*cloud[:, 1] + d) - cloud[:, 2])**2
    err = dist.sum()
    idx = np.where(dist > 0.01)

    # Publish point cloud and nonplanar indices.
    pub = rospy.Publisher('/cloud_indexed', CloudIndexed, queue_size=1)

    msg = CloudIndexed()
    header = Header()
    header.frame_id = "/kinect2_link"
    header.stamp = rospy.Time.now()
    msg.cloud_sources.cloud = point_cloud2.create_cloud_xyz32(header, cloud.tolist())
    msg.cloud_sources.view_points.append(Point(0, 0, 0))

    for i in xrange(cloud.shape[0]):
        msg.cloud_sources.camera_source.append(Int64(0))
    
    for i in idx[0]:
        msg.indices.append(Int64(i))

    print '-----------------------------------'
    print 'Press hit key to publish... (Y/N)'
    in_content = raw_input('Input:')
    if in_content == 'Y' or in_content == "y":
        pub.publish(msg)
        print 'Published cloud with', len(msg.indices), 'indices'
    elif in_content == 'N' or in_content == 'n':
        exit(0)
    else:
        print 'Wrong key.'
        exit(0)

    rospy.sleep(2)
