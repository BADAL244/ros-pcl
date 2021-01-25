#!/usr/bin/env python

import numpy as np
import pcl
import pickle
import sklearn
from sklearn.preprocessing import LabelEncoder

from imagecontrol.srv import GetNormals
from imagecontrol.features import compute_color_histograms
from imagecontrol.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from imagecontrol.marker_tools import *
from imagecontrol.msg import DetectedObjectsArray
from imagecontrol.msg import DetectedObject
from imagecontrol.pcl_helper import *

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('get_normals',
                                          GetNormals)
    return get_normals_prox(cloud).cluster

def pcl_callback(pcl_msg):


    # Convert ROS msg to PCL data (XYZRGB)
    ros_cloud = ros_to_pcl(pcl_msg)

   
        
    # Store the detected objects and labels in these lists
    detected_objects_labels = []
    detected_objects = []
    color_cluster_point_list = []


    # Extract histogram features (similar to capture_features.py)
    histogram_bins = 64
    chists = compute_color_histograms(ros_cloud,
                                        nbins=histogram_bins,
                                        using_hsv=True)
    normals = get_normals(ros_cloud)
    nhists = compute_normal_histograms(normals,
                                        nbins=histogram_bins)
    feature = np.concatenate((chists, nhists))

    # Make the prediction, retrieve the label for the result and add it
    #   to detected_objects_labels list
    prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
    label = encoder.inverse_transform(prediction)[0]
    detected_objects_labels.append(label)

    # Publish a label into RViz
    label_pos = list(ros_cloud[pts_list[0]])
    label_pos[2] += .4
    object_markers_pub.publish(make_label(label, label_pos, index))

    # Add the detected object to the list of detected objects.
    do = DetectedObject()
    do.label = label
    do.cloud = ros_cloud
    detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    
    # Create new cloud containing all clusters, each with a unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_object_cluster = pcl_to_ros(cluster_cloud)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)

    # Publish ROS messages of the point clouds and detected objects
    pcl_objects_cloud_pub.publish(ros_cloud_object_cluster) # solid color objects
    pcl_objects_pub.publish(ros_cloud_objects)      # original color objects
    pcl_table_pub.publish(ros_cloud_table)          # table cloud
    detected_objects_pub.publish(detected_objects)  # detected object labels

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('recognition', anonymous=True)

    # Create Subscriber to receive the published data coming from the
    #   pcl_callback() function that will be processing the point clouds
    pcl_sub = rospy.Subscriber('/sensor_stick/point_cloud', pc2.PointCloud2,
                               pcl_callback, queue_size=1)

    # Create Publishers
    object_markers_pub = rospy.Publisher('/object_markers', Marker,
                                         queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects',
                                           DetectedObjectsArray,
                                           queue_size=1)
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_objects_cloud_pub = rospy.Publisher('/pcl_objects_cloud', PointCloud2,
                                            queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)

    # Load model from disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
