#!/usr/bin/env python

import numpy as np 

import pickle
import rospy 
from imagecontrol.features import compute_color_histograms
from imagecontrol.features import compute_normal_histograms

from imagecontrol.pcl_helper import *

import pcl

from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, PointField
from imagecontrol.srv import GetNormals
from imagecontrol.srv import GetCloud


print("not needed")
models = ['stair', 'moveable_chair' , 'static_chair' , 'table_fan', 'i_dont_know']

def get_normals(cloud):
    get_normal_prox = rospy.ServiceProxy('get_normals' , GetNormals)
    return get_normal_prox(cloud).cluster

def capture_sample():
    return rospy.wait_for_message('/Non_plane', PointCloud2)
# def pcl_callback(pcl_msg):
#     cloud_filter = ros_to_pcl(pcl_msg)

#     #print(cloud_filter)
#     global something
#     global labeled_features
#     print(pcl_callback)
#     while(something):
#         for model_name in models:
#             for i in range(30):
#                 x = int(input())
#                 if(x == 1):
#                     print("main to aa gya")
#                     data = capture_sample()
#                     print("yaha phucha kya ??")
#                     c_hists = compute_color_histograms(data , using_hsv = True)
#                     print(c_hists)
#                     normals = get_normals(data)
#                     n_hist = compute_normal_histograms(normals)
#                     print(n_hist)
#                     print(len(c_hists) , len(n_hist))
#                     total_features = np.concatenate((n_hist , c_hists))
#                     labeled_features.append([total_features, model_name])
#                 else:
#                     pass
#                 print('got the item')
#                 print(i +1  , model_name)
#             print("latest model has been completed")
#             print(model_name)
#         print("writing")
#         pickle.dump(labeled_features, open('new_data_all.sav', 'wb'))
#         something = False




    # while(something):
    #     for j in range(3):
    #         for i in range(10):
    #             x = int(input())
    #             if(x == 1):
    #                 print("hi kumar")
    #             else:
    #                 pass
    #             print("U completed")
    #         print("i also compled")
    #     print("its written")
    #     something = False







if __name__ == "__main__":
    rospy.init_node('clustering' , anonymous=True)

    #pcl_sub = rospy.Subscriber("/Non_plane" , pc2.PointCloud2 , pcl_callback , queue_size = 1)

    labeled_features = []

    capture_attempts = 30
    histogram_bins = 64

    for model_name in models:
        
        print(model_name)
        for i in range(capture_attempts):
            # make five attempts to get a valid a point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                # Capture the point cloud using the sensor stock RGBD camera
                print("yes looking ofr data")
                x = int(input())
                if (x== 1):
                    sample_cloud = capture_sample()
                    # Convert the ros cloud to a pcl cloud
                    print("yehhhh got the data")
                    sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()
                else:
                    pass
                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True

            # Extract histogram features
            chists = compute_color_histograms(sample_cloud,
                                              nbins=histogram_bins,
                                              using_hsv=True)
            print("got hte normal ??????")
            normals = get_normals(sample_cloud)
            print("yes get the normal")
            nhists = compute_normal_histograms(normals,
                                               nbins=histogram_bins)
            print(chists , nhists)
            feature = np.concatenate((chists, nhists))
            labeled_features.append([feature, model_name])
        print("model sample has been collected , please insert a  new model")
        print(model_name)
        

    pickle.dump(labeled_features, open('training_set_new.sav', 'wb'))

    

