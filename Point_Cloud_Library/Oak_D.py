# -*- coding: utf-8 -*-
"""
Created on Tue May 30 09:14:10 2023

@author: Qikpod - Robotics
"""

import depthai
import numpy as np
import pcl

# Create a DepthAI pipeline
pipeline = depthai.Pipeline()

# Create a node to get the left camera frames
left_cam = pipeline.createMonoCamera()
left_cam.setBoardSocket(depthai.CameraBoardSocket.LEFT)

# Create a node to get the depth frames
depth = pipeline.createStereoDepth()
depth.setOutputDepth(True)
depth.setConfidenceThreshold(200)
left_cam.out.link(depth.left)

# Create a node to convert depth into disparity
disparity = pipeline.createDisparityDepth()
depth.disparity.link(disparity.inputDepth)

# Create a node to generate a point cloud
point_cloud = pipeline.createXLinkOut()
point_cloud.setStreamName("point_cloud")
disparity.depth.link(point_cloud.input)

# Start the pipeline
device = depthai.Device(pipeline)
device.startPipeline()

try:
    while True:
        # Get the next set of frames from the camera
        data = device.getOutputQueue(name="point_cloud", maxSize=1).get()
        point_cloud_data = data.getPointCloud()

        # Convert the point cloud data to a numpy array
        point_cloud_array = np.array(point_cloud_data.getPoints(), dtype=np.float32).reshape(-1, 3)

        # Convert the point cloud array to a PCL point cloud
        pcl_cloud = pcl.PointCloud()
        pcl_cloud.from_array(point_cloud_array)

        # Process the PCL point cloud as needed
        # For example, you can extract features, perform segmentation, etc.

        # Visualize the PCL point cloud
        pcl.visualization.CloudViewer('Point Cloud Viewer').showCloud(pcl_cloud)

except KeyboardInterrupt:
    pass

finally:
    # Stop the pipeline and clean up
    device.close()
