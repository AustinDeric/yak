[![Build Status](https://travis-ci.org/AustinDeric/yak.svg?branch=master)](https://travis-ci.org/AustinDeric/yak)

# yak 
yak (yet another kinfu) is a library and ros wrapper for truncated sign distance fields (TSDF). 

# yak_meshing
ROS package to mesh TSDF volumes generated by Kinect Fusion-like packages.

Meshing happens through the /GetMesh service, which in turn calls the kinfu_ros /GetTSDF service. yak_meshing_node expects a serialized TSDF voxel volume. OpenVDB's voxel meshing algorithm is used to produce a triangular mesh, which is saved as an .obj.
