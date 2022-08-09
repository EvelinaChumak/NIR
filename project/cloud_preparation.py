import open3d as o3d
import numpy as np

def read_clouds_target_sample(down_sample, data):
    demo_icp_pcds = data
    source = o3d.io.read_point_cloud(demo_icp_pcds)
    target = o3d.io.read_point_cloud(demo_icp_pcds).random_down_sample(down_sample)
    return source, target

def read_clouds_source_sample(down_sample, data):
    demo_icp_pcds = data
    source = o3d.io.read_point_cloud(demo_icp_pcds).random_down_sample(down_sample)
    target = o3d.io.read_point_cloud(demo_icp_pcds)
    return source, target
