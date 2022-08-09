import math
import open3d as o3d
import numpy as np
import random as rd

def evalution(source, target, threshold, trans_init):
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source, target, threshold, trans_init)
    return evaluation.fitness
    
def ICP_registration(source, target, threshold, trans_init, max_iteration=30):
    reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration))
    return reg_p2p.transformation
    
def random_transform(low=-0.02, high=0.02, rotation = 0.02):
    x = rd.uniform(-rotation, rotation)*math.pi/180
    y = rd.uniform(-rotation, rotation)*math.pi/180
    z = rd.uniform(-rotation, rotation)*math.pi/180
    
    cx, cy, cz = math.cos(x), math.cos(y), math.cos(z)
    sx, sy, sz = math.sin(x), math.sin(y), math.sin(z)
    
    return np.asarray([[cz*cy, -sy*sx*cz - sz*cx, -sy*cx*cz + sz*sx, rd.uniform(low,high)],
                       [sz*cy, -sy*sx*sz + cx*cz, -sy*cx*sz - sx*cz, rd.uniform(low,high)],
                       [   sy,             cy*sx,             cy*cx, rd.uniform(low,high)],
                       [  0.0,               0.0,               0.0,                 1.0]]) 
    