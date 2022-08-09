import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
from draw_result import draw_registration_result
import registration
from cloud_preparation import read_clouds_source_sample, read_clouds_target_sample
from time import time

trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0, 0.0], 
                         [0.0, 0.0, 0.0, 1.0]])
threshold = 0.02


def show_correspondese_down_sample_from_1_to_0_one_cloud():   
    down_sample = 1
    down_sample_array = []
    correspondese_array = []
    time_array = []
    while down_sample > 0:
        down_sample_array.append(down_sample)
        time_array_2 = []
        correspondese_array_2 = []
        for j in range (0,3):
            start_time = time()    
            source, target = read_clouds_target_sample(down_sample, o3d.data.LivingRoomPointClouds().paths[1])
            target.transform(registration.random_transform())
            transformation = registration.ICP_registration(source, target, threshold, -trans_init)
            correspondese_array_2.append(np.asarray(registration.evalution(source, target, threshold, transformation)))
            time_array_2.append(time() - start_time)
        time_array.append(np.mean(time_array_2))
        correspondese_array.append(np.mean(correspondese_array_2))

        down_sample = down_sample - 0.01

    plt.plot(down_sample_array, correspondese_array)
    plt.xlabel('Down sample')
    plt.ylabel('Correspondese')
    plt.show()
    plt.plot(down_sample_array, time_array)
    plt.xlabel('Down sample')
    plt.ylabel('Time')
    plt.show()

def show_correspondese_down_sample_from_1_to_0_ten_cloud():   
    down_sample = 1
    down_sample_array = []
    correspondese_array = []
    time_array = []
    while down_sample > 0:
        down_sample_array.append(down_sample)
        time_array_2 = []
        correspondese_array_2 = []
        for i in range(0,10): 
            time_array_3 = []
            correspondese_array_3 = []
            for j in range (0,3):
                start_time = time()    
                source, target = read_clouds_target_sample(down_sample, o3d.data.LivingRoomPointClouds().paths[i])
                target.transform(registration.random_transform())
                transformation = registration.ICP_registration(source, target, threshold, -trans_init)
                correspondese_array_3.append(np.asarray(registration.evalution(source, target, threshold, transformation)))
                time_array_3.append(time() - start_time)
            correspondese_array_2.append(np.mean(correspondese_array_3))
            time_array_2.append(np.mean(time_array_3))

        time_array.append(np.mean(time_array_2))
        correspondese_array.append(np.mean(correspondese_array_2))

        down_sample = down_sample - 0.01

    plt.plot(down_sample_array, correspondese_array)
    plt.xlabel('Down sample')
    plt.ylabel('Correspondese')
    plt.show()
    plt.plot(down_sample_array, time_array)
    plt.xlabel('Down sample')
    plt.ylabel('Time')
    plt.show()


def show_correspondese_for_different_cloud():
    down_sample = 0.33
    size_1 = []
    corresponce_1 = []
    for i in range(0,57): 
        size_2 = []
        corresponce_2 = []
        for j in range (0,1): 
            source, target = read_clouds_target_sample(down_sample, o3d.data.LivingRoomPointClouds().paths[i])
            target.transform(registration.random_transform())
            transformation = registration.ICP_registration(source, target, threshold, -trans_init)
            corresponce_2.append(np.asarray(registration.evalution(source, target, threshold, transformation)))
            print(registration.evalution(source, target, threshold, transformation))
            size_2.append(np.asarray(source.points).size / 3)
        corresponce_1.append(np.mean(corresponce_2))
        size_1.append(np.mean(size_2))
    
    plt.scatter(size_1, corresponce_1)
    plt.xlabel('Point size')
    plt.ylabel('Correspondese')
    plt.show()

def show_cloud_density_and_size():
    s = []
    d = []
    for i in range(0,57):
        source, _ = read_clouds_target_sample(1, o3d.data.LivingRoomPointClouds().paths[i])
        size = np.asarray(source.points).size / 3
        s.append(size)

        neighbor_distance = np.asarray(source.compute_nearest_neighbor_distance())
        cloud_density = neighbor_distance.sum()/len(neighbor_distance)
        d.append(cloud_density)
        
    plt.scatter(s, d)
    plt.xlabel('Point size')
    plt.ylabel('Density')
    plt.show()
    

def show_correspondese_with_coef():
    size_1 = []
    corresponce_1 = []
    t = []
    for i in range(0,57): 
        size_2 = []
        corresponce_2 = []
        t_2 = []
        for j in range (0,1): 
            source, target = read_clouds_target_sample(1, o3d.data.LivingRoomPointClouds().paths[i])
            neighbor_distance = np.asarray(source.compute_nearest_neighbor_distance())
            coeff = 0.5-(1/neighbor_distance.sum()*100)
            print(coeff)
            start_time = time()
            source, target = read_clouds_target_sample(coeff, o3d.data.LivingRoomPointClouds().paths[i])
            target.transform(registration.random_transform())

            transformation = registration.ICP_registration(source, target, threshold, -trans_init)
            corresponce_2.append(np.asarray(registration.evalution(source, target, threshold, transformation)))
            size_2.append(np.asarray(source.points).size / 3)
            print((time() - start_time))
            t_2.append((time() - start_time))
        corresponce_1.append(np.mean(corresponce_2))
        size_1.append(np.mean(size_2))
        t.append(np.mean(t_2))
    
    plt.scatter(size_1, corresponce_1)
    plt.xlabel('Point size')
    plt.ylabel('Correspondese')
    plt.show()
    plt.scatter(size_1, t)
    plt.xlabel('Point size')
    plt.ylabel('Time')
    plt.show()
    

def show_correspondese_source_sample():
    source, target = read_clouds_source_sample(0.02, o3d.data.LivingRoomPointClouds().paths[1])

    source.transform(registration.random_transform())
    draw_registration_result(source, target, trans_init)
    print(registration.evalution(source, target, threshold, trans_init))
    transformation = registration.ICP_registration(source, target, threshold, -trans_init)
    draw_registration_result(source, target, transformation)
    print(registration.evalution(source, target, threshold, transformation))
