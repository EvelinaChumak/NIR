import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import registration
from cloud_preparation import read_clouds_target_sample


threshold = 0.02
    
down_sample = 1
delta = 0.01

def fist_prepair():
    """
    Функция первого прогона алгоритма для определения сдвига облака точек
    """
    change = []
    correspodence_1 = []
    delta = 0.01
    low=-0.5
    high=0.5
    rotation = 1
    while high > 0:
        correspodence_2 = []
        for i in range (0, 10):
            correspodence_3 = []
            for j in range (0,5):
                
                source, target = read_clouds_target_sample(down_sample, o3d.data.LivingRoomPointClouds().paths[i])
                trans_init = registration.random_transform(low=low, high=high, rotation=rotation)

                transformation = registration.ICP_registration(source, target, threshold, -trans_init)
                correspodence_3.append(registration.evalution(source, target, threshold, transformation))
                print(registration.evalution(source, target, threshold, transformation))
                
            correspodence_2.append(np.mean(correspodence_3))     
        
        correspodence_1.append(np.mean(correspodence_2))    
        change.append(high)
        low = low + delta
        high = high - delta
        rotation = rotation - delta

    plt.plot(change, correspodence_1)
    plt.xlabel('Change')
    plt.ylabel('Correspondese')
    plt.minorticks_on()
    plt.show()

def second_prepair():
    """
    Функция второго прогона алгоритма для определения сдвига облака точек
    """
    change = []
    correspodence_1 = []
    low=-0.1
    high=0.1
    rotation = 0.1
    while rotation > 0:
        correspodence_2 = []
        for i in range (0, 10):
            correspodence_3 = []
            for j in range (0,5):
                
                source, target = read_clouds_target_sample(down_sample, o3d.data.LivingRoomPointClouds().paths[i])
                trans_init = registration.random_transform(low=low, high=high, rotation=rotation)

                transformation = registration.ICP_registration(source, target, threshold, -trans_init)
                correspodence_3.append(registration.evalution(source, target, threshold, transformation))
                print(registration.evalution(source, target, threshold, transformation))
                
            correspodence_2.append(np.mean(correspodence_3))     
        
        correspodence_1.append(np.mean(correspodence_2))    
        change.append(high)
        low = low + delta
        high = high - delta
        rotation = rotation - delta

    plt.plot(change, correspodence_1)
    plt.xlabel('Change')
    plt.ylabel('Correspondese')
    plt.minorticks_on()
    plt.show()

def cloud_size():
    sizes = []
    for i in range(0,57):
        source, _ = read_clouds_target_sample(down_sample, o3d.data.LivingRoomPointClouds().paths[i])
        size = np.asarray(source.points).size / 3
        print(f'{i}: {size}')
        sizes.append(size)
    sizes = sorted(sizes)
    print(sizes)


def cloud_density():
    density = []
    for i in range(0,57):
        source, _ = read_clouds_target_sample(down_sample, o3d.data.LivingRoomPointClouds().paths[i])
        neighbor_distance = np.asarray(source.compute_nearest_neighbor_distance())
        cloud_density = neighbor_distance.sum()/len(neighbor_distance)
        print(f'{i}: {cloud_density}')
        density.append(cloud_density)
    print(density)
