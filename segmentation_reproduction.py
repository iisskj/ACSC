import numpy as np
import open3d as o3d

def region_growing_kernel(input, groundRemoval, numberOfNeighbours, 
                           minRegionSize, maxRegionSize, smoothnessThreshold, curvatureThreshold):
    ref_input = np.ascontiguousarray(input)
    # 初始化pointcloud 数量是输入的numpy array中的point数量
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(ref_input[:, :3])
    cloud.colors = o3d.utility.Vector3dVector(np.expand_dims((ref_input[:, 3] / 255.0), 1).repeat(3, axis=1))

    # segmentation
    parameter = o3d.geometry.SegmentationParameter(
        groundRemoval=groundRemoval, 
        numberOfNeighbours=numberOfNeighbours, 
        minRegionSize=minRegionSize, 
        maxRegionSize=maxRegionSize, 
        smoothnessThreshold=smoothnessThreshold, 
        curvatureThreshold=curvatureThreshold
    )
    clusters = o3d.geometry.region_growing(cloud, parameter)

    # calc available point count
    useful_point_count = 0
    for indices in clusters:
        useful_point_count += len(indices.indices)

    # results
    data_field = 5
    result = np.empty((useful_point_count, data_field), dtype=np.float32)

    # output point id
    point_id = 0
    for label_id, indices in enumerate(clusters):
        for indice in indices.indices:
            buf_index_base = point_id * data_field
            result[buf_index_base + 0] = cloud.points[indice][0]
            result[buf_index_base + 1] = cloud.points[indice][1]
            result[buf_index_base + 2] = cloud.points[indice][2]
            result[buf_index_base + 3] = cloud.colors[indice][0]
            result[buf_index_base + 4] = label_id
            point_id += 1

    return result


import open3d as o3d
import numpy as np


def region_growing_segmentation(pc, voxel_size=0.05, curvature_threshold=1.0, smoothness_threshold=0.2, min_cluster_size=50):
    """
    Segment point cloud using region growing algorithm.

    :param pc: numpy array of shape (n, 4) representing a point cloud. The first 3 columns represent the (x, y, z)
               coordinates of each point, and the 4th column represents the intensity value.
    :param voxel_size: float value indicating the size of voxel for downsampling. Default value is 0.05.
    :param curvature_threshold: float value indicating the maximum allowable difference in curvature between points
                                in a cluster. Default value is 1.0.
    :param smoothness_threshold: float value indicating the maximum allowable difference in normals between points
                                 in a cluster. Default value is 0.2.
    :param min_cluster_size: int value indicating the minimum number of points that a cluster must have. Default value
                             is 50.
    :return: numpy array of shape (n, 5) representing the segmented point cloud. The first 3 columns represent the
             (x, y, z) coordinates of each point, the 4th column represents the intensity value, and the 5th column
             represents the label of the cluster to which the point belongs.
    """
    # Create Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc[:, :3])
    # pcd.colors = o3d.utility.Vector3dVector(np.ones_like(pc[:, :3]))  # Set all points to white
    pcd.colors = o3d.utility.Vector3dVector(np.expand_dims((pc[:, 3] / 255.0), 1).repeat(3, axis=1))
    pcd.normals = o3d.utility.Vector3dVector(np.zeros_like(pc[:, :3]))  # Initialize normals to 0

    # Compute normals
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))

    # Perform region growing segmentation
    labels = np.array(pcd.cluster_dbscan(eps=voxel_size, min_points=min_cluster_size, print_progress=True))

    # Assign intensity values and return results as numpy array
    segmented_pc = np.zeros((len(pc), 5))
    segmented_pc[:, :3] = np.asarray(pcd.points)
    segmented_pc[:, 3] = pc[:, 3]  # Intensity values
    segmented_pc[:, 4] = labels

    return segmented_pc
