import numpy as np
import open3d as o3d


def rad2deg(rad):
    return 180.0 * rad / np.pi


def deg2rad(deg):
    return np.pi * deg / 180.0


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def filter_by_normal_angle(pcd: o3d.geometry.PointCloud, angle_deg) -> o3d.geometry.PointCloud:
    normals_arr = np.asarray(pcd.normals)
    points_arr = np.asarray(pcd.points)

    normals_filter_arr = np.empty((0, 3))
    points_filter_arr = np.empty((0, 3))
    z_direction = (0, 0, 1)

    for i in range(normals_arr.shape[0]):
        norm = normals_arr[i]
        pnt = points_arr[i]
        a = (norm[0], norm[1], norm[2])
        b = z_direction
        if angle_between(a,b) < deg2rad(angle_deg):
            normals_filter_arr = np.append(normals_filter_arr, np.array([norm]), axis=0)
            points_filter_arr  = np.append(points_filter_arr,  np.array([pnt]), axis=0)

    pcd_filter = o3d.geometry.PointCloud()
    pcd_filter.points = o3d.utility.Vector3dVector(points_filter_arr)
    pcd_filter.normals = o3d.utility.Vector3dVector(normals_filter_arr)

    return pcd_filter
