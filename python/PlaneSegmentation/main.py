import sys
import time
import numpy as np
import open3d as o3d
import matplotlib
import matplotlib.pyplot as plt
from tqdm import tqdm

from filter import rad2deg, deg2rad, unit_vector, angle_between, filter_by_normal_angle
from PlaneSegmentationByNormals import PlaneSegmentationByNormals

def main():
    pcd_path = "e:\\data\\02_fokus\\BspDaten\\Langenleuba\\"
    pcd_file = "SLOS-Innen_12mm_merge_sub_10_cm.pcd"
    n_planes = 25
    epsilon_deg = 10.0
    epsilon_deg_min = 45.0
    epsilon_deg_max = 135.0
    epsilon_cond_use = False

    pcd = o3d.io.read_point_cloud(pcd_path + pcd_file)
    print("pcd: " + str(np.asarray(pcd.points).size))
    # pcd_filter = filter_by_normal_angle(pcd, 45.0)
    # print("pcd_filter: "+ str(np.asarray(pcd_filter.points).size))

    # cmap = plt.get_cmap("viridis")
    # norm = matplotlib.colors.Normalize(vmin=0, vmax=n_planes)
    # geometries = []
    # with tqdm(total=1) as pbar:
    #     for i in range(n_planes):
    #         rgba = cmap(norm(i))
    #         break_cond = False
    #         epsilon_cond = False
    #         if (epsilon_cond_use):
    #             max_iter = 0
    #             if (max_iter >= 1000):
    #                 break_cond = True
    #             # timeout = time.time() + 30 * 1  # 5 minutes from now
    #             while (not epsilon_cond):
    #                 # time.sleep(1)
    #                 max_iter += 1
    #                 plane_model, inliers = pcd_filter.segment_plane(distance_threshold=0.10,
    #                                                          ransac_n=3,
    #                                                          num_iterations=1000)
    #                 [a, b, c, d] = plane_model
    #                 if (angle_between((a,b,c),(0,0,1)) < deg2rad(epsilon_deg)) or break_cond:
    #                     # print(rad2deg(angle_between((a,b,c),(0,0,1))))
    #                     epsilon_cond = True
    #                     inlier_cloud = pcd.select_by_index(inliers)
    #                     inlier_cloud.paint_uniform_color([rgba[0], rgba[1], rgba[2]])
    #                     geometries.append(inlier_cloud)
    #                     pcd = pcd.select_by_index(inliers, invert=True)
    #                 # if ((angle_between((a,b,c),(0,0,1)) > deg2rad(epsilon_deg_min)) and ((angle_between((a,b,c),(0,0,1)) < deg2rad(epsilon_deg_max))) ) or break_cond:
    #                 #     epsilon_cond = True
    #                 #     inlier_cloud = pcd.select_by_index(inliers)
    #                 #     inlier_cloud.paint_uniform_color([rgba[0], rgba[1], rgba[2]])
    #                 #     geometries.append(inlier_cloud)
    #                 #     pcd = pcd.select_by_index(inliers, invert=True)
    #                 # if time.time() > timeout:
    #                 #     epsilon_cond = True
    #                 #     inlier_cloud = pcd.select_by_index(inliers)
    #                 #     inlier_cloud.paint_uniform_color([rgba[0], rgba[1], rgba[2]])
    #                 #     # geometries.append(inlier_cloud) # no selection after timeout
    #                 #     # pcd = pcd.select_by_index(inliers, invert=True) # no deletion after timeout
    #         else:
    #             plane_model, inliers = pcd_filter.segment_plane(distance_threshold=0.05,
    #                                                      ransac_n=3,
    #                                                      num_iterations=1000)
    #             [a, b, c, d] = plane_model
    #             # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    #             inlier_cloud = pcd.select_by_index(inliers)
    #             inlier_cloud.paint_uniform_color([rgba[0],rgba[1],rgba[2]])
    #             geometries.append(inlier_cloud)
    #         pbar.update(1 / n_planes)

    # PlaneSegmentationByNormals().process("ALL", pcd, 25)
    # PlaneSegmentationByNormals().process2("WALL", pcd, 4, 20, 160)
    PlaneSegmentationByNormals().process2("FLOOR", pcd, 4, -20, 20)



if __name__ == '__main__':
    main()