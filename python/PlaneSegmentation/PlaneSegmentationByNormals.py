import numpy as np
import open3d as o3d
from tqdm import tqdm
from filter import deg2rad, angle_between
import matplotlib
import matplotlib.pyplot as plt


class PlaneSegmentationByNormals:

    def __init__(self):
        self.__choice_table = \
            {
                "ALL": self.segment_all_planes
            }

        self.__choice_table_epsilon = \
            {
                "WALL": self.segment_wall_planes,
                "FLOOR": self.segment_floor_planes
            }

    def process(self, case, pcd, n_planes):
        return self.__choice_table[case](pcd, n_planes)

    def segment_all_planes(self, pcd, n_planes):
        cmap = plt.get_cmap("viridis")
        norm = matplotlib.colors.Normalize(vmin=0, vmax=n_planes)
        geometries = []
        with tqdm(total=1) as pbar:
            for i in range(n_planes):
                plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,
                                                         ransac_n=3,
                                                         num_iterations=1000)
                [a, b, c, d] = plane_model
                # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
                inlier_cloud = pcd.select_by_index(inliers)
                rgba = cmap(norm(i))
                inlier_cloud.paint_uniform_color([rgba[0], rgba[1], rgba[2]])
                geometries.append(inlier_cloud)
                pcd = pcd.select_by_index(inliers, invert=True)
                pbar.update(1 / n_planes)
        o3d.visualization.draw_geometries(geometries)
        return

    def process2(self, case, pcd, n_planes, epsilon_deg_min, epsilon_deg_max):
        return self.__choice_table_epsilon[case](pcd, n_planes, epsilon_deg_min, epsilon_deg_max)

    def segment_wall_planes(self, pcd, n_planes, epsilon_deg_min, epsilon_deg_max):
        cmap = plt.get_cmap("viridis")
        norm = matplotlib.colors.Normalize(vmin=0, vmax=n_planes)
        geometries = []
        with tqdm(total=1) as pbar:
            for i in range(n_planes):
                epsilon_cond = False
                while not epsilon_cond:
                    plane_model, inliers = pcd.segment_plane(distance_threshold=0.10, ransac_n=3, num_iterations=1000)
                    [a, b, c, d] = plane_model
                    cond_1 = angle_between((a, b, c), (0, 0, 1)) > deg2rad(epsilon_deg_min)
                    cond_2 = angle_between((a, b, c), (0, 0, 1)) < deg2rad(epsilon_deg_max)
                    if cond_1 and cond_2:
                        epsilon_cond = True
                        inlier_cloud = pcd.select_by_index(inliers)
                        rgba = cmap(norm(i))
                        inlier_cloud.paint_uniform_color([rgba[0], rgba[1], rgba[2]])
                        geometries.append(inlier_cloud)
                        pcd = pcd.select_by_index(inliers, invert=True)
                pbar.update(1 / n_planes)
        o3d.visualization.draw_geometries(geometries)
        return

    def segment_floor_planes(self, pcd, n_planes, epsilon_deg_min, epsilon_deg_max):
        cmap = plt.get_cmap("viridis")
        norm = matplotlib.colors.Normalize(vmin=0, vmax=n_planes)
        geometries = []
        with tqdm(total=1) as pbar:
            for i in range(n_planes):
                epsilon_cond = False
                while not epsilon_cond:
                    plane_model, inliers = pcd.segment_plane(distance_threshold=0.10, ransac_n=3, num_iterations=1000)
                    [a, b, c, d] = plane_model
                    cond_1 = angle_between((a, b, c), (0, 0, 1)) > deg2rad(epsilon_deg_min)
                    cond_2 = angle_between((a, b, c), (0, 0, 1)) < deg2rad(epsilon_deg_max)
                    if cond_1 and cond_2:
                        epsilon_cond = True
                        inlier_cloud = pcd.select_by_index(inliers)
                        rgba = cmap(norm(i))
                        inlier_cloud.paint_uniform_color([rgba[0], rgba[1], rgba[2]])
                        geometries.append(inlier_cloud)
                        pcd = pcd.select_by_index(inliers, invert=True)
                pbar.update(1 / n_planes)
        o3d.visualization.draw_geometries(geometries)
        return
