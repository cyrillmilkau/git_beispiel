import open3d as o3d
import numpy as np

pcd=o3d.io.read_point_cloud('C:\\Users\\BLIKA389\\Desktop\\2022-02-15_gewoelbe2_full.ply')
pcd.estimate_normals()
pcd.orient_normals_consistent_tangent_plane(10000)
#o3d.visualization.draw_geometries([pcd])

#poisson mesh
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    poisson_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=11)


#remove low density vertices
densities = np.asarray(densities)
vertices_to_remove = densities < np.quantile(densities, 0.5)
poisson_mesh.remove_vertices_by_mask(vertices_to_remove)
# poisson_mesh.paint_uniform_color([0.5,0.5,0.5])

o3d.visualization.draw_geometries([poisson_mesh],mesh_show_wireframe=True, mesh_show_back_face=True)
# o3d.io.write_triangle_mesh("gewoelbe2_poisson.ply", poisson_mesh)