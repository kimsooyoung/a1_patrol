import numpy as np
import open3d as o3d
import argparse

parser = argparse.ArgumentParser(description="Parsing parameters for generate pcd map for ndt_localization ...")
parser.add_argument('--map_name', default='untitled', help='Name of your map')
args = parser.parse_args()

if __name__ == '__main__':
    raw_pcd = o3d.io.read_point_cloud("your path to original .pcd file")
    raw_pcd_np = np.asarray(raw_pcd.points)

    new_pcd_np = np.zeros_like(raw_pcd_np)
    new_pcd_np[:, 0] = raw_pcd_np[:, 0]
    new_pcd_np[:, 1] = -raw_pcd_np[:, 2]
    new_pcd_np[:, 2] = raw_pcd_np[:, 1]

    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(new_pcd_np)

    o3d.io.write_point_cloud(args.map_name + "_plane.pcd", new_pcd)

    # matlab_pcd = o3d.io.read_point_cloud("./plain_map.pcd")

    # o3d.visualization.draw_geometries([new_pcd])  # visualization


    # o3d.visualization.draw_geometries([matlab_pcd, new_pcd])

