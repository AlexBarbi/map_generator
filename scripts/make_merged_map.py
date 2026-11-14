#!/usr/bin/env python3
"""Stack the SC-PGO keyframe scans into one map, show it and save it as a PCD with intensity.

    make_merged_map.py [save_directory] [--output optimized_map.pcd] [--no-vis]

save_directory is SC-PGO's: Scans/*.pcd (keyframe scans, lidar frame) and optimized_poses.txt (one KITTI-format pose
per keyframe, falling back to odom_poses.txt). Every keyframe is used. By default the points of things that moved
(people, carts) are left out: a point is dropped when other keyframes saw through where it was (dynamic_removal.py).
Needs numpy, scipy and open3d.

Based on SC-PGO's utils/python/makeMergedMap.py.
"""
import argparse
import os
import sys

import numpy as np
import open3d as o3d  # intensity is read/written through the o3d.t (tensor) API

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)
import dynamic_removal  # noqa: E402

color_table = np.load(os.path.join(script_dir, "jet_table.npy"))
intensity_color_max = 200

parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
parser.add_argument("data_dir", nargs="?", default="save_data",
                    help="SC-PGO save_directory, holding Scans/ and *_poses.txt (default: ./save_data)")
parser.add_argument("--near", type=float, default=0.7,
                    help="drop points closer than this to the lidar, in m; 0 keeps them all (default: 0.7). "
                         "The original 2 m was meant for the car of the SC-PGO datasets and drops most of an indoor scan.")
parser.add_argument("--color", choices=["auto", "intensity", "height"], default="auto",
                    help="viewer coloring; auto colors by height when every intensity is 0, as with the Gazebo lidars (default: auto)")
parser.add_argument("--no-floor", action="store_true",
                    help="hide the floor in the viewer: the points up to --floor-margin above the estimated floor height. The saved map keeps them.")
parser.add_argument("--floor-margin", type=float, default=0.15,
                    help="height above the estimated floor that --no-floor hides, in m (default: 0.15)")
parser.add_argument("--point-size", type=float, default=1.0,
                    help="viewer point size in pixels, also used by the screenshots saved with P (default: 1, the smallest; Open3D's default is 5)")
parser.add_argument("--no-vis", action="store_true",
                    help="save the map without opening the viewer, which blocks until its window is closed (unattended runs)")
parser.add_argument("--output", default="optimized_map.pcd",
                    help="map file to save; a relative path is inside data_dir (default: optimized_map.pcd)")
parser.add_argument("--keep-dynamic", action="store_true",
                    help="keep the points of things that moved (people, carts). By default they are removed: a point is "
                         "dropped when other keyframes saw through where it was (dynamic_removal.py)")
parser.add_argument("--dynamic-output", default=None,
                    help="also save the removed points to this PCD (relative: inside data_dir), to check what went")
parser.add_argument("--max-range", type=float, default=0.0,
                    help="drop points farther than this from their keyframe, in m, after the dynamic removal (which uses "
                         "them all); 0 keeps them all (default: 0)")
args = parser.parse_args()

data_dir = os.path.expanduser(args.data_dir)
scan_dir = os.path.join(data_dir, "Scans")
scan_files = sorted(os.listdir(scan_dir))

pose_file = os.path.join(data_dir, "optimized_poses.txt")
if not os.path.exists(pose_file):  # runs recorded before SC-PGO wrote optimized_poses.txt
    pose_file = os.path.join(data_dir, "odom_poses.txt")
    print("optimized_poses.txt not found, falling back to (unoptimized)", pose_file)
poses = [np.vstack((np.reshape([float(v) for v in line.split()], (3, 4)), [0, 0, 0, 1]))
         for line in open(pose_file) if line.strip()]

# SC-PGO rewrites the pose files at 1 Hz, so the newest keyframes may not have a pose yet
num_nodes = min(len(scan_files), len(poses))
if len(poses) < len(scan_files):
    print("WARNING:", len(scan_files), "scans but only", len(poses), "poses in", pose_file,
          "- stacking the first", num_nodes, "scans only")
if num_nodes == 0:
    sys.exit("no keyframe to merge in " + data_dir)

# The keyframes are read first and stacked after the dynamic removal, which looks at each map point from every keyframe
scans_local, scans_intensity = [], []
for node_idx in range(num_nodes):
    print("read keyframe scan idx", node_idx)
    scan = o3d.t.io.read_point_cloud(os.path.join(scan_dir, scan_files[node_idx])).point
    scan_xyz_local = scan.positions.numpy().astype(np.float64)
    scan_intensity = scan.intensity.numpy()[:, :1]
    if args.near > 0:  # platform-myself structure ghost points
        near = np.linalg.norm(scan_xyz_local, axis=1) > args.near
        scan_xyz_local, scan_intensity = scan_xyz_local[near], scan_intensity[near]
    scans_local.append(scan_xyz_local)
    scans_intensity.append(scan_intensity)
scan_poses = poses[:num_nodes]

if args.keep_dynamic:
    dynamic_masks = [np.zeros(len(s), dtype=bool) for s in scans_local]
else:
    dynamic_masks = dynamic_removal.find_dynamic(scans_local, scan_poses)

kept_xyz, kept_intensity, removed_xyz, removed_intensity = [], [], [], []
for scan_xyz_local, scan_intensity, scan_pose, dynamic in zip(scans_local, scans_intensity, scan_poses, dynamic_masks):
    scan_xyz = scan_xyz_local @ scan_pose[:3, :3].T + scan_pose[:3, 3]  # global coord
    keep = ~dynamic
    if args.max_range > 0:
        keep &= np.linalg.norm(scan_xyz_local, axis=1) < args.max_range
    kept_xyz.append(scan_xyz[keep])
    kept_intensity.append(scan_intensity[keep])
    removed_xyz.append(scan_xyz[dynamic])
    removed_intensity.append(scan_intensity[dynamic])
np_xyz_all = np.vstack(kept_xyz)
np_intensity_all = np.vstack(kept_intensity)
print("stacked", len(np_xyz_all), "points from", num_nodes, "keyframes")

if not args.no_vis:
    # The viewer colors and floor cut need the whole map (height range, floor height)
    vis_mask = np.ones(len(np_xyz_all), dtype=bool)
    if args.no_floor:
        floor_z = dynamic_removal.estimate_floor_z(np_xyz_all[:, 2])
        vis_mask = np_xyz_all[:, 2] > floor_z + args.floor_margin
        print("floor at z = %.2f m, hiding the points below z = %.2f m" % (floor_z, floor_z + args.floor_margin))

    if args.color == "height" or (args.color == "auto" and not np.any(np_intensity_all)):
        z_color_min, z_color_max = np.percentile(np_xyz_all[vis_mask, 2], [1, 99])
        print("coloring by height, z from %.2f to %.2f m" % (z_color_min, z_color_max))
        color_ratio = (np_xyz_all[:, 2] - z_color_min) / max(z_color_max - z_color_min, 1e-6)
    else:
        color_ratio = np_intensity_all[:, 0] / intensity_color_max
    colors_idx = np.round((len(color_table) - 1) * np.clip(color_ratio, 0, 1)).astype(int)

    print("draw the merged map.")
    pcd_for_vis = o3d.geometry.PointCloud()
    pcd_for_vis.points = o3d.utility.Vector3dVector(np_xyz_all[vis_mask])
    pcd_for_vis.colors = o3d.utility.Vector3dVector(color_table[colors_idx][vis_mask])
    # draw_geometries has no point size option, so build the window by hand
    map_vis = o3d.visualization.Visualizer()
    map_vis.create_window("Merged map")
    map_vis.add_geometry(pcd_for_vis)
    map_vis.get_render_option().point_size = args.point_size
    map_vis.run()
    map_vis.destroy_window()


def write_xyzi(path, xyz, intensity):
    cloud = o3d.t.geometry.PointCloud()
    cloud.point.positions = o3d.core.Tensor(xyz.astype(np.float32))
    cloud.point.intensity = o3d.core.Tensor(intensity.astype(np.float32))
    o3d.t.io.write_point_cloud(path, cloud, compressed=True)  # binary_compressed


map_name = os.path.join(data_dir, os.path.expanduser(args.output))  # an absolute output replaces data_dir
write_xyzi(map_name, np_xyz_all, np_intensity_all)
print("intensity map is save (path:", map_name, ")")

if args.dynamic_output is not None:
    removed_name = os.path.join(data_dir, os.path.expanduser(args.dynamic_output))
    write_xyzi(removed_name, np.vstack(removed_xyz), np.vstack(removed_intensity))
    print("removed (dynamic) points saved (path:", removed_name, ")")
