import os
import json
import copy

import numpy as np
import open3d as o3d


def make_source_and_target():
    """
    Loads Open3D demo point clouds for ICP.
    Falls back to synthetic data if download is not available.
    """
    try:
        data = o3d.data.DemoICPPointClouds()
        source = o3d.io.read_point_cloud(data.paths[0])
        target = o3d.io.read_point_cloud(data.paths[1])
        print("Loaded Open3D DemoICPPointClouds.")
        return source, target

    except Exception as e:
        print("Demo data not available, using synthetic point clouds:", e)

        mesh = o3d.geometry.TriangleMesh.create_sphere(
            radius=1.0).subdivide_midpoint(2)
        source = mesh.sample_points_poisson_disk(3000)

        T = np.eye(4)
        T[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz(
            (0.3, -0.2, 0.15))
        T[:3, 3] = np.array([0.3, 0.1, 0.2])

        target = copy.deepcopy(source).transform(T)
        return source, target


def preprocess(pcd, voxel=0.05):
    """
    Downsample + estimate normals (required for point-to-plane ICP).
    Bigger voxel => fewer points => easier to understand.
    """
    pcd_down = pcd.voxel_down_sample(voxel)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.0, max_nn=30)
    )
    return pcd_down


def run_icp(source, target, voxel=0.05):
    """
    Runs point-to-plane ICP and returns result + downsampled clouds.
    """
    source_d = preprocess(source, voxel)
    target_d = preprocess(target, voxel)

    threshold = voxel * 2.5
    init = np.eye(4)

    result = o3d.pipelines.registration.registration_icp(
        source_d,
        target_d,
        threshold,
        init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )
    return result, source_d, target_d


def ensure_parent_dir(file_path: str) -> None:
    parent = os.path.dirname(file_path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def save_pose_txt(T, out_path="outputs/pose_T.txt"):
    """Save 4x4 pose matrix to disk."""
    ensure_parent_dir(out_path)
    np.savetxt(out_path, T, fmt="%.6f")
    print(f"Saved pose TXT to: {out_path}")


def save_pose_json(T, out_path="outputs/pose.json"):
    """Save pose to JSON (robot integration friendly)."""
    ensure_parent_dir(out_path)

    R = T[:3, :3]
    t = T[:3, 3]

    data = {
        "translation_m": {"x": float(t[0]), "y": float(t[1]), "z": float(t[2])},
        "rotation_matrix": [[float(v) for v in row] for row in R],
        "T_4x4": [[float(v) for v in row] for row in T],
    }

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    print(f"Saved pose JSON to: {out_path}")


def rotation_matrix_to_rotvec(R):
    """
    Convert 3x3 rotation matrix to UR rotation vector (rx, ry, rz).
    UR pose uses axis-angle vector (axis * angle), in radians.
    """
    trace = float(np.trace(R))
    cos_theta = (trace - 1.0) / 2.0
    cos_theta = max(-1.0, min(1.0, cos_theta))

    theta = float(np.arccos(cos_theta))
    if theta < 1e-12:
        return 0.0, 0.0, 0.0

    sin_theta = float(np.sin(theta))
    if abs(sin_theta) < 1e-12:
        # Rare near-180° case; keep simple for this demo
        return 0.0, 0.0, 0.0

    axis_x = (R[2, 1] - R[1, 2]) / (2.0 * sin_theta)
    axis_y = (R[0, 2] - R[2, 0]) / (2.0 * sin_theta)
    axis_z = (R[1, 0] - R[0, 1]) / (2.0 * sin_theta)

    return float(axis_x * theta), float(axis_y * theta), float(axis_z * theta)


def save_ur_pose(T, out_path="outputs/ur_pose.txt"):
    """
    Save UR pose format: p[x, y, z, rx, ry, rz]
    - x,y,z in meters
    - rx,ry,rz in radians (rotation vector)
    """
    ensure_parent_dir(out_path)

    R = T[:3, :3]
    t = T[:3, 3]
    rx, ry, rz = rotation_matrix_to_rotvec(R)

    pose_str = f"p[{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}, {rx:.6f}, {ry:.6f}, {rz:.6f}]"

    with open(out_path, "w", encoding="utf-8") as f:
        f.write(pose_str + "\n")

    print(f"Saved UR pose to: {out_path}")
    print("UR Pose:", pose_str)


def show_two_clouds(a, b, title):
    """Show ONLY TWO point clouds to make results easy to understand."""
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title)

    vis.add_geometry(a)
    vis.add_geometry(b)

    vis.poll_events()
    vis.update_renderer()
    vis.run()
    vis.destroy_window()


def main():
    # 1) Load data
    source, target = make_source_and_target()

    # 2) ICP
    result, source_d, target_d = run_icp(source, target, voxel=0.05)
    T = result.transformation

    print("\n=== ICP Result ===")
    print("Fitness:", result.fitness)
    print("RMSE   :", result.inlier_rmse)
    print("T (4x4):\n", T)

    # 3) Save outputs (do NOT commit generated outputs)
    save_pose_txt(T, "outputs/pose_T.txt")
    save_pose_json(T, "outputs/pose.json")
    save_ur_pose(T, "outputs/ur_pose.txt")

    # 4) Prepare colored clouds
    src = copy.deepcopy(source_d)
    tgt = copy.deepcopy(target_d)

    src.paint_uniform_color([1, 0, 0])  # red = source (before)
    tgt.paint_uniform_color([0, 1, 0])  # green = target

    src_aligned = copy.deepcopy(src).transform(T)
    src_aligned.paint_uniform_color([0, 0, 1])  # blue = aligned source (after)

    # 5) Clear comparison: BEFORE then AFTER
    show_two_clouds(tgt, src, "BEFORE ICP (green=target, red=source)")
    show_two_clouds(tgt, src_aligned, "AFTER ICP (green=target, blue=aligned)")


if __name__ == "__main__":
    main()
