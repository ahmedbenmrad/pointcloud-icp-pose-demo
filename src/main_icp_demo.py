import os
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

        mesh = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
        mesh = mesh.subdivide_midpoint(2)
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


def save_pose(T, out_path="outputs/pose_T.txt"):
    """
    Save 4x4 pose matrix to disk.
    """
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    np.savetxt(out_path, T, fmt="%.6f")
    print(f"Saved pose to: {out_path}")


def show_two_clouds(a, b, title):
    """
    Shows ONLY TWO point clouds to make results easy to understand.
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title)

    vis.add_geometry(a)
    vis.add_geometry(b)

    # Optional: set a better initial view
    vis.poll_events()
    vis.update_renderer()

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    # 1) Load data
    source, target = make_source_and_target()

    # 2) ICP
    result, source_d, target_d = run_icp(source, target, voxel=0.05)
    T = result.transformation

    print("\n=== ICP Result ===")
    print("Fitness:", result.fitness)
    print("RMSE   :", result.inlier_rmse)
    print("T (4x4):\n", T)

    # 3) Save pose
    save_pose(T, "outputs/pose_T.txt")

    # 4) Prepare colored clouds
    src = copy.deepcopy(source_d)
    tgt = copy.deepcopy(target_d)

    src.paint_uniform_color([1, 0, 0])  # red = source (before)
    tgt.paint_uniform_color([0, 1, 0])  # green = target

    src_aligned = copy.deepcopy(src).transform(T)
    src_aligned.paint_uniform_color([0, 0, 1])  # blue = aligned source (after)

    # 5) SHOW BEFORE then AFTER (very clear)
    show_two_clouds(tgt, src, "BEFORE ICP (green=target, red=source)")
    show_two_clouds(tgt, src_aligned, "AFTER ICP (green=target, blue=aligned)")
