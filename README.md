# pointcloud-icp-pose-demo

3D robot-guidance demo: point cloud registration (ICP) → 6D pose output (Open3D).

- **Input:** two point clouds (Open3D demo dataset)
- **Method:** point-to-plane ICP registration
- **Output:** 4x4 pose matrix (R|t) saved to `outputs/pose_T.txt`

## Run (Windows)

```bash
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python src/main_icp_demo.py

## Output files (generated)
When you run the script, it generates:

- `outputs/pose_T.txt`  (4x4 transform matrix)
- `outputs/pose.json`   (translation + rotation matrix + full 4x4)
- `outputs/ur_pose.txt` (UR pose: p[x,y,z,rx,ry,rz])

These files are generated each run and are ignored by Git (`.gitignore`).

## Outputs (generated locally)
When you run the script, it creates:
- `outputs/pose_T.txt` (4x4 transform matrix)
- `outputs/pose.json` (translation + rotation matrix + full 4x4)
- `outputs/ur_pose.txt` (UR format: `p[x,y,z,rx,ry,rz]`)

Note: These files are generated each run and are not committed to Git (see `.gitignore`).
## Robot Guidance Output (Pose Export)

This demo exports the estimated pose in two formats:

- `outputs/pose_T.txt`  
  4×4 homogeneous transform matrix **T** (rotation + translation)

- `outputs/pose.json`  
  Integration-friendly pose export:
  - `translation_m`: x/y/z in meters  
  - `rotation_matrix`: 3×3 rotation matrix  
  - `T_4x4`: full 4×4 matrix

### Example usage (pseudo)
Use `translation_m` and `rotation_matrix` to convert the pose into your robot controller
frame (Base/Tool/TCP) depending on your cell setup and calibration.
