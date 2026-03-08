# pointcloud-icp-pose-demo

3D robot-guidance demo: point cloud registration (ICP) → 6D pose output (Open3D).

- **Input:** two point clouds (Open3D demo dataset)
- **Method:** point-to-plane ICP registration
- **Output:** 4x4 pose matrix (R|t) saved to `outputs/pose_T.txt`

## Run (Windows)
## Run (Windows)

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python src/main_icp_demo.py
```

## Outputs (generated locally)

- `outputs/pose_T.txt` — 4x4 transform matrix (R|t)
- `outputs/pose.json` — translation + rotation matrix + full 4x4
- `outputs/ur_pose.txt` — UR pose `p[x,y,z,rx,ry,rz]`

> Note: These files are generated each run and are ignored by Git (see `.gitignore`).

## Screenshots

![Before ICP](outputs/screenshots/01_before.png)  
![After ICP](outputs/screenshots/02_after.png)  
![Console](outputs/screenshots/03_console.png)
