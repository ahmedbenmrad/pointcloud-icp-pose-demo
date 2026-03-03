# pointcloud-icp-pose-demo
3D robot-guidance demo: point cloud registration (ICP) + 6D pose output (Open3D).

- **Input:** two point clouds (Open3D demo dataset)
- **Method:** point-to-plane ICP registration
- **Output:** 4×4 pose matrix (R|t) saved to `outputs/pose_T.txt`

## Run (Windows)
```bash
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python src/main_icp_demo.py
