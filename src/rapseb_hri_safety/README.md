```markdown

Purpose
- Platform to plan and execute spray-painting tasks with a 6-DOF robotic arm. Includes ROS 2 packages for safety, path generation and UR simulation, calibration tooling and example trajectory output for validation.

Status & composition
- Active development for Arise Project (RAPSEB Subgrant).
- Languages: Python, C++, CMake. ROS 2 workspace present at `ros2_ws/`.


- ROS 2 packages under ros2_ws/src/:
  - rapseb_hri_safety
    - Node/script: `ros2_ws/src/rapseb_hri_safety/hri_safety_guard.py`
    - Launch: `ros2_ws/src/rapseb_hri_safety/launch/hri_safety_guard.launch.py`
  - spraying_pathways
  - ur_simulation_gazebo
- Example/recorded trajectory: `ros2_ws/trajectory_output.csv`
- Docker environment files present under a Dockerfile directory at the repo root.

Prerequisites
- Ubuntu 20.04 / 22.04 (match to your ROS 2 distro)
- ROS 2 (Foxy / Galactic / Humble — use the distro you have installed)
- Python 3.8+
- colcon and colcon-common-extensions
- build-essential, CMake >= 3.10, Eigen, Boost and vendor SDKs (EtherCAT/CAN/serial) where required
- Docker 

Repository layout 
- ros2_ws/
  - src/
    - rapseb_hri_safety/
      - hri_safety_guard.py
      - launch/hri_safety_guard.launch.py
    - spraying_pathways/
    - ur_simulation_gazebo/
  - trajectory_output.csv
- Dockerfile/ (Docker environment)

ROS 2 — build and run 

1. Source system ROS 2 environment
- Replace `<ros2-distro>` with your installed distro ( humble is our default )
```bash
source /opt/ros/<ros2-distro>/setup.bash
```

2. Build the workspace
```bash
cd ros2_ws
# build with symlink install for iterative development
colcon build --symlink-install
```

3. Source the workspace
```bash
# after build, in every shell where you run nodes
source install/setup.bash
```

4. Confirm packages
```bash
colcon list
# expected to list at minimum:
# rapseb_hri_safety       ros2_ws/src/rapseb_hri_safety
# spraying_pathways       ros2_ws/src/spraying_pathways
# ur_simulation_gazebo    ros2_ws/src/ur_simulation_gazebo
```

5. Run the provided safety launch (exact, discovered launch file)
```bash
# runs the HRI safety guard launch discovered in the repo
ros2 launch rapseb_hri_safety hri_safety_guard.launch.py
```

6. Discover other executables and launch files (use these commands to get exact names)
```bash
# list executables declared for a package (replace <package>)
ros2 pkg executables <package>

# list launch files in a package's launch directory
ls -la ros2_ws/src/<package>/launch
```

Example workflow (build + run safety guard)
```bash
# example session
source /opt/ros/<ros2-distro>/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch rapseb_hri_safety hri_safety_guard.launch.py
```

Notes on running nodes directly
- If a package exposes executables via its setup/ament configuration, use:
```bash
ros2 run <package> <executable> [--ros-args --params-file <params.yaml>]
```
- If a script is not installed as an entry point, you can run it directly for quick inspection, but ensure you source ROS 2 and set PYTHONPATH appropriately:
```bash
# for debugging only (prefer ros2 run / ros2 launch for deployment)
source /opt/ros/<ros2-distro>/setup.bash
python3 ros2_ws/src/rapseb_hri_safety/hri_safety_guard.py
```
(Running the script directly may require additional environment variables or package setup; prefer the installed entry point.)

Non-ROS (native C++/CMake) build (if present)
- Generic out-of-source build pattern (root CMakeLists.txt or other CMake targets):
```bash
# top-level repository, if a standard CMake build is present
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -- -j$(nproc)

# locate produced executables
find . -type f -perm -111 -maxdepth 6 -print
```
- Install:
```bash
cmake --install . --prefix ../install
../install/bin/<executable> --config ../configs/<config>.yaml
```
Docker 
- Build (Dockerfile is in the repo under `Dockerfile/`):
```bash
docker build -t robotic-arm-spraying:latest -f Dockerfile/Dockerfile .
```
- Run container (example with device passthrough and mapped workspaces):
```bash
docker run --rm -it --network host \
  --device /dev/ttyUSB0 \
  -v $(pwd)/ros2_ws:/workspace/ros2_ws \
  -v $(pwd)/configs:/workspace/configs \
  robotic-arm-spraying:latest /bin/bash


# inside container:
source /opt/ros/<ros2-distro>/setup.bash
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch rapseb_hri_safety hri_safety_guard.launch.py
```

Configuration, params and calibration
- Use ROS 2 parameter YAML files and pass them via `--ros-args --params-file <file>.yaml` when running nodes or in launch files.
- Place robot kinematics, joint limits and spray parameters in `configs/` (create this directory if not present).
- Use provided calibration or path-generation scripts in the workspace packages (look under `ros2_ws/src/*/tools` or `ros2_ws/src/*/scripts`).

Files of interest
- `ros2_ws/trajectory_output.csv` — recorded trajectory data useful for verification and replay.
- `ros2_ws/src/rapseb_hri_safety/hri_safety_guard.py` — HRI safety guard node.
- `ros2_ws/src/rapseb_hri_safety/launch/hri_safety_guard.launch.py` — Launch file to start the safety guard.

Testing & validation
- ROS 2 integration: create launch that starts controllers + mock hardware; then run end-to-end test.
- Python tests: `pytest` from the repo root if tests exist.
- C++ tests: run `ctest` from the native build directory.

Operational safety
- Always keep hardware E-stop enabled during commissioning.
- Validate collision maps and enforce joint/Cartesian velocity & acceleration limits.
- Use PPE and ventilation when operating spray equipment.

Contributing
- Fork -> branch feature/<short-desc> -> PR with tests and changelog entry. Include simulation logs or recorded trajectories where behavior changes are non-trivial.

Maintainer / contact
-Parity-Platform (info@parityplatform.com)

