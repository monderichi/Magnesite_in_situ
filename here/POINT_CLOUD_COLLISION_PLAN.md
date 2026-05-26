# Point Cloud Collision Avoidance Plan

## Current Situation
- ✅ RealSense D456 camera works and publishes point cloud
- ✅ Robot moves without colliding (currently working)
- ⚠️ Risk: Point cloud artifacts (noise, robot self-seen points, table surface) could cause collision issues
- ❌ Self-filter node is disabled in CMakeLists.txt (missing dependencies)

## The Problem
When MoveIt uses point clouds for collision detection, these artifacts become obstacles:
1. **Robot Self-Seen Points** - Camera sees the robot arm itself
2. **Table/Ground Plane** - Camera sees the work surface
3. **Noise/Outliers** - Sensor noise creates false obstacles
4. **Edge Artifacts** - Depth discontinuities at object edges

## Solution Architecture

```
Raw Point Cloud (/camera/depth/color/points)
    ↓
[1] Point Cloud Preprocessor (Python node)
    ├── Remove ground plane (RANSAC)
    ├── Remove robot self-seen points (TF-based filtering)
    ├── Remove outliers (statistical filtering)
    └── Crop to workspace bounds
    ↓
Filtered Point Cloud (/camera/depth/color/points_filtered)
    ↓
[2] MoveIt Planning Scene (if octomap enabled)
    └── Uses filtered cloud for collision avoidance
    ↓
Safe Robot Motion
```

## Implementation Plan

### Phase 1: Python Point Cloud Filter (Immediate)
Create `pointcloud_collision_filter.py`:
- **Input**: `/camera/depth/color/points`
- **Output**: `/camera/depth/color/points_filtered`
- **Functions**:
  1. **Ground Removal**: Remove points below Z threshold (table surface)
  2. **Robot Self-Filter**: Remove points near robot links using TF
  3. **Workspace Crop**: Only keep points within work volume
  4. **Outlier Removal**: Statistical outlier filter

### Phase 2: Launch File Integration
- Add filter node to `bringup_mycobot_320_realsense.launch.py`
- Update RViz to show filtered cloud
- Configure MoveIt to use filtered topic

### Phase 3: Advanced Features (Future)
- C++ self-filter with MoveIt ShapeMask (fix dependencies)
- Dynamic obstacle tracking
- Octomap integration for 3D occupancy grid

## Configuration Parameters

### Camera Position (Top-Down)
```yaml
camera_x: 0.5      # 50cm forward from base
camera_y: 0.0      # Centered
camera_z: 0.5      # 50cm height
camera_roll: 0.0   # No roll
camera_pitch: 1.5708  # 90° looking DOWN
camera_yaw: 0.0    # No yaw
```

### Filter Parameters
```yaml
# Ground removal
ground_z_max: 0.05  # Remove points below 5cm (table)

# Robot self-filter
robot_filter_padding: 0.05  # 5cm padding around robot links

# Workspace bounds (relative to base_link)
workspace_min_x: 0.0
workspace_max_x: 1.0
workspace_min_y: -0.5
workspace_max_y: 0.5
workspace_min_z: 0.0
workspace_max_z: 1.0

# Outlier removal
outlier_mean_k: 50
outlier_std_dev: 1.0
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/depth/color/points` | PointCloud2 | Raw from RealSense |
| `/camera/depth/color/points_filtered` | PointCloud2 | Filtered for MoveIt |
| `/self_filter/collision_markers` | MarkerArray | Debug: robot collision shapes |

## Files to Modify

1. `src/spraying_pathways/scripts/pointcloud_collision_filter.py` (NEW)
2. `src/spraying_pathways/launch/bringup_mycobot_320_realsense.launch.py`
3. `src/spraying_pathways/rviz/mycobot_320_realsense_minimal.rviz`
4. `src/spraying_pathways/setup.py` (add entry point)

## Testing Checklist

- [ ] Filter node starts without errors
- [ ] TF frames are correctly resolved
- [ ] Ground plane points are removed
- [ ] Robot arm points are filtered
- [ ] Filtered cloud shows in RViz
- [ ] Robot can plan through filtered space
- [ ] Real obstacles are still detected
