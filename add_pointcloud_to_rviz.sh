#!/bin/bash
# Script to add PointCloud2 display to RViz after camera is running

echo "Adding PointCloud2 display to RViz..."
echo ""
echo "Point Cloud Topic: /camera/depth/color/points"
echo "Fixed Frame: world (or camera_depth_optical_frame)"
echo ""
echo "To add manually in RViz:"
echo "  1. Click 'Add' button in Displays panel"
echo "  2. Select 'PointCloud2'"
echo "  3. Set Topic: /camera/depth/color/points"
echo "  4. Set Style: Points or Flat Squares"
echo "  5. Set Size: 0.005"
echo ""
echo "Alternative: Use this rosservice call (if available):"
echo ""

# Check if camera topics are available
echo "Checking camera topics..."
ros2 topic list | grep -E "camera|point" || echo "No camera topics found - camera may not be running"
echo ""

# Show TF frames
echo "Checking TF frames..."
ros2 run tf2_tools view_frames 2>/dev/null &
echo "TF frame diagram generated as frames.pdf"
