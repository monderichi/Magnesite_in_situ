#!/usr/bin/env python3

import os
import sys
import xml.etree.ElementTree as ET
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler, euler_matrix

import tf2_ros
from rcl_interfaces.srv import GetParameters
from urdf_parser_py.urdf import URDF
from rclpy.duration import Duration
from tf_transformations import quaternion_multiply, quaternion_matrix
from urdf_parser_py.urdf import Box, Cylinder, Sphere, Mesh

# optional auto-measure via numpy-stl
try:
    from stl import mesh as stlmesh
except ImportError:
    stlmesh = None


def parse_pose(pose_text):
    values = list(map(float, pose_text.strip().split()))
    pos = values[0:3]
    rpy = values[3:6] if len(values) == 6 else [0.0, 0.0, 0.0]
    return pos, rpy


def rotate_and_translate(world_pos, world_rpy, local_pos):
    rot = euler_matrix(*world_rpy)[:3, :3]
    rotated = rot @ np.array(local_pos)
    translated = np.array(world_pos) + rotated
    return translated.tolist()


def combine_pose(world_pos, world_rpy, local_pos, local_rpy):
    position = rotate_and_translate(world_pos, world_rpy, local_pos)
    total_rpy = [w + l for w, l in zip(world_rpy, local_rpy)]
    quat = quaternion_from_euler(*total_rpy)
    return position, quat


class CombinedProcessor(Node):
    def __init__(self, world_file):
        super().__init__('combined_known_unknown_processor')
        self.get_logger().info("Initialized combined known/unknown object processor")
        self.position_offset = np.array([-0.25, 0.0, -0.715])
        self.known_objects = []
        self.subscription_pc = self.create_subscription(
            PointCloud2, '/final_points', self.pointcloud_callback, 10)

        self.known_pub = self.create_publisher(MarkerArray, "/known_objects_markers", 10)
        self.unknown_pub = self.create_publisher(PointCloud2, "/unknown_points", 10)

        self.parse_and_store_known_objects(world_file)

        # Publish known markers repeatedly every 2 seconds
        self.create_timer(2.0, self.publish_known_markers)

        # TF listener for link poses
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Collect robot collision primitives from URDF
        self.robot_collisions = []   # dicts: {link, shape, size, local_pos, local_rpy}
        self.robot_marker_pub = self.create_publisher(MarkerArray, "/robot_objects_markers", 10)
        self.load_robot_collision_geometry_from_robot_description()

        # Publish robot markers (e.g., 5 Hz)
        self.create_timer(0.2, self.publish_robot_markers)

        self.eps_scale = 1e-3

    def parse_and_store_known_objects(self, world_file):
        try:
            tree = ET.parse(world_file)
        except Exception as e:
            self.get_logger().error(f"Failed to parse world file: {e}")
            return

        root = tree.getroot()
        includes = root.findall(".//include")
        models = []

        for inc in includes:
            uri = inc.find("uri")
            pose_tag = inc.find("pose")
            if uri is None or not uri.text.startswith("file://"):
                continue
            path = uri.text.replace("file://", "")
            model_path = os.path.join(path, "model.sdf") if os.path.isdir(path) else path
            if not os.path.exists(model_path):
                self.get_logger().warn(f"Model not found: {model_path}")
                continue
            pos, rpy = parse_pose(pose_tag.text) if pose_tag is not None else ([0, 0, 0], [0, 0, 0])
            models.append({"model_path": model_path, "pose": pos, "rpy": rpy})

        for model in models:
            try:
                tree = ET.parse(model["model_path"])
                root = tree.getroot()
                model_elem = root.find(".//model")
                model_name = model_elem.get("name", "unknown")
            except Exception as e:
                self.get_logger().warn(f"Could not parse {model['model_path']}: {e}")
                continue

            for col in root.findall(".//collision"):
                name = col.get("name", "part")
                pose_tag = col.find("pose")
                local_pos, local_rpy = parse_pose(pose_tag.text) if pose_tag is not None else ([0, 0, 0], [0, 0, 0])
                global_pos, quat = combine_pose(model["pose"], model["rpy"], local_pos, local_rpy)

                geom = col.find("geometry")
                if geom is None:
                    continue

                if geom.find("box") is not None:
                    size = list(map(float, geom.find("box/size").text.strip().split()))
                    shape = "box"
                elif geom.find("cylinder") is not None:
                    radius = float(geom.find("cylinder/radius").text)
                    height = float(geom.find("cylinder/length").text)
                    size = [radius, height]
                    shape = "cylinder"
                else:
                    continue

                self.known_objects.append({
                    "id": f"{model_name}::{name}",
                    "shape": shape,
                    "size": size,
                    "position": global_pos,
                    "orientation": quat
                })
                self.get_logger().info(f"[Parsed] {model_name}::{name} as {shape} at {global_pos}")

        self.get_logger().info(f"Stored {len(self.known_objects)} known collision objects.")

    def publish_known_markers(self):
        padding = 0.001  # same padding as filtering
        if not self.known_objects:
            self.get_logger().warn("No known objects to publish as markers.")
            return

        marker_array = MarkerArray()
        for i, obj in enumerate(self.known_objects):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "known_objs"
            marker.id = i
            marker.action = Marker.ADD

            # Apply translation offset here
            pos = np.array(obj["position"]) + self.position_offset
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pos.tolist()

            qx, qy, qz, qw = obj["orientation"]
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.6

            if obj["shape"] == "box":
                marker.type = Marker.CUBE
                # Add padding on all sides
                marker.scale.x = obj["size"][0] + padding * 2
                marker.scale.y = obj["size"][1] + padding * 2
                marker.scale.z = obj["size"][2] + padding * 2
            elif obj["shape"] == "cylinder":
                marker.type = Marker.CYLINDER
                # Add padding to radius and height
                marker.scale.x = marker.scale.y = (obj["size"][0] + padding) * 2  # diameter + padding
                marker.scale.z = obj["size"][1] + padding * 2  # height + padding

            marker_array.markers.append(marker)

        self.known_pub.publish(marker_array)

    def quaternion_to_rotation_matrix(self, q):
        qx, qy, qz, qw = q
        # Compute rotation matrix from quaternion
        R = np.array([
            [1 - 2*qy*qy - 2*qz*qz,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw,         1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw,     1 - 2*qx*qx - 2*qy*qy]
        ])
        return R

    def remove_known(self, points):
        padding = 0.001
        mask = np.ones(len(points), dtype=bool)

        for obj in self.known_objects:
            pos = np.array(obj['position']) + self.position_offset
            q = obj['orientation']  # quaternion x,y,z,w

            # Build rotation matrix from quaternion
            R_mat = self.quaternion_to_rotation_matrix(q)

            # Inverse rotation = transpose (orthonormal matrix)
            R_inv = R_mat.T

            if obj['shape'] == "box":
                dx, dy, dz = obj['size']
                dx += padding * 2
                dy += padding * 2
                dz += padding * 2
            elif obj['shape'] == "cylinder":
                r, h = obj['size']
                r += padding
                dx = dy = r * 2
                dz = h + padding * 2
            else:
                continue

            # Translate points relative to the object's center
            pts_local = points - pos

            # Rotate points into the object's local frame
            pts_local_rot = pts_local @ R_inv

            # Check if the point is inside the axis-aligned box (with padding)
            in_box = (
                (pts_local_rot[:, 0] >= -dx / 2) & (pts_local_rot[:, 0] <= dx / 2) &
                (pts_local_rot[:, 1] >= -dy / 2) & (pts_local_rot[:, 1] <= dy / 2) &
                (pts_local_rot[:, 2] >= -dz / 2) & (pts_local_rot[:, 2] <= dz / 2)
            )

            mask &= ~in_box

        return points[mask]

    def pointcloud_callback(self, msg):
        raw_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if len(raw_points) == 0:
            self.get_logger().warn("Empty pointcloud, skipping.")
            return

        try:
            points = np.array([[p[0], p[1], p[2]] for p in raw_points], dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f"Failed to convert pointcloud: {e}")
            return

        if points.ndim != 2 or points.shape[1] != 3:
            self.get_logger().warn("PointCloud format invalid or unexpected shape.")
            return

        self.get_logger().info(f"PointCloud received with {len(points)} points.")

        unknown_points = self.remove_known(points)
        self.get_logger().info(f"Found {len(unknown_points)} unknown points.")

        # Also remove the robot
        unknown_points = self.remove_robot(unknown_points)
        self.get_logger().info(f"Found {len(unknown_points)} unknown points after removing robot.")

        # ---- print XYZ range of unknown points ----
        """if unknown_points.size > 0:
            mins = unknown_points.min(axis=0)
            maxs = unknown_points.max(axis=0)
            span = maxs - mins
            center = (mins + maxs) / 2.0
            m = np.round(mins, 3).tolist()
            M = np.round(maxs, 3).tolist()
            S = np.round(span, 3).tolist()
            C = np.round(center, 3).tolist()
            self.get_logger().info(f"[UNKNOWN RANGE] min={m}  max={M}  span={S}  center={C}")
        else:
            self.get_logger().info("[UNKNOWN RANGE] no points")
        """

        unknown_points = self.remove_ground_by_z(
            unknown_points,
            z_plane=float(self.position_offset[2]),
            tol=0.02  # 2 cm – adjust if needed
        )

        self.get_logger().info(f"Found {len(unknown_points)} unknown points after removing ground.")
        self.publish_unknown_pointcloud(unknown_points, msg.header)
        

    def publish_unknown_pointcloud(self, points, header):
        cloud = pc2.create_cloud_xyz32(header, points.tolist())
        self.unknown_pub.publish(cloud)

    
    def get_robot_description_xml(self):
        client = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("get_parameters not available; cannot fetch /robot_description")
            return None
        req = GetParameters.Request()
        req.names = ["robot_description"]
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if not fut.result() or not fut.result().values:
            self.get_logger().warn("Empty /robot_description")
            return None
        return fut.result().values[0].string_value

    def _aabb_from_mesh(self, filename, scale_xyz):
        if stlmesh is None:
            self.get_logger().warn("numpy-stl not installed; cannot auto-measure mesh AABB.")
            return None, None
        path = filename.replace("file://", "")
        if not path.lower().endswith(".stl"):
            self.get_logger().warn(f"Non-STL mesh '{path}' — skipping AABB (numpy-stl handles only STL).")
            return None, None
        try:
            m = stlmesh.Mesh.from_file(path, calculate_normals=False, speedups=True)
            pts = m.vectors.reshape(-1, 3)
            pts = pts[np.isfinite(pts).all(axis=1)]
            if pts.size == 0:
                self.get_logger().warn(f"Mesh has no valid vertices: {path}")
                return None, None

            min_xyz = pts.min(axis=0).astype(float)
            max_xyz = pts.max(axis=0).astype(float)
            extents = (max_xyz - min_xyz)           # [x,y,z]
            center  = (min_xyz + max_xyz) * 0.5     # mesh-frame center

            s = np.array(scale_xyz if scale_xyz is not None else [1.0, 1.0, 1.0], dtype=float)
            size   = (extents * s).tolist()
            center = (center  * s).tolist()

            eps = 1e-4
            size = [max(eps, float(v)) for v in size]
            return size, center
        except Exception as e:
            self.get_logger().warn(f"Failed AABB for {path}: {e}")
            return None, None


    def load_robot_collision_geometry_from_robot_description(self):
        xml = self.get_robot_description_xml()
        if not xml:
            self.get_logger().warn("Robot URDF not found; robot will NOT be filtered.")
            return
        try:
            urdf = URDF.from_xml_string(xml)
        except Exception as e:
            self.get_logger().error(f"Failed to parse URDF: {e}")
            return

        count = 0
        for link in urdf.links:
            if not link.collisions:
                continue
            for coll in link.collisions:
                xyz = coll.origin.xyz if coll.origin else [0.0, 0.0, 0.0]
                rpy = coll.origin.rpy if coll.origin else [0.0, 0.0, 0.0]
                g = coll.geometry

                if isinstance(g, Box):
                    size = list(g.size)  # [x, y, z]
                    self.robot_collisions.append({
                        "link": link.name, "shape": "box", "size": size,
                        "local_pos": xyz, "local_rpy": rpy
                    })
                    count += 1
                    self.get_logger().info(f"[URDF] {link.name} {('box','cylinder','sphere')[0 if isinstance(g, Box) else (1 if isinstance(g, Cylinder) else 2)]} size={self.robot_collisions[-1]['size']}")

                elif isinstance(g, Cylinder):
                    r = float(g.radius); h = float(g.length)
                    self.robot_collisions.append({
                        "link": link.name, "shape": "cylinder", "size": [r, h],
                        "local_pos": xyz, "local_rpy": rpy
                    })
                    count += 1
                    self.get_logger().info(f"[URDF] {link.name} {('box','cylinder','sphere')[0 if isinstance(g, Box) else (1 if isinstance(g, Cylinder) else 2)]} size={self.robot_collisions[-1]['size']}")

                elif isinstance(g, Sphere):
                    r = float(g.radius)
                    self.robot_collisions.append({
                        "link": link.name, "shape": "sphere", "size": [r],
                        "local_pos": xyz, "local_rpy": rpy
                    })
                    count += 1
                    self.get_logger().info(f"[URDF] {link.name} {('box','cylinder','sphere')[0 if isinstance(g, Box) else (1 if isinstance(g, Cylinder) else 2)]} size={self.robot_collisions[-1]['size']}")

                elif isinstance(g, Mesh):
                    scale = list(g.scale) if g.scale is not None else [1.0, 1.0, 1.0]
                    self.get_logger().info(f"[URDF-MESH] {link.name}: file='{g.filename}', scale={scale}")

                    size, center = self._aabb_from_mesh(g.filename, scale)
                    if size:
                        # Shift the AABB center into the link frame
                        R_local = euler_matrix(*(rpy if rpy else [0.0, 0.0, 0.0]))[:3, :3]
                        xyz_np  = np.array(xyz, dtype=float)
                        cen_np  = np.array(center, dtype=float)
                        local_pos_adj = (xyz_np + R_local @ cen_np).tolist()

                        self.robot_collisions.append({
                            "link": link.name, "shape": "box", "size": size,
                            "local_pos": local_pos_adj, "local_rpy": rpy
                        })
                        count += 1
                        self.get_logger().info(f"[URDF-MESH-AABB] {link.name} -> box size={size}, center={center}")
                    else:
                        self.get_logger().warn(f"Mesh collision on '{link.name}' ignored (no AABB).")

        self.get_logger().info(f"Loaded {count} robot collision primitives from URDF.")

    def get_link_world_pose(self, link_name):
        try:
            t = self.tf_buffer.lookup_transform(
                "world", link_name, rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed for {link_name}: {e}")
            return None, None
        trans = np.array([t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z], dtype=np.float32)
        quat = np.array([t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w], dtype=np.float32)
        return trans, quat

    def compute_robot_objects_world(self):
        objs = []
        for rc in self.robot_collisions:
            trans, quat = self.get_link_world_pose(rc["link"])
            if trans is None:
                continue
            R_world = quaternion_matrix(quat)[:3, :3]
            local_pos = np.array(rc["local_pos"], dtype=np.float32)

            local_quat = np.array(quaternion_from_euler(*rc["local_rpy"]), dtype=np.float32)
            world_quat = np.array(quaternion_multiply(quat, local_quat), dtype=np.float32)

            world_pos = trans + R_world @ local_pos
            # Do NOT add offset to the robot
            # world_pos = world_pos + self.position_offset

            objs.append({
                "id": f"robot::{rc['link']}",
                "shape": rc["shape"],
                "size": rc["size"],
                "position": world_pos.tolist(),
                "orientation": world_quat.tolist()
            })
        return objs

    def publish_robot_markers(self):
        robot_objs = self.compute_robot_objects_world()
        if not robot_objs:
            return
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, obj in enumerate(robot_objs):
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = now
            m.ns = "robot_objs"
            m.id = i
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = obj["position"]
            qx, qy, qz, qw = obj["orientation"]
            m.pose.orientation.x = qx; m.pose.orientation.y = qy
            m.pose.orientation.z = qz; m.pose.orientation.w = qw
            # blue for the robot
            m.color.r = 0.0; m.color.g = 0.2; m.color.b = 1.0; m.color.a = 0.6
            if obj["shape"] == "box":
                m.type = Marker.CUBE
                sx, sy, sz = obj["size"]
                m.scale.x = max(self.eps_scale, sx)
                m.scale.y = max(self.eps_scale, sy)
                m.scale.z = max(self.eps_scale, sz)

            elif obj["shape"] == "cylinder":
                m.type = Marker.CYLINDER
                r, h = obj["size"]
                m.scale.x = m.scale.y = max(self.eps_scale, 2.0 * r)
                m.scale.z = max(self.eps_scale, h)

            elif obj["shape"] == "sphere":
                m.type = Marker.SPHERE
                r = obj["size"][0]
                d = max(self.eps_scale, 2.0 * r)
                m.scale.x = m.scale.y = m.scale.z = d
            else:
                continue
            ma.markers.append(m)
        self.robot_marker_pub.publish(ma)


    def remove_robot(self, points):
        padding = 0.08
        eps = 5e-4 #1e-4
        inflate_box   = 1.80 #1.18
        inflate_cyl_r = 2.00 #1.30
        inflate_cyl_h = 1.80 #1.12
        inflate_sphere= 1.80 #1.18
        mask = np.ones(len(points), dtype=bool)
        robot_objs = self.compute_robot_objects_world()
        for obj in robot_objs:
            pos = np.array(obj['position'], dtype=np.float32)
            q = obj['orientation']
            R_mat = self.quaternion_to_rotation_matrix(q)
            R_inv = R_mat.T
            pts_local = points - pos
            pts_local_rot = pts_local @ R_inv

            if obj['shape'] == "box":
                #dx, dy, dz = obj['size']
                dx, dy, dz = np.array(obj['size'], dtype=np.float64) * inflate_box
                dx += padding * 2; dy += padding * 2; dz += padding * 2
                in_shape = (
                    (np.abs(pts_local_rot[:, 0]) <= dx / 2 + eps) &
                    (np.abs(pts_local_rot[:, 1]) <= dy / 2 + eps) &
                    (np.abs(pts_local_rot[:, 2]) <= dz / 2 + eps)
                )
            elif obj['shape'] == "cylinder":
                r, h = obj['size']
                #r = r + padding
                r = r * inflate_cyl_r + padding
                #hz = h / 2.0 + padding
                hz = (h * inflate_cyl_h)/2.0 + padding
                rho2 = pts_local_rot[:, 0]**2 + pts_local_rot[:, 1]**2
                in_shape = (rho2 <= (r+eps)**2) & (np.abs(pts_local_rot[:, 2]) <= hz + eps)
            elif obj['shape'] == "sphere":
                #r = obj['size'][0] + padding
                r = obj['size'][0] * inflate_sphere + padding
                rr = (pts_local_rot[:, 0]**2 + pts_local_rot[:, 1]**2 + pts_local_rot[:, 2]**2)
                in_shape = rr <= (r+eps)**2
            else:
                continue

            mask &= ~in_shape

        return points[mask]
    
    def remove_ground_by_z(self, points, z_plane=None, tol=0.02):
        """
        Removes a floor plane around a given z-plane with tolerance tol (in meters).
        - z_plane: the floor's z. If None, uses self.position_offset[2].
        - tol: range ±tol around z_plane to be removed.
        """
        if points.size == 0:
            return points
        if z_plane is None:
            z_plane = float(self.position_offset[2])
        mask = np.abs(points[:, 2] - z_plane) > tol
        # optional logging about how many were removed
        removed = int((~mask).sum())
        if removed:
            self.get_logger().info(f"[GROUND FILTER] removed {removed} points at z≈{z_plane}±{tol}")
        return points[mask]


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: ros2 run spraying_pathways unknown_object_detector.py <path_to_world_file>")
        return
    world_file = sys.argv[1]
    print(f"Starting combined object processor with world file: {world_file}")
    node = CombinedProcessor(world_file)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
