#!/usr/bin/env python3

from __future__ import annotations
import time
from typing import Optional, List, Dict

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from trajectory_msgs.msg import JointTrajectory
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration as DurationMsg

# NEW: for canceling FollowJointTrajectory action goals
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from unique_identifier_msgs.msg import UUID

try:
    from sklearn.cluster import DBSCAN
except Exception:
    DBSCAN = None


class UnknownPointsStopper(Node):
    def __init__(self) -> None:
        super().__init__("unknown_points_stopper")

        # -------- Parameters --------
        self.declare_parameter("cloud_topic", "/unknown_points")
        self.declare_parameter("controller_name", "joint_trajectory_controller")
        self.declare_parameter("dbscan_eps", 0.05)        # meters
        self.declare_parameter("dbscan_min_samples", 10)
        self.declare_parameter("cluster_min_size", 30)    # points
        self.declare_parameter("stop_cooldown_sec", 1.5)

        self.cloud_topic: str = self.get_parameter("cloud_topic").get_parameter_value().string_value
        self.controller_name: str = self.get_parameter("controller_name").get_parameter_value().string_value
        self.dbscan_eps: float = float(self.get_parameter("dbscan_eps").value)
        self.dbscan_min_samples: int = int(self.get_parameter("dbscan_min_samples").value)
        self.cluster_min_size: int = int(self.get_parameter("cluster_min_size").value)
        self.stop_cooldown_sec: float = float(self.get_parameter("stop_cooldown_sec").value)

        if DBSCAN is None:
            self.get_logger().error(
                "scikit-learn not found. Install with:\n"
                "  sudo apt-get install python3-sklearn   (or)\n"
                "  pip3 install scikit-learn"
            )

        # Publisher to controller topic (fallback stop)
        traj_topic = f"/{self.controller_name}/joint_trajectory"
        self.traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
        self.get_logger().info(f"Stop publisher -> {traj_topic}")

        # Marker publisher (centroids of obstacle clusters)
        self.marker_pub = self.create_publisher(MarkerArray, "obstacle_centroids", 10)

        # Subscriber to unknown points cloud
        self.cloud_sub = self.create_subscription(PointCloud2, self.cloud_topic, self._cloud_cb, 10)
        self.get_logger().info(f"Listening on {self.cloud_topic}")

        # Manual stop service
        self.stop_srv = self.create_service(Trigger, "stop_now", self._stop_now_srv)

        # CancelGoal service client (FollowJointTrajectory)
        self._cancel_cli = None  # lazy-created on first use

        self._last_stop_time: float = 0.0

    # -------- Callbacks --------
    def _cloud_cb(self, msg: PointCloud2) -> None:
        if DBSCAN is None:
            return

        pts = self._pc2_to_numpy(msg)
        if pts is None:
            return

        n_pts = int(pts.shape[0])
        if n_pts == 0:
            self._publish_centroid_markers(msg.header.frame_id, [])
            self.get_logger().info("DBSCAN: points=0, clusters=0, noise_pts=0")
            return

        # Cluster entire cloud
        labels = self._dbscan_labels(pts)
        if labels is None:
            self._publish_centroid_markers(msg.header.frame_id, [])
            return

        # --- Stats (exclude noise = -1)
        noise_pts = int(np.sum(labels == -1))
        unique_labels = [int(l) for l in np.unique(labels) if l != -1]
        num_clusters = len(unique_labels)

        cluster_sizes: Dict[int, int] = {}
        centroids_all: Dict[int, np.ndarray] = {}
        for lbl in unique_labels:
            mask = (labels == lbl)
            sz = int(np.sum(mask))
            cluster_sizes[lbl] = sz
            centroids_all[lbl] = pts[mask].mean(axis=0)

        # Obstacle clusters that exceed threshold
        obstacle_labels = [lbl for lbl, sz in cluster_sizes.items() if sz >= self.cluster_min_size]
        obstacle_sizes = [cluster_sizes[lbl] for lbl in obstacle_labels]
        obstacle_centroids = [centroids_all[lbl] for lbl in obstacle_labels]

        # --- Logging
        self.get_logger().info(
            f"DBSCAN: points={n_pts}, clusters={num_clusters}, noise_pts={noise_pts}, "
            f"eps={self.dbscan_eps}, min_samples={self.dbscan_min_samples}"
        )
        if num_clusters > 0:
            sizes_sorted = sorted(cluster_sizes.items(), key=lambda kv: (-kv[1], kv[0]))
            sizes_str = ", ".join([f"#{lbl}:{sz}" for lbl, sz in sizes_sorted])
            self.get_logger().info(f"Cluster sizes: {sizes_str}")

        if obstacle_labels:
            obst_pairs = list(zip(obstacle_labels, obstacle_sizes))
            obst_str = ", ".join([f"#{lbl}:{sz}" for lbl, sz in sorted(obst_pairs, key=lambda kv: (-kv[1], kv[0]))])
            self.get_logger().warn(f"Obstacles: count={len(obstacle_labels)}, sizes=[{obst_str}]")
            for lbl, c in zip(obstacle_labels, obstacle_centroids):
                self.get_logger().warn(
                    f"  Obstacle #{lbl}: centroid=({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})"
                )
        else:
            self.get_logger().info("Obstacles: count=0")

        # Markers for obstacle centroids
        self._publish_centroid_markers(msg.header.frame_id, obstacle_centroids)

        # Stop (cancel goal) if any obstacle detected (with cooldown)
        if obstacle_labels:
            now = time.time()
            if now - self._last_stop_time >= self.stop_cooldown_sec:
                self._last_stop_time = now
                if not self._cancel_follow_joint_trajectory():
                    self.get_logger().warn(
                        "CancelGoal failed or unavailable → publishing EMPTY JointTrajectory fallback."
                    )
                    self._publish_empty_trajectory()

    def _publish_empty_trajectory(self) -> None:
        self.traj_pub.publish(JointTrajectory())

    def _stop_now_srv(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok = self._cancel_follow_joint_trajectory()
        if not ok:
            self._publish_empty_trajectory()
            res.success = True
            res.message = "CancelGoal failed/unavailable; fallback EMPTY JointTrajectory published."
        else:
            res.success = True
            res.message = "FollowJointTrajectory CancelGoal sent."
        return res

    # -------- Cancel FollowJointTrajectory goal --------
    def _cancel_follow_joint_trajectory(self) -> bool:
        """
        Cancel any active FollowJointTrajectory goals by calling the CancelGoal
        service with an all-zero UUID (cancel-all).
        """
        srv_name = f"/{self.controller_name}/follow_joint_trajectory/_action/cancel_goal"
        if self._cancel_cli is None:
            self._cancel_cli = self.create_client(CancelGoal, srv_name)

        # Don't block forever; just a short wait
        if not self._cancel_cli.service_is_ready():
            ready = self._cancel_cli.wait_for_service(timeout_sec=0.2)
            if not ready:
                self.get_logger().warn(f"CancelGoal service not available: {srv_name}")
                return False

        req = CancelGoal.Request()
        req.goal_info = GoalInfo()
        req.goal_info.goal_id = UUID(uuid=[0] * 16)  # all-zero → cancel-all
        req.goal_info.stamp.sec = 0
        req.goal_info.stamp.nanosec = 0

        future = self._cancel_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

        if not future.done():
            self.get_logger().warn("CancelGoal call timed out.")
            return False

        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(f"CancelGoal call failed: {e}")
            return False

        # Log what the server says back
        return_code = getattr(resp, "return_code", 0)
        goals_canceling = getattr(resp, "goals_canceling", [])
        self.get_logger().warn(
            f"FollowJointTrajectory CancelGoal sent (return_code={return_code}, "
            f"goals_canceling={len(goals_canceling)})."
        )
        return True

    # -------- Marker helpers --------
    def _publish_centroid_markers(self, frame_id: str, centroids: List[np.ndarray]) -> None:
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        lifetime = DurationMsg(sec=0, nanosec=int(0.5 * 1e9))  # 0.5 s

        for i, c in enumerate(centroids):
            m = Marker()
            m.header.frame_id = frame_id or "world"
            m.header.stamp = stamp
            m.ns = "obstacle_centroid"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(c[0])
            m.pose.position.y = float(c[1])
            m.pose.position.z = float(c[2])
            m.pose.orientation.w = 1.0
            m.scale.x = 0.06
            m.scale.y = 0.06
            m.scale.z = 0.06
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.lifetime = lifetime
            ma.markers.append(m)

        self.marker_pub.publish(ma)

    # -------- DBSCAN / PointCloud2 helpers --------
    def _dbscan_labels(self, pts_xyz: np.ndarray) -> Optional[np.ndarray]:
        try:
            model = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples)
            return model.fit_predict(pts_xyz.astype(np.float32))
        except Exception as e:
            self.get_logger().error(f"DBSCAN failed: {e}")
            return None

    @staticmethod
    def _pc2_to_numpy(msg: PointCloud2) -> Optional[np.ndarray]:
        from sensor_msgs_py import point_cloud2 as pc2  # lazy import
        try:
            gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            arr = np.fromiter(gen, dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])
            if arr.size == 0:
                return np.empty((0, 3), dtype=np.float32)
            return np.vstack((arr["x"], arr["y"], arr["z"])).T
        except Exception:
            try:
                pts = np.array(
                    list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
                    dtype=np.float32,
                )
                return pts.reshape(-1, 3) if pts.size else np.empty((0, 3), dtype=np.float32)
            except Exception:
                return None


def main(args=None):
    rclpy.init(args=args)
    node = UnknownPointsStopper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
