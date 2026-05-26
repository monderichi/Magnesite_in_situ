#!/usr/bin/env python3
"""
conveyor_rendezvous_solver.py

Analytical rendezvous solver for a robot manipulator intercepting objects on a
linear conveyor belt. Computes the optimal Y intercept point where robot and
object arrive simultaneously, accounting for planning overhead, robot Cartesian
speed, belt speed, and push duration.

Designed for ROS2 + MoveIt2 conveyor operations. Import as a module or run
standalone for simulation/testing.

Usage:
    python3 conveyor_rendezvous_solver.py --demo
"""

import argparse
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class RendezvousResult:
    """Result of a rendezvous computation."""
    y_r: float          # Rendezvous Y position in base_link (m)
    t_r: float          # Time from now until rendezvous (s)
    feasible: bool      # True if the rendezvous is physically reachable
    reason: str = ""    # Human-readable status / failure reason


class ConveyorRendezvousSolver:
    """
    Solves the 1-D rendezvous problem along the conveyor belt Y axis.

    The belt moves objects toward the robot along -Y (decreasing Y).
    The robot starts from a fixed park pose at Y_PARK and moves along Y
    to meet the object. The push action is along X with constant Z.
    """

    def __init__(
        self,
        y_park: float,
        v_robot: float,
        t_overhead: float,
        y_workspace_min: float,
        y_workspace_max: float,
        t_push: float = 2.0,
        tool_width_y: float = 0.08,
        nominal_motion: float = 0.70,
        robot_vel_scale: float = 0.10,
    ):
        """
        Args:
            y_park: Robot park Y position in base_link (m). This is the
                pose the robot returns to after each cycle.
            v_robot: Robot Cartesian Y speed magnitude (m/s). Measure this
                empirically at your operating velocity_scale, or estimate from
                joint limits: v_cartesian ≈ v_joint_max × scale × lever_arm.
            t_overhead: Planning + network + perception latency overhead (s).
                This is the speed-independent part of execution time.
            y_workspace_min: Minimum reachable Y in base_link (m).
            y_workspace_max: Maximum reachable Y in base_link (m).
            t_push: Duration from robot arrival at rendezvous Y until push
                action is fully complete (s). Includes orient + lower + push.
            tool_width_y: Effective tool/contact width in Y direction (m).
                The object must not drift more than this during the push.
            nominal_motion: Nominal motion time at 100% speed (s)
            robot_vel_scale: MoveIt velocity scaling factor
        """
        self.y_park = float(y_park)
        self.v_robot = float(v_robot)
        self.t_overhead = float(t_overhead)
        self.planning_overhead = float(t_overhead)
        self.y_min = float(y_workspace_min)
        self.y_max = float(y_workspace_max)
        self.t_push = float(t_push)
        self.tool_width = float(tool_width_y)
        self.nominal_motion = float(nominal_motion)
        self.robot_vel_scale = float(robot_vel_scale)

        if self.v_robot <= 0:
            raise ValueError("v_robot must be positive")
        if self.tool_width <= 0:
            raise ValueError("tool_width_y must be positive")

    def _calc_exec_time(self) -> float:
        if self.robot_vel_scale <= 0:
            return 30.0
        return self.planning_overhead + self.nominal_motion / self.robot_vel_scale

    # ------------------------------------------------------------------
    # Core solver
    # ------------------------------------------------------------------
    def solve(
        self,
        y_obj: float,
        v_obj: float,
        push_compensation: str = "center",
    ) -> RendezvousResult:
        """
        Compute the rendezvous point for a tracked object.

        Args:
            y_obj: Current object Y position in base_link (m).
            v_obj: Object Y velocity (m/s). NEGATIVE for belt moving toward
                robot (decreasing Y). The sign is handled internally.
            push_compensation: How to offset the target to account for object
                motion DURING the push:
                - "none":    Aim for object at y_r exactly at t_r.
                - "center":  Aim for object center at push midpoint (default).
                - "end":     Aim for object at y_r at push completion.

        Returns:
            RendezvousResult with y_r, t_r, and feasibility flag.
        """
        v_o = abs(float(v_obj))  # Belt speed magnitude (always positive)
        v_r = self.v_robot

        # --------------------------------------------------------------
        # 1. Effective object position after push compensation
        # --------------------------------------------------------------
        # During push, the object drifts downstream by v_o * t_push.
        # We offset the problem so the rendezvous y_r corresponds to the
        # desired contact point during the push window.
        if push_compensation == "center":
            # Object center should pass through y_r at t_r + t_push/2
            # Belt moves toward -Y, so object is at y_r EARLIER in time.
            # => effective start position is y_obj - v_o * t_push/2
            y_eff = float(y_obj) - v_o * self.t_push / 2.0
        elif push_compensation == "end":
            # Object should reach y_r at t_r + t_push
            # => effective start position is y_obj - v_o * t_push
            y_eff = float(y_obj) - v_o * self.t_push
        elif push_compensation == "none":
            y_eff = float(y_obj)
        else:
            raise ValueError(f"Unknown push_compensation: {push_compensation}")

        # --------------------------------------------------------------
        # 2. Critical distance test
        # --------------------------------------------------------------
        # y_critical is the closest an object can be (upstream) and still
        # allow the robot to meet it upstream of (or at) y_park.
        # Derived from: t_overhead + 0 = (y_critical - y_park) / v_o
        # => y_critical = y_park + t_overhead * v_o
        y_critical = self.y_park + self.t_overhead * v_o

        # --------------------------------------------------------------
        # 3. Case A: Object far upstream → rendezvous upstream (y_r >= y_park)
        # --------------------------------------------------------------
        if y_eff >= y_critical:
            # Robot moves upstream (+Y) to meet object earlier.
            # Both approach each other: closing speed = v_r + v_o
            #
            # t_r = t_overhead + (y_r - y_park) / v_r      [robot]
            # t_r = (y_eff - y_r) / v_o                    [object]
            #
            # Solving:
            #   y_r = (v_r * y_eff + v_o * y_park - t_overhead * v_r * v_o)
            #         / (v_r + v_o)
            denom = v_r + v_o
            y_r = (v_r * y_eff + v_o * self.y_park - self.t_overhead * v_r * v_o) / denom
            t_r = self.t_overhead + (y_r - self.y_park) / v_r

            if y_r > y_eff:
                return RendezvousResult(
                    y_r, t_r, False,
                    f"Math error: y_r={y_r:.4f} upstream of y_eff={y_eff:.4f}"
                )
            if y_r > self.y_max:
                return RendezvousResult(
                    y_r, t_r, False,
                    f"y_r={y_r:.4f} exceeds workspace max {self.y_max:.4f}"
                )

        # --------------------------------------------------------------
        # 4. Case B: Object close, robot faster → rendezvous downstream
        # --------------------------------------------------------------
        elif v_r > v_o:
            # Robot moves downstream (-Y) and catches up from behind.
            # Robot is faster, so it can overtake the belt.
            #
            # t_r = t_overhead + (y_park - y_r) / v_r      [robot]
            # t_r = (y_eff - y_r) / v_o                    [object]
            #
            # Solving:
            #   y_r = (v_r * y_eff - v_o * y_park - t_overhead * v_r * v_o)
            #         / (v_r - v_o)
            denom = v_r - v_o
            y_r = (v_r * y_eff - v_o * self.y_park - self.t_overhead * v_r * v_o) / denom
            t_r = self.t_overhead + (self.y_park - y_r) / v_r

            if y_r >= self.y_park:
                return RendezvousResult(
                    y_r, t_r, False,
                    f"Case B math inconsistency: y_r={y_r:.4f} >= y_park={self.y_park:.4f}"
                )
            if y_r < self.y_min:
                return RendezvousResult(
                    y_r, t_r, False,
                    f"y_r={y_r:.4f} below workspace min {self.y_min:.4f}"
                )

        # --------------------------------------------------------------
        # 5. Case C: Uncatchable
        # --------------------------------------------------------------
        else:
            return RendezvousResult(
                self.y_park, float('inf'), False,
                f"Uncatchable: y_eff={y_eff:.4f} < crit={y_critical:.4f} and "
                f"v_robot={v_r:.4f} <= v_belt={v_o:.4f}"
            )

        # --------------------------------------------------------------
        # 6. Validate tool contact during push
        # --------------------------------------------------------------
        y_drift = v_o * self.t_push
        if y_drift > self.tool_width:
            return RendezvousResult(
                y_r, t_r, False,
                f"Object drifts {y_drift:.4f}m during push, "
                f"exceeds tool width {self.tool_width:.4f}m"
            )

        return RendezvousResult(y_r, t_r, True, "OK")

    # ------------------------------------------------------------------
    # Batch solver for multiple objects
    # ------------------------------------------------------------------
    def solve_batch(
        self,
        objects: list,
        push_compensation: str = "center",
    ) -> list:
        """
        Solve rendezvous for multiple tracked objects.

        Args:
            objects: List of dicts with keys 'id', 'y', 'vy'.
            push_compensation: Same as solve().

        Returns:
            List of dicts with 'id', 'result' (RendezvousResult).
        """
        results = []
        for obj in objects:
            res = self.solve(
                y_obj=obj["y"],
                v_obj=obj["vy"],
                push_compensation=push_compensation,
            )
            results.append({"id": obj.get("id", None), "result": res})
        return results

    # ------------------------------------------------------------------
    # Online parameter estimation helpers
    # ------------------------------------------------------------------
    @staticmethod
    def estimate_robot_cartesian_speed(
        joint_speed_rad_s: float,
        velocity_scale: float,
        lever_arm_m: float = 0.35,
    ) -> float:
        """
        Rough estimate of Cartesian Y speed from joint parameters.

        Args:
            joint_speed_rad_s: Max joint speed (rad/s), e.g. 3.14 for myCobot.
            velocity_scale: MoveIt velocity scaling factor (0.0–1.0).
            lever_arm_m: Approximate lever arm from base to workspace (m).

        Returns:
            Estimated Cartesian speed in m/s.
        """
        return joint_speed_rad_s * velocity_scale * lever_arm_m

    @staticmethod
    def estimate_push_duration(
        nominal_motion_s: float,
        velocity_scale: float,
        planning_overhead_s: float,
    ) -> float:
        """
        Estimate total push-cycle duration from park back to park.

        For a 5-step sequence (hover→orient→lower→push→lift→park), the
        push-relevant portion is typically ~60% of total motion time.
        """
        total_motion = planning_overhead_s + nominal_motion_s / velocity_scale
        return total_motion * 0.6


# ===================================================================
# Demo / self-test
# ===================================================================
def demo():
    print("=" * 60)
    print("Conveyor Rendezvous Solver — Demo")
    print("=" * 60)

    # myCobot 320 M5 typical parameters
    solver = ConveyorRendezvousSolver(
        y_park=0.255,          # Fixed EE Y when parked
        v_robot=0.035,         # ~3.5 cm/s Cartesian Y at 10% scale
        t_overhead=2.0,        # MoveIt plan + network + perception
        y_workspace_min=-0.10,
        y_workspace_max=0.35,
        t_push=3.0,            # orient + lower + push + lift
        tool_width_y=0.06,     # 6 cm pusher width
    )

    # Scenarios
    scenarios = [
        {"name": "Far upstream (easy catch)",    "y": 0.45, "vy": -0.02},
        {"name": "Medium distance (upstream meet)", "y": 0.30, "vy": -0.02},
        {"name": "Close, robot faster (downstream chase)", "y": 0.26, "vy": -0.015},
        {"name": "Too close, robot slow (fail)",  "y": 0.26, "vy": -0.05},
        {"name": "Fast belt, drift too large",    "y": 0.40, "vy": -0.08},
    ]

    print(f"\nSolver config:")
    print(f"  Park Y      = {solver.y_park:.3f} m")
    print(f"  Robot speed = {solver.v_robot:.3f} m/s")
    print(f"  Overhead    = {solver.t_overhead:.1f} s")
    print(f"  Push time   = {solver.t_push:.1f} s")
    print(f"  Tool width  = {solver.tool_width:.3f} m")
    print(f"  Workspace   = [{solver.y_min:.2f}, {solver.y_max:.2f}] m")
    print()

    for s in scenarios:
        res = solver.solve(y_obj=s["y"], v_obj=s["vy"], push_compensation="center")
        status = "✅ FEASIBLE" if res.feasible else "❌ INFEASIBLE"
        print(f"{s['name']:<35} | y_obj={s['y']:+.3f}  vy={s['vy']:+.3f}")
        print(f"  → y_r={res.y_r:+.4f} m  t_r={res.t_r:.2f} s  [{status}]")
        if not res.feasible:
            print(f"     Reason: {res.reason}")
        print()

    # Estimate parameters from known robot specs
    print("-" * 60)
    print("Parameter estimation from robot specs:")
    v_est = ConveyorRendezvousSolver.estimate_robot_cartesian_speed(
        joint_speed_rad_s=3.14, velocity_scale=0.10, lever_arm_m=0.35
    )
    print(f"  Estimated v_robot at 10% scale ≈ {v_est:.3f} m/s")
    t_push_est = ConveyorRendezvousSolver.estimate_push_duration(
        nominal_motion_s=0.70, velocity_scale=0.10, planning_overhead_s=2.0
    )
    print(f"  Estimated push duration ≈ {t_push_est:.1f} s")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Conveyor Rendezvous Solver")
    parser.add_argument("--demo", action="store_true", help="Run self-test demo")
    args = parser.parse_args()

    if args.demo:
        demo()
    else:
        parser.print_help()
