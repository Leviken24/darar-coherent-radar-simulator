"""
Physical Simulation Engine for Distributed Aperture Rotating Array Radar (DARAR).

Implements the physical geometry, geodetic baseline calculations, target kinematics,
electromagnetic propagation delays (tau_lk = (R_k + R_l)/c), array rotation dynamics,
and alpha-filter servo synchronization from the research paper:
"Distributed coherent detection method for rotating array radar"
Journal of Systems Engineering and Electronics, Vol. 37, No. 5, October 2026.
"""

import math
import numpy as np
from typing import Dict, List, Tuple, Any

C_LIGHT = 299792458.0  # Speed of light in m/s
EARTH_RADIUS = 6371000.0  # Earth radius in meters


class RadarNode:
    """Represents a single rotating array radar node in the DARAR system."""
    def __init__(self, node_id: int, name: str, lon: float, lat: float, alt: float,
                 local_pos: Tuple[float, float, float], initial_servo_angle: float):
        self.node_id = node_id
        self.name = name
        self.lon = lon  # Longitude in degrees
        self.lat = lat  # Latitude in degrees
        self.alt = alt  # Altitude in meters
        self.local_pos = np.array(local_pos, dtype=float)  # (x, y, z) in meters
        self.initial_servo_angle = initial_servo_angle  # theta_i^0 in degrees
        self.current_servo_angle = initial_servo_angle  # psi_i^k
        self.target_pointing_angle = 0.0  # theta_i^k
        self.rotation_history: List[float] = [initial_servo_angle]
        self.error_history: List[float] = []

    def get_world_pointing_angle(self) -> float:
        """Returns the actual beam pointing angle in the world horizontal plane (degrees)."""
        # World pointing = current servo angle - initial mechanical offset
        return (self.current_servo_angle - self.initial_servo_angle) % 360.0


class DroneTarget:
    """Represents the cooperative or low-observable drone target."""
    def __init__(self, initial_pos: Tuple[float, float, float] = (0.0, 10000.0, 500.0),
                 velocity: Tuple[float, float, float] = (20.0, 0.0, 0.0),
                 rcs: float = 0.01):
        self.pos = np.array(initial_pos, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.rcs = rcs  # Radar cross section in m^2 (e.g. 0.01 m^2 for small drone)
        self.trajectory_history: List[List[float]] = [self.pos.tolist()]

    def update(self, dt: float):
        """Update drone position over time interval dt (seconds)."""
        self.pos += self.velocity * dt
        self.trajectory_history.append(self.pos.tolist())
        if len(self.trajectory_history) > 200:
            self.trajectory_history.pop(0)


class DARARPhysicsEngine:
    """
    Physical simulation engine orchestrating 3 radar nodes, central control,
    servo synchronization via alpha-filter, and true propagation delay calculations.
    """
    def __init__(self, baseline_length: float = 60.0, target_dist: float = 10000.0,
                 drone_speed: float = 20.0):
        self.baseline_length = baseline_length
        self.c = C_LIGHT
        self.earth_radius = EARTH_RADIUS

        # Setup standard 3-node system (Table 1 & Section 5.1):
        # Reference station is Radar 1 at center (0, 0, 0)
        # Radar 2 is at -baseline_length on X axis (e.g., -60m or -400m)
        # Radar 3 is at +baseline_length on X axis (e.g., +60m or +400m)
        # Initial mechanical offsets: Radar 1: 0 deg, Radar 2: 30 deg, Radar 3: 330 deg (-30 deg)
        # Compute exact longitude offset accounting for latitude cos(lat) factor
        lat_ref = 39.907
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(lat_ref))
        delta_lon_deg = baseline_length / meters_per_deg_lon

        self.nodes = [
            RadarNode(1, "Radar 1 (Reference)", lon=116.397, lat=lat_ref, alt=50.0,
                      local_pos=(0.0, 0.0, 0.0), initial_servo_angle=0.0),
            RadarNode(2, "Radar 2 (West)", lon=116.397 - delta_lon_deg, lat=lat_ref, alt=50.0,
                      local_pos=(-baseline_length, 0.0, 0.0), initial_servo_angle=30.0),
            RadarNode(3, "Radar 3 (East)", lon=116.397 + delta_lon_deg, lat=lat_ref, alt=50.0,
                      local_pos=(baseline_length, 0.0, 0.0), initial_servo_angle=330.0),
        ]

        # Central processing station coordinates
        self.central_pos = np.array([0.0, -20.0, 0.0], dtype=float)

        # Drone target setup
        self.target = DroneTarget(initial_pos=(0.0, target_dist, 500.0),
                                 velocity=(drone_speed, 0.0, 0.0))

        # Rotation & servo parameters
        self.rotation_speed_rpm = 10.0  # 10 RPM = 60 deg/sec
        self.cycle_index = 0
        self.dt_cycle = 0.05  # 50 ms per cycle
        self.is_synchronized = False

    def reset(self, baseline_length: float = 60.0, target_dist: float = 10000.0,
              drone_speed: float = 20.0):
        """Reset the physical engine with new parameters."""
        self.__init__(baseline_length, target_dist, drone_speed)

    def compute_geodetic_baseline(self, node_i: RadarNode, node_ref: RadarNode) -> float:
        """
        Calculates baseline distance Di between radar i and reference radar using Equation (13):
        Di = sqrt((H1 - Hi)^2 + (R * arccos(cos(B1)*cos(Bi)*cos(L1 - Li) + sin(B1)*sin(Bi)))^2)
        """
        l1, b1, h1 = math.radians(node_ref.lon), math.radians(node_ref.lat), node_ref.alt
        li, bi, hi = math.radians(node_i.lon), math.radians(node_i.lat), node_i.alt

        # Great circle arc argument
        arg = math.cos(b1) * math.cos(bi) * math.cos(l1 - li) + math.sin(b1) * math.sin(bi)
        arg = max(-1.0, min(1.0, arg))
        great_circle_dist = self.earth_radius * math.acos(arg)

        d_i = math.sqrt((h1 - hi) ** 2 + great_circle_dist ** 2)
        return d_i

    def compute_coarse_time_delay(self, node_index: int, target_azimuth_deg: float) -> float:
        """
        Calculates coarse compensation time parameter for radar i using Equations (14) & (15):
        |Delta t_i| = Delta r_i / c = (D_i * sin(theta)) / c
        Delta t_i = sgn(L_i - L_1) * (D_i * sin(theta)) / c
        """
        ref_node = self.nodes[0]
        node_i = self.nodes[node_index]
        if node_index == 0:
            return 0.0

        d_i = np.linalg.norm(node_i.local_pos - ref_node.local_pos)
        theta_rad = math.radians(target_azimuth_deg)

        # Sign from relative position along baseline direction
        sign = 1.0 if (node_i.local_pos[0] - ref_node.local_pos[0]) > 0 else -1.0
        delta_t_i = sign * (d_i * math.sin(theta_rad)) / self.c
        return delta_t_i

    def step_servo_synchronization(self) -> Dict[str, Any]:
        """
        Executes one synchronization cycle of the parallel control model (Section 3.1, Eq. 10-12):
        1. Direction difference: Delta theta_i = theta_i^0 - theta_1^0
        2. Central instruction for cycle k: theta^k
        3. Unit controller command: theta_i^k = theta^k + Delta theta_i
        4. Alpha filter smoothing: psi_i^k = alpha * theta_i^k + (1 - alpha) * psi_i^{k-1}
        """
        self.cycle_index += 1
        ref_node = self.nodes[0]

        # Central controller command for cycle k: rotating at rotation_speed_rpm
        omega_deg_per_sec = self.rotation_speed_rpm * 360.0 / 60.0
        theta_k = (self.cycle_index * omega_deg_per_sec * self.dt_cycle) % 360.0

        # Dynamic alpha schedule: alpha starts near 0.85 for rapid adaptation,
        # then smoothly decreases towards 0.05 once error is small (Section 3.1)
        sync_errors = []
        for i, node in enumerate(self.nodes):
            delta_theta_i = node.initial_servo_angle - ref_node.initial_servo_angle
            target_servo_cmd = (theta_k + delta_theta_i) % 360.0
            node.target_pointing_angle = target_servo_cmd

            # Angular difference handling modulo 360
            angle_diff = (target_servo_cmd - node.current_servo_angle + 180.0) % 360.0 - 180.0

            # Dynamic alpha determination based on cycle and current error
            if self.cycle_index < 15:
                alpha = 0.85
            elif self.cycle_index < 40:
                alpha = 0.45
            else:
                alpha = 0.12

            # Alpha filter update
            smoothed_angle = node.current_servo_angle + alpha * angle_diff
            node.current_servo_angle = smoothed_angle % 360.0
            node.rotation_history.append(node.current_servo_angle)

            # Record pointing error relative to reference radar's world beam direction
            ref_world_angle = ref_node.get_world_pointing_angle()
            node_world_angle = node.get_world_pointing_angle()
            error = abs((node_world_angle - ref_world_angle + 180.0) % 360.0 - 180.0)
            node.error_history.append(error)
            sync_errors.append(error)

        max_error = max(sync_errors)
        self.is_synchronized = (max_error < 0.1)

        # Update target motion
        self.target.update(self.dt_cycle)

        return {
            "cycle": self.cycle_index,
            "central_command_deg": theta_k,
            "servo_angles": [n.current_servo_angle for n in self.nodes],
            "world_pointing_angles": [n.get_world_pointing_angle() for n in self.nodes],
            "sync_errors_deg": sync_errors,
            "max_error_deg": max_error,
            "is_synchronized": self.is_synchronized,
            "drone_pos": self.target.pos.tolist(),
        }

    def compute_all_propagation_delays(self) -> np.ndarray:
        """
        Computes the physical propagation times tau_lk for all 9 channel combinations:
        tau_lk = (R_k + R_l) / c
        Returns 3x3 matrix where [l, k] is delay from transmitter k to receiver l.
        """
        k_num = len(self.nodes)
        l_num = len(self.nodes)
        delays = np.zeros((l_num, k_num), dtype=float)

        target_pos = self.target.pos
        # Distance from each radar transmitter to target
        r_ranges = [np.linalg.norm(target_pos - node.local_pos) for node in self.nodes]

        for l in range(l_num):
            for k in range(k_num):
                # Total two-way propagation distance
                d_lk = r_ranges[k] + r_ranges[l]
                delays[l, k] = d_lk / self.c

        return delays

    def get_antenna_gain_pattern(self, pointing_error_deg: float) -> float:
        """
        Computes 32-element linear array normalized antenna gain (Fig. 2a):
        AF(theta) = |sin(N * pi * d/lambda * sin(theta)) / (N * sin(pi * d/lambda * sin(theta)))|
        with N = 32, d = lambda / 2
        """
        N = 32
        theta_rad = math.radians(pointing_error_deg)
        denom_arg = 0.5 * math.pi * math.sin(theta_rad)
        if abs(denom_arg) < 1e-9:
            return 1.0
        num = math.sin(N * denom_arg)
        denom = N * math.sin(denom_arg)
        gain = abs(num / denom)
        return max(1e-4, gain)

    def compute_beam_pointing_gain_loss_db(self, pointing_error_deg: float) -> float:
        """Computes synthesis gain loss in dB due to beam pointing error (Fig. 2b)."""
        gain = self.get_antenna_gain_pattern(pointing_error_deg)
        # Two-way loss considering transmit and receive patterns
        loss_db = 20.0 * math.log10(gain)
        return loss_db
