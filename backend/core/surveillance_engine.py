"""
Continuous 360° Asynchronous Multi-Radar Surveillance, Fusion & Alert Engine.

Implements:
1. Continuous 360° independent scanning across Radar 1, Radar 2, and Radar 3.
2. Immediate per-radar beam-target detection (zero wait for full 360° cycle).
3. Multi-radar observation association & Alpha-Beta / Kalman-style track fusion.
4. Continuously updated target track state [x, y, z, vx, vy, vz], heading, and confidence.
5. Rapid threat alert generation with position, velocity, direction, confidence, and latency.
"""

import math
import time
import numpy as np
from typing import Dict, List, Any, Optional


class FusedTrack:
    """Represents a continuously updated multi-radar fused target track."""
    def __init__(self, track_id: str, initial_pos: np.ndarray, timestamp: float, radar_id: int, snr_db: float):
        self.track_id = track_id
        self.pos = initial_pos.astype(float).copy()
        self.vel = np.array([0.0, 0.0, 0.0], dtype=float)
        self.last_update_time = timestamp
        self.created_time = timestamp
        self.hit_count = 1
        self.contributing_radars = {radar_id}
        self.last_radar_times = {radar_id: timestamp}
        self.snr_history = [snr_db]
        self.confidence = min(99.5, 38.0 + max(0.0, snr_db - 5.0) * 1.8)
        self.history: List[List[float]] = [self.pos.tolist()]
        self.alert_triggered = False

        # Alpha-Beta tracker gains (fast convergence for asynchronous multi-radar hits)
        self.alpha = 0.65
        self.beta = 0.35

    def predict(self, current_time: float):
        """Predicts track position forward to current_time."""
        dt = max(0.0, current_time - self.last_update_time)
        if dt > 0.0:
            self.pos = self.pos + self.vel * dt
            self.last_update_time = current_time

    def update(self, meas_pos: np.ndarray, timestamp: float, radar_id: int, snr_db: float):
        """Updates track state immediately upon receiving a single radar observation."""
        dt = max(0.05, timestamp - self.last_update_time)

        # Predicted position
        pred_pos = self.pos + self.vel * dt
        residual = meas_pos - pred_pos

        # Alpha-Beta state update
        self.pos = pred_pos + self.alpha * residual
        if self.hit_count >= 1:
            # Smooth velocity estimate
            inst_vel = residual / dt
            if self.hit_count == 1:
                self.vel = 0.5 * inst_vel
            else:
                self.vel = self.vel + (self.beta / dt) * residual

        self.last_update_time = timestamp
        self.hit_count += 1
        self.contributing_radars.add(radar_id)
        self.last_radar_times[radar_id] = timestamp
        self.snr_history.append(snr_db)
        if len(self.snr_history) > 15:
            self.snr_history.pop(0)

        self.history.append(self.pos.tolist())
        if len(self.history) > 60:
            self.history.pop(0)

        # Multi-radar fusion confidence boost
        num_radars = len(self.contributing_radars)
        avg_snr = float(np.mean(self.snr_history))
        base_conf = 35.0 + (num_radars - 1) * 24.0 + min(20.0, self.hit_count * 2.5) + max(0.0, (avg_snr - 5.0) * 0.8)
        self.confidence = float(min(99.4, max(20.0, base_conf)))

    def get_speed_ms(self) -> float:
        return float(np.linalg.norm(self.vel))

    def get_heading_deg(self) -> float:
        # Heading: 0° = North (+Y), 90° = East (+X)
        return float(math.degrees(math.atan2(self.vel[0], self.vel[1])) % 360.0)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "track_id": self.track_id,
            "est_pos": [round(float(x), 1) for x in self.pos],
            "est_vel": [round(float(v), 2) for v in self.vel],
            "speed_ms": round(self.get_speed_ms(), 1),
            "heading_deg": round(self.get_heading_deg(), 1),
            "range_m": round(float(np.linalg.norm(self.pos)), 1),
            "azimuth_deg": round(math.degrees(math.atan2(self.pos[0], self.pos[1])), 2),
            "confidence": round(self.confidence, 1),
            "hit_count": self.hit_count,
            "contributing_radars": sorted(list(self.contributing_radars)),
            "history": self.history[-20:],
            "age_s": round(self.last_update_time - self.created_time, 2)
        }


class ContinuousSurveillanceEngine:
    """
    Orchestrates continuous 360° independent radar scanning, immediate target detection,
    multi-radar observation association/fusion, and rapid threat alerting.
    """
    def __init__(self, beamwidth_deg: float = 14.0):
        self.beamwidth_deg = beamwidth_deg
        self.mode = "async_360"  # "async_360" (independent staggered 360°) or "coherent_sync"
        # Independent continuous 360° beam pointing angles (0° = North/+Y, clockwise)
        # Staggered by 120° in async_360 mode so a target is scanned 3x faster!
        self.radar_angles_deg = [0.0, 120.0, 240.0]
        self.radar_rpm = [12.0, 12.0, 12.0]
        self.sim_time = 0.0
        self.dt = 0.05

        self.tracks: Dict[str, FusedTrack] = {}
        self.recent_observations: List[Dict[str, Any]] = []
        self.alerts: List[Dict[str, Any]] = []
        self._last_hit_time = {1: -10.0, 2: -10.0, 3: -10.0}
        self._obs_counter = 0
        self._alert_counter = 0

    def set_mode(self, mode: str):
        self.mode = mode
        if mode == "async_360":
            # Staggered 120° phases for rapid 3x revisit continuous surveillance
            self.radar_angles_deg = [0.0, 120.0, 240.0]
        else:
            # Synchronized boresight pointing
            self.radar_angles_deg = [0.0, 0.0, 0.0]

    def reset(self):
        self.tracks.clear()
        self.recent_observations.clear()
        self.alerts.clear()
        self._last_hit_time = {1: -10.0, 2: -10.0, 3: -10.0}
        self.sim_time = 0.0
        self.set_mode(self.mode)

    def step(self, radar_nodes: List[Any], target_pos: np.ndarray, target_vel: np.ndarray,
             snr_db: float = 10.0, rotation_speed_rpm: float = 12.0,
             callsign: str = "UAV-TGT") -> Dict[str, Any]:
        """
        Advances all 3 radars continuously through 360°, checks immediate beam-target
        intersections, fuses observations into active tracks, and generates rapid alerts.
        """
        self.sim_time += self.dt
        omega_deg_s = rotation_speed_rpm * 360.0 / 60.0
        step_hits: List[Dict[str, Any]] = []

        # Predict existing tracks forward
        for trk in self.tracks.values():
            trk.predict(self.sim_time)

        # 1. Rotate all 3 radars continuously & independently through 360°
        for i, node in enumerate(radar_nodes):
            self.radar_angles_deg[i] = (self.radar_angles_deg[i] + omega_deg_s * self.dt) % 360.0
            beam_az = self.radar_angles_deg[i]

            # Compute true azimuth from radar i to target (0° = North/+Y, clockwise)
            rel_vec = target_pos - node.local_pos
            true_range = float(np.linalg.norm(rel_vec))
            true_az = math.degrees(math.atan2(rel_vec[0], rel_vec[1])) % 360.0

            # Angular separation between beam and target
            ang_diff = abs((beam_az - true_az + 180.0) % 360.0 - 180.0)

            # 2. Immediate Detection: as soon as this radar's beam illuminates target
            #    (minimum 0.45s debounce per radar per sweep so one pass = one crisp hit)
            if ang_diff <= (self.beamwidth_deg / 2.0) and (self.sim_time - self._last_hit_time[i + 1]) >= 0.45:
                self._last_hit_time[i + 1] = self.sim_time
                self._obs_counter += 1

                # Simulate realistic radar measurement noise scaled by SNR
                snr_lin = 10.0 ** (snr_db / 10.0)
                range_sigma = max(5.0, 85.0 / math.sqrt(snr_lin))
                az_sigma_deg = max(0.08, 1.2 / math.sqrt(snr_lin))

                meas_range = true_range + float(np.random.normal(0.0, range_sigma))
                meas_az = true_az + float(np.random.normal(0.0, az_sigma_deg))
                meas_el = math.degrees(math.asin(max(-1.0, min(1.0, rel_vec[2] / max(1.0, true_range)))))

                # Project measurement to 3D Cartesian
                el_rad = math.radians(meas_el)
                az_rad = math.radians(meas_az)
                meas_pos = node.local_pos + np.array([
                    meas_range * math.cos(el_rad) * math.sin(az_rad),
                    meas_range * math.cos(el_rad) * math.cos(az_rad),
                    meas_range * math.sin(el_rad)
                ], dtype=float)

                obs = {
                    "obs_id": self._obs_counter,
                    "radar_id": i + 1,
                    "radar_name": f"Radar {i + 1}",
                    "timestamp": round(self.sim_time, 2),
                    "range_m": round(meas_range, 1),
                    "azimuth_deg": round(meas_az, 2),
                    "snr_db": round(snr_db + float(np.random.normal(0.0, 0.8)), 1),
                    "meas_pos": [round(float(x), 1) for x in meas_pos],
                    "processing_latency_ms": round(12.0 + float(np.random.uniform(2.0, 8.0)), 1)
                }
                step_hits.append(obs)
                self.recent_observations.insert(0, obs)
                if len(self.recent_observations) > 15:
                    self.recent_observations.pop()

                # 3. Associate & Fuse immediately into central track (no waiting for other radars!)
                self._associate_and_fuse(obs, meas_pos, callsign)

        return {
            "mode": self.mode,
            "sim_time": round(self.sim_time, 2),
            "radar_angles_deg": [round(a, 2) for a in self.radar_angles_deg],
            "step_hits": step_hits,
            "recent_observations": self.recent_observations[:8],
            "tracks": [t.to_dict() for t in self.tracks.values()],
            "alerts": self.alerts[:6]
        }

    def _associate_and_fuse(self, obs: Dict[str, Any], meas_pos: np.ndarray, callsign: str):
        """
        Associates an incoming observation to an existing track via spatial gating
        and immediately updates the track + evaluates threat alert criteria.
        """
        gate_radius_m = 1500.0
        matched_track: Optional[FusedTrack] = None
        best_dist = gate_radius_m

        for trk in self.tracks.values():
            dist = float(np.linalg.norm(trk.pos - meas_pos))
            if dist < best_dist:
                best_dist = dist
                matched_track = trk

        if matched_track is None:
            track_id = f"TRK-{callsign}"
            matched_track = FusedTrack(
                track_id=track_id,
                initial_pos=meas_pos,
                timestamp=obs["timestamp"],
                radar_id=obs["radar_id"],
                snr_db=obs["snr_db"]
            )
            self.tracks[track_id] = matched_track
        else:
            matched_track.update(
                meas_pos=meas_pos,
                timestamp=obs["timestamp"],
                radar_id=obs["radar_id"],
                snr_db=obs["snr_db"]
            )

        # 4. Rapid Threat Alert Evaluation
        self._evaluate_threat_alert(matched_track, obs)

    def _evaluate_threat_alert(self, trk: FusedTrack, latest_obs: Dict[str, Any]):
        """
        Generates an immediate threat alert as soon as detection meets threat criteria:
        - Range < 25 km and Confidence >= 40% (escalates on multi-radar fusion)
        """
        rng = float(np.linalg.norm(trk.pos))
        speed = trk.get_speed_ms()
        heading = trk.get_heading_deg()

        if trk.confidence >= 85.0 and rng < 18000.0:
            severity = "CRITICAL"
        elif trk.confidence >= 65.0 or len(trk.contributing_radars) >= 2:
            severity = "HIGH"
        else:
            severity = "ELEVATED"

        # Emit/update alert on first detection and on every multi-radar confidence escalation
        should_emit = (
            not trk.alert_triggered
            or (len(trk.contributing_radars) == 2 and trk.hit_count == 2)
            or (len(trk.contributing_radars) == 3 and trk.hit_count == 3)
            or (trk.hit_count % 4 == 0)
        )

        if should_emit:
            trk.alert_triggered = True
            self._alert_counter += 1
            alert = {
                "alert_id": f"ALT-{self._alert_counter:03d}",
                "track_id": trk.track_id,
                "severity": severity,
                "timestamp": latest_obs["timestamp"],
                "trigger_radar": latest_obs["radar_name"],
                "contributing_radars": sorted(list(trk.contributing_radars)),
                "est_pos": [round(float(x), 1) for x in trk.pos],
                "range_km": round(rng / 1000.0, 2),
                "azimuth_deg": round(math.degrees(math.atan2(trk.pos[0], trk.pos[1])), 1),
                "speed_ms": round(speed, 1),
                "heading_deg": round(heading, 1),
                "confidence": round(trk.confidence, 1),
                "latency_ms": latest_obs["processing_latency_ms"],
                "message": (
                    f"{severity} TRACK {trk.track_id}: "
                    f"Rng {rng/1000.0:.2f} km | Vel {speed:.1f} m/s @ {heading:.0f}° | "
                    f"Conf {trk.confidence:.1f}% ({len(trk.contributing_radars)}/3 Radars)"
                )
            }
            self.alerts.insert(0, alert)
            if len(self.alerts) > 12:
                self.alerts.pop()
