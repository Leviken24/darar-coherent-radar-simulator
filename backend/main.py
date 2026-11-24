"""
FastAPI Simulation Backend for Distributed Aperture Rotating Array Radar (DARAR).

Provides REST and WebSocket interfaces for:
1. Physical radar kinematics, servo rotation dynamics, and drone target trajectory
2. 9-channel electromagnetic signal propagation, pulse compression, and noise injection
3. Stage-by-stage radar signal processing:
   - Stage 1: Spatial synchronization (parallel control model, alpha-filter)
   - Stage 2: Signal transmission & reception (tau_lk = (R_k + R_l)/c)
   - Stage 3: Multi-channel signal laboratory (9 oscilloscope channels)
   - Stage 4: First-level coarse time compensation (INS baseline & beam pointing)
   - Stage 5: Second-level Regional Cross-Correlation (RCC: 3, 5, 7 points)
   - Stage 6: Coherent signal synthesis and quantitative performance metrics
4. Live Monte Carlo benchmark reproducing Figure 13 (PP vs CC vs RCC vs Theoretical limit)
"""

import sys
import os
import math
import asyncio
from typing import Dict, List, Any, Optional
from pydantic import BaseModel, Field
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import numpy as np

# Ensure core modules are on path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.physics_engine import DARARPhysicsEngine, C_LIGHT
from core.signal_engine import DARARSignalEngine
from core.benchmark_engine import DARARBenchmarkEngine
from core.live_dataset_engine import LiveDatasetEngine
from core.analytics_engine import DARARAnalyticsEngine

app = FastAPI(
    title="DARAR Distributed Coherent Radar Simulation API",
    description="Interactive Engineering Digital Twin for Distributed Rotating Array Radar",
    version="1.0.0"
)

# Enable CORS for frontend development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mount frontend directory for static assets and index.html
frontend_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../frontend"))
if os.path.exists(frontend_dir):
    app.mount("/static", StaticFiles(directory=frontend_dir), name="static")

@app.get("/")
def serve_index():
    index_file = os.path.join(frontend_dir, "index.html")
    if os.path.exists(index_file):
        return FileResponse(index_file)
    return {"message": "DARAR API online. Frontend files not found at " + index_file}

# Global simulation state
physics_engine = DARARPhysicsEngine(baseline_length=60.0, target_dist=10000.0, drone_speed=20.0)
signal_engine = DARARSignalEngine(snr_db=10.0, rcc_window_points=5, num_samples=128)
live_dataset_engine = LiveDatasetEngine()
analytics_engine = DARARAnalyticsEngine()
benchmark_cache: Optional[Dict[str, Any]] = None
active_target_metadata: Dict[str, Any] = {
    "source": "simulated_uav",
    "callsign": "UAV-SIM-01",
    "icao24": "SIM-001",
    "radial_velocity_ms": 20.0
}


class SimParams(BaseModel):
    baseline_length: float = Field(default=60.0, description="Radar baseline distance in meters (e.g. 60m to 400m)")
    drone_distance: float = Field(default=10000.0, description="Target initial distance in meters (e.g. 10km to 17km)")
    drone_speed: float = Field(default=20.0, description="Target velocity in m/s")
    drone_altitude: float = Field(default=500.0, description="Target altitude in meters")
    snr_db: float = Field(default=10.0, description="Single radar SNR in dB (e.g. 5 to 25 dB)")
    rcc_window_points: int = Field(default=5, description="RCC search window in samples (3, 5, or 7)")
    rotation_speed_rpm: float = Field(default=10.0, description="Radar rotation speed in RPM")
    carrier_freq_hz: float = Field(default=5.5e9, description="Radar carrier frequency (C-band: 5.5 GHz)")
    signal_bandwidth_hz: float = Field(default=5.0e6, description="Signal bandwidth in Hz (5 MHz)")
    ins_error_m: float = Field(default=0.5, description="INS baseline position measurement error in meters")
    phase_jitter_deg: float = Field(default=5.0, description="Channel phase synchronization jitter in degrees")


@app.get("/api/status")
def get_status():
    return {
        "status": "online",
        "system": "Distributed Aperture Rotating Array Radar (DARAR)",
        "paper": "Distributed coherent detection method for rotating array radar (JSEE 2026)",
        "baseline_m": physics_engine.baseline_length,
        "is_synchronized": physics_engine.is_synchronized,
        "nodes_count": len(physics_engine.nodes),
        "channels_count": 9
    }


@app.post("/api/physics/reset")
def reset_physics(params: SimParams):
    global physics_engine, signal_engine
    physics_engine.reset(
        baseline_length=params.baseline_length,
        target_dist=params.drone_distance,
        drone_speed=params.drone_speed
    )
    physics_engine.rotation_speed_rpm = params.rotation_speed_rpm
    physics_engine.target.pos[2] = params.drone_altitude

    signal_engine = DARARSignalEngine(
        carrier_freq_hz=params.carrier_freq_hz,
        bandwidth_hz=params.signal_bandwidth_hz,
        snr_db=params.snr_db,
        rcc_window_points=params.rcc_window_points,
        num_samples=128
    )
    return {"message": "Simulation reset successfully", "baseline": params.baseline_length}


@app.post("/api/physics/step")
def step_physics(steps: int = 1):
    result = None
    for _ in range(max(1, min(steps, 20))):
        result = physics_engine.step_servo_synchronization()
    return result


@app.post("/api/simulation/full_cycle")
def run_full_simulation_cycle(params: SimParams):
    """
    Executes a complete 6-stage engineering simulation run based on current parameters.
    All returned data is calculated from the physical and mathematical engines.
    """
    global physics_engine, signal_engine

    # 1. Update physical parameters
    physics_engine.baseline_length = params.baseline_length
    physics_engine.rotation_speed_rpm = params.rotation_speed_rpm
    physics_engine.nodes[1].local_pos[0] = -params.baseline_length
    physics_engine.nodes[2].local_pos[0] = params.baseline_length
    physics_engine.target.pos[1] = params.drone_distance
    physics_engine.target.pos[2] = params.drone_altitude

    # Update signal engine
    signal_engine.snr_db = params.snr_db
    signal_engine.rcc_window = params.rcc_window_points
    signal_engine.fc = params.carrier_freq_hz
    signal_engine.B = params.signal_bandwidth_hz

    # Step servo sync once
    servo_state = physics_engine.step_servo_synchronization()

    # --- STAGE 1: SPATIAL SYNCHRONIZATION DATA ---
    stage1_data = {
        "cycle": servo_state["cycle"],
        "central_command_deg": servo_state["central_command_deg"],
        "nodes": [
            {
                "id": n.node_id,
                "name": n.name,
                "local_pos": n.local_pos.tolist(),
                "initial_servo_angle": n.initial_servo_angle,
                "current_servo_angle": n.current_servo_angle,
                "world_pointing_angle": n.get_world_pointing_angle(),
                "sync_error_deg": servo_state["sync_errors_deg"][i]
            }
            for i, n in enumerate(physics_engine.nodes)
        ],
        "max_sync_error_deg": servo_state["max_error_deg"],
        "is_synchronized": servo_state["is_synchronized"],
        "pointing_gain_loss_db": physics_engine.compute_beam_pointing_gain_loss_db(servo_state["max_error_deg"]),
        "drone_pos": physics_engine.target.pos.tolist(),
        "central_pos": physics_engine.central_pos.tolist()
    }

    # --- STAGE 2: PHYSICAL PROPAGATION DELAYS ---
    delays_matrix = physics_engine.compute_all_propagation_delays()
    two_way_distances = delays_matrix * C_LIGHT
    target_pos = physics_engine.target.pos
    target_azimuth_deg = math.degrees(math.atan2(target_pos[0], target_pos[1]))

    stage2_data = {
        "delays_matrix_seconds": delays_matrix.tolist(),
        "delays_matrix_us": (delays_matrix * 1e6).tolist(),
        "two_way_distances_m": two_way_distances.tolist(),
        "target_azimuth_deg": target_azimuth_deg,
        "c_light": C_LIGHT
    }

    # Add INS baseline measurement error and phase jitter
    measured_baseline = params.baseline_length + np.random.normal(0, params.ins_error_m)
    phase_jitter_rad = np.radians(np.random.normal(0, params.phase_jitter_deg, (3, 3)))
    phase_jitter_rad[0, 0] = 0.0

    # --- STAGE 3: SIGNAL GENERATION & 9-CHANNEL ECHOES ---
    raw_echo_data = signal_engine.synthesize_channel_echoes(
        delays=delays_matrix,
        snr_db=params.snr_db,
        additional_phase_errors=phase_jitter_rad
    )

    channels_data = {}
    for l in range(3):
        for k in range(3):
            ch_key = f"S{l+1}{k+1}"
            sig = raw_echo_data["signals"][l, k, :]
            channels_data[ch_key] = {
                "l": l + 1,
                "k": k + 1,
                "real": np.real(sig).tolist(),
                "imag": np.imag(sig).tolist(),
                "envelope": np.abs(sig).tolist(),
                "peak_amplitude": float(np.max(np.abs(sig))),
                "peak_index": int(np.argmax(np.abs(sig))),
                "true_delay_ns": float(raw_echo_data["true_delays"][l, k] * 1e9),
                "true_phase_deg": float(math.degrees(raw_echo_data["true_phases"][l, k]))
            }

    stage3_data = {
        "channels": channels_data,
        "sample_points": list(range(signal_engine.N_samples)),
        "center_sample": raw_echo_data["center_sample"],
        "snr_db": params.snr_db,
        "noise_sigma": float(raw_echo_data["sigma_w"]),
        "math_model": {
            "s11": "s_{11}(t) = g_{11} A_{11}(t - \\tau_{11}) e^{-j 2\\pi f_c \\tau_{11} - j\\varphi_{11}} + n_{11}(t)",
            "slk": "s_{lk}(t) = g_{lk} A_{lk}(t - \\tau_{lk}) e^{-j 2\\pi f_c \\tau_{lk} - j\\varphi_{lk}} + n_{lk}(t)"
        }
    }

    # --- STAGE 4: FIRST-LEVEL COARSE TIME COMPENSATION ---
    # Compute coarse delay using Equation (14) & (15) with measured baseline
    coarse_delays = np.zeros((3, 3))
    theta_rad = math.radians(target_azimuth_deg)
    d_proj = measured_baseline * math.sin(theta_rad) / C_LIGHT

    coarse_delays[0, :] = [-d_proj / 2, 0.0, d_proj / 2]
    coarse_delays[1, :] = [-d_proj, -d_proj / 2, 0.0]
    coarse_delays[2, :] = [0.0, d_proj / 2, d_proj]

    coarse_echo_data = signal_engine.apply_coarse_time_compensation(raw_echo_data, coarse_delays)

    coarse_channels = {}
    for l in range(3):
        for k in range(3):
            ch_key = f"S{l+1}{k+1}"
            sig = coarse_echo_data["signals"][l, k, :]
            coarse_channels[ch_key] = {
                "envelope": np.abs(sig).tolist(),
                "peak_index": int(np.argmax(np.abs(sig))),
                "applied_shift_samples": int(coarse_echo_data["applied_shifts_samples"][l, k]),
                "coarse_delay_ns": float(coarse_delays[l, k] * 1e9)
            }

    # Synthesize before any compensation for direct comparison
    raw_synth_data = signal_engine.perform_coherent_synthesis(
        {"signals": raw_echo_data["signals"], "time_errors": raw_echo_data["true_delays"], "phase_errors": raw_echo_data["true_phases"]}
    )

    stage4_data = {
        "channels": coarse_channels,
        "formula": "\\Delta t_i = \\text{sgn}(L_i - L_1) \\cdot \\frac{D_i \\sin\\theta}{c}",
        "baseline_used_m": measured_baseline,
        "target_azimuth_deg": target_azimuth_deg
    }

    # --- STAGE 5: SECOND-LEVEL REGIONAL CROSS-CORRELATION (RCC) ---
    rcc_echo_data = signal_engine.estimate_and_compensate_rcc(coarse_echo_data, search_window_points=params.rcc_window_points)
    cc_echo_data = signal_engine.estimate_and_compensate_conventional_cc(coarse_echo_data)
    pp_echo_data = signal_engine.estimate_and_compensate_peak_picking(coarse_echo_data)

    rcc_channels = {}
    for l in range(3):
        for k in range(3):
            ch_key = f"S{l+1}{k+1}"
            sig = rcc_echo_data["signals"][l, k, :]
            rcc_channels[ch_key] = {
                "envelope": np.abs(sig).tolist(),
                "peak_index": int(np.argmax(np.abs(sig))),
                "estimated_time_shift_ns": float(rcc_echo_data["estimated_time_shifts"][l, k] * 1e9),
                "estimated_phase_deg": float(math.degrees(rcc_echo_data["estimated_phases"][l, k])),
                "residual_time_error_ns": float(rcc_echo_data["time_errors"][l, k] * 1e9),
                "residual_phase_error_deg": float(math.degrees(rcc_echo_data["phase_errors"][l, k]))
            }

    stage5_data = {
        "channels": rcc_channels,
        "correlation_curves": rcc_echo_data["correlation_curves"],
        "search_window_points": params.rcc_window_points,
        "search_window_ns": params.rcc_window_points * signal_engine.Ts * 1e9
    }

    # --- STAGE 6: COHERENT SYNTHESIS & BENCHMARKS ---
    synth_rcc = signal_engine.perform_coherent_synthesis(rcc_echo_data)
    synth_cc = signal_engine.perform_coherent_synthesis(cc_echo_data)
    synth_pp = signal_engine.perform_coherent_synthesis(pp_echo_data)
    synth_coarse = signal_engine.perform_coherent_synthesis(coarse_echo_data)

    stage6_data = {
        "uncompensated": {
            "waveform_abs": raw_synth_data["synthesized_signal_abs"],
            "gain_db": raw_synth_data["actual_gain_db"],
            "efficiency_percent": raw_synth_data["efficiency_percent"],
            "snr_db": raw_synth_data["snr_c_db"]
        },
        "coarse_only": {
            "waveform_abs": synth_coarse["synthesized_signal_abs"],
            "gain_db": synth_coarse["actual_gain_db"],
            "efficiency_percent": synth_coarse["efficiency_percent"],
            "snr_db": synth_coarse["snr_c_db"]
        },
        "peak_picking": {
            "waveform_abs": synth_pp["synthesized_signal_abs"],
            "gain_db": synth_pp["actual_gain_db"],
            "efficiency_percent": synth_pp["efficiency_percent"],
            "snr_db": synth_pp["snr_c_db"]
        },
        "conventional_cc": {
            "waveform_abs": synth_cc["synthesized_signal_abs"],
            "gain_db": synth_cc["actual_gain_db"],
            "efficiency_percent": synth_cc["efficiency_percent"],
            "snr_db": synth_cc["snr_c_db"]
        },
        "rcc_fine": {
            "waveform_abs": synth_rcc["synthesized_signal_abs"],
            "gain_db": synth_rcc["actual_gain_db"],
            "ideal_gain_db": synth_rcc["ideal_gain_db"],
            "efficiency_percent": synth_rcc["efficiency_percent"],
            "snr_db": synth_rcc["snr_c_db"],
            "time_errors_ns": synth_rcc["time_errors_ns"],
            "phase_errors_deg": synth_rcc["phase_errors_deg"]
        },
        "theoretical_gain_db": 9.54,
        "single_snr_db": synth_rcc["snr_s_db"],
        "radar_paper_outfield_benchmark": {
            "uav_distance_km": 17,
            "baseline_m": 60,
            "measured_gain_db": 8.69,
            "ideal_gain_db": 9.54,
            "measured_efficiency_percent": 82.3,
            "detection_power_increase_percent": 65.0
        }
    }

    # --- ADVANCED AEROSPACE & DSP ANALYTICS ---
    iq_phasors = analytics_engine.compute_iq_phasors(raw_echo_data, rcc_echo_data, target_bin=raw_echo_data["center_sample"])

    # Compute radial velocity and Doppler shift fd = 2 * vr * fc / c
    tgt_pos = physics_engine.target.pos
    tgt_vel = physics_engine.target.velocity
    rng_norm = max(1.0, float(np.linalg.norm(tgt_pos)))
    radial_vel = float(np.dot(tgt_pos, tgt_vel) / rng_norm)
    doppler_shift_hz = (2.0 * radial_vel * params.carrier_freq_hz) / C_LIGHT

    range_doppler = analytics_engine.compute_range_doppler_map(
        snr_db=params.snr_db,
        radial_velocity_ms=radial_vel,
        efficiency_pct=synth_rcc["efficiency_percent"],
        carrier_freq_hz=params.carrier_freq_hz
    )

    # RMS time & phase error for Fig. 5a operating point
    rms_time_err_s = float(np.sqrt(np.mean(rcc_echo_data["time_errors"] ** 2)))
    rms_time_err_norm = rms_time_err_s * params.signal_bandwidth_hz
    rms_phase_err_deg = float(np.degrees(np.sqrt(np.mean(rcc_echo_data["phase_errors"] ** 2))))
    loss_surface = analytics_engine.compute_loss_surface_fig5(rms_time_err_norm, rms_phase_err_deg)

    return {
        "stage1": stage1_data,
        "stage2": stage2_data,
        "stage3": stage3_data,
        "stage4": stage4_data,
        "stage5": stage5_data,
        "stage6": stage6_data,
        "analytics": {
            "iq_phasors": iq_phasors,
            "range_doppler": range_doppler,
            "loss_surface": loss_surface,
            "radial_velocity_ms": radial_vel,
            "doppler_shift_hz": doppler_shift_hz,
            "target_metadata": active_target_metadata
        }
    }


@app.get("/api/datasets/adsb/live")
def get_live_adsb_targets(limit: int = 20):
    """Fetches real-time airborne aircraft from OpenSky Network ADS-B feed."""
    return live_dataset_engine.fetch_live_adsb_targets(limit=limit)


@app.post("/api/datasets/adsb/lock")
def lock_live_adsb_target(target: Dict[str, Any]):
    """Locks the 3-node DARAR radar simulation onto a selected live ADS-B aircraft."""
    global active_target_metadata
    proj = live_dataset_engine.project_adsb_to_radar_local(target, stand_off_range_m=14000.0)
    physics_engine.target.pos = np.array(proj["local_pos"], dtype=float)
    physics_engine.target.velocity = np.array(proj["velocity_vec"], dtype=float)
    active_target_metadata = proj
    return {"status": "locked", "target": proj}


@app.get("/api/datasets/outfield")
def get_outfield_dataset():
    """Returns the 20-cycle Outfield UAV Detection Dataset from Section 5.2 (Figs. 15-19)."""
    return live_dataset_engine.get_outfield_uav_dataset()


@app.post("/api/datasets/outfield/cycle/{cycle_idx}")
def load_outfield_cycle(cycle_idx: int):
    """Loads a specific detection cycle (1-20) from the paper's Outfield UAV Experiment."""
    global active_target_metadata
    ds = live_dataset_engine.get_outfield_uav_dataset()
    idx = max(0, min(len(ds["cycles"]) - 1, cycle_idx - 1))
    cyc = ds["cycles"][idx]
    physics_engine.target.pos = np.array(cyc["local_pos"], dtype=float)
    physics_engine.target.velocity = np.array(cyc["velocity_vec"], dtype=float)
    active_target_metadata = {
        "source": "paper_outfield_uav",
        "callsign": f"UAV-OUTFIELD-C{cyc['cycle']}",
        "icao24": "JSEE-2026",
        "range_m": cyc["range_m"],
        "azimuth_deg": cyc["azimuth_deg"],
        "pitch_deg": cyc["pitch_deg"],
        "paper_efficiency_pct": cyc["paper_efficiency_pct"]
    }
    return {"status": "loaded", "cycle": cyc}


@app.get("/api/analytics/antenna_pattern")
def get_antenna_pattern():
    """Returns the 32-element phased array antenna pattern (Fig. 2a)."""
    return analytics_engine.compute_antenna_pattern_fig2()


@app.get("/api/benchmark/fig13")
def get_fig13_benchmark():
    """Returns Monte Carlo benchmark data replicating Figure 13 from the paper."""
    global benchmark_cache
    if benchmark_cache is None:
        engine = DARARBenchmarkEngine(trials_per_snr=15)
        benchmark_cache = engine.run_benchmark()
    return benchmark_cache


@app.websocket("/ws/simulation")
async def websocket_simulation_stream(websocket: WebSocket):
    """Real-time streaming WebSocket endpoint for continuous live 3D animation & telemetry."""
    await websocket.accept()
    try:
        while True:
            # Advance physical simulation
            state = physics_engine.step_servo_synchronization()
            delays = physics_engine.compute_all_propagation_delays()

            payload = {
                "cycle": state["cycle"],
                "central_command_deg": state["central_command_deg"],
                "servo_angles": state["servo_angles"],
                "world_pointing_angles": state["world_pointing_angles"],
                "sync_errors_deg": state["sync_errors_deg"],
                "max_error_deg": state["max_error_deg"],
                "is_synchronized": state["is_synchronized"],
                "drone_pos": state["drone_pos"],
                "delays_matrix_us": (delays * 1e6).tolist()
            }
            await websocket.send_json(payload)
            await asyncio.sleep(0.05)  # 20 Hz telemetry stream
    except WebSocketDisconnect:
        pass
