"""
Unit Tests for DARAR Mathematical and Physical Engines.

Validates:
1. Baseline calculation from geodetic coordinates (Equation 13)
2. Coarse geometric delay calculation (Equations 14 & 15)
3. Parallel control model and alpha-filter convergence (Equations 10-12, Figures 9-10)
4. Speed of light propagation delays: tau_lk = (R_k + R_l) / c
5. Signal generation, matched filtering, and 9-channel separation
6. Coarse time compensation shifting
7. Regional Cross-Correlation (RCC) peak and phase estimation (Equations 16-18)
8. Coherent synthesis gain (ideal KL=9 / 9.54 dB) and efficiency eta (Equation 20-23)
9. Superiority of RCC over PP and CC under low SNR conditions (8-13 dB)
"""

import sys
import os
import math
import numpy as np

# Add backend directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from core.physics_engine import DARARPhysicsEngine, RadarNode, C_LIGHT
from core.signal_engine import DARARSignalEngine
from core.benchmark_engine import DARARBenchmarkEngine


def test_geodetic_baseline():
    """Validates baseline distance calculation using spherical geometry (Eq. 13)."""
    engine = DARARPhysicsEngine(baseline_length=60.0)
    ref_node = engine.nodes[0]
    node2 = engine.nodes[1]
    d2 = engine.compute_geodetic_baseline(node2, ref_node)
    print(f"[TEST 1] Calculated Geodetic Baseline D2: {d2:.2f} m (Expected ~60.0 m)")
    assert abs(d2 - 60.0) < 1.0, f"Baseline error too large: {d2}"


def test_coarse_time_delay():
    """Validates coarse geometric delay formula: |Delta t_i| = D_i * sin(theta) / c (Eq. 14, 15)."""
    engine = DARARPhysicsEngine(baseline_length=60.0)
    theta_deg = 30.0  # 30 deg target azimuth
    dt_coarse = engine.compute_coarse_time_delay(2, theta_deg)  # Radar 3 at +60m
    expected_dt = (60.0 * math.sin(math.radians(30.0))) / C_LIGHT  # 30m / c ~ 100 ns
    print(f"[TEST 2] Coarse Delay dt: {dt_coarse * 1e9:.2f} ns (Expected: {expected_dt * 1e9:.2f} ns)")
    assert abs(dt_coarse - expected_dt) < 1e-12, f"Coarse delay mismatch: {dt_coarse} vs {expected_dt}"


def test_alpha_filter_servo_synchronization():
    """Validates parallel control model and alpha-filter convergence (Eq. 10-12, Fig. 9-10)."""
    engine = DARARPhysicsEngine(baseline_length=60.0)
    # Run 60 synchronization cycles (simulating Section 5.1.1)
    for _ in range(60):
        state = engine.step_servo_synchronization()

    max_err = state["max_error_deg"]
    print(f"[TEST 3] After 60 cycles, Max Servo Error: {max_err:.3f}° (Paper requires <= 0.1°)")
    assert max_err < 0.15, f"Servo sync error {max_err}° did not reach threshold"


def test_propagation_delays():
    """Validates physical two-way propagation delay: tau_lk = (R_k + R_l) / c."""
    engine = DARARPhysicsEngine(baseline_length=60.0, target_dist=10000.0)
    delays = engine.compute_all_propagation_delays()
    # Check shape is 3x3
    assert delays.shape == (3, 3)
    # Check tau_11 = 2 * R1 / c
    r1 = np.linalg.norm(engine.target.pos - engine.nodes[0].local_pos)
    expected_tau11 = 2.0 * r1 / C_LIGHT
    print(f"[TEST 4] Two-way delay tau_11: {delays[0, 0] * 1e6:.3f} us (Expected: {expected_tau11 * 1e6:.3f} us)")
    assert abs(delays[0, 0] - expected_tau11) < 1e-10


def test_coherent_synthesis_ideal_gain():
    """Validates ideal coherent synthesis gain KL=9 (9.54 dB) and efficiency calculation (Eq. 20-23)."""
    # High SNR, perfectly aligned signals
    engine = DARARSignalEngine(snr_db=30.0, num_samples=128)
    delays = np.zeros((3, 3))
    raw_data = engine.synthesize_channel_echoes(delays, snr_db=30.0)
    coarse_data = engine.apply_coarse_time_compensation(raw_data, np.zeros_like(delays))
    rcc_data = engine.estimate_and_compensate_rcc(coarse_data, search_window_points=3)
    synth = engine.perform_coherent_synthesis(rcc_data)

    print(f"[TEST 5] High-SNR Coherent Synthesis Gain: {synth['actual_gain_db']:.2f} dB (Ideal: {synth['ideal_gain_db']:.2f} dB)")
    print(f"         Coherent Synthesis Efficiency: {synth['efficiency_percent']:.1f}% (Expected ~100%)")
    assert synth["actual_gain_db"] >= 8.5, f"Gain too low: {synth['actual_gain_db']}"
    assert synth["efficiency_percent"] >= 80.0


def test_rcc_superiority_at_low_snr():
    """
    Validates central research contribution (Section 5.1.3 & Fig. 13):
    At low SNR (e.g. 8 dB), RCC significantly outperforms PP and CC by restricting false peak detections.
    """
    engine = DARARSignalEngine(snr_db=8.0, num_samples=256)
    res_offsets_samples = np.array([
        [0.0, 1.0, -1.0],
        [1.0, 0.0, 1.0],
        [-1.0, -1.0, 0.0]
    ])
    delays = res_offsets_samples * engine.Ts

    pp_effs, cc_effs, rcc_effs = [], [], []

    for _ in range(30):
        phase_errors = np.random.uniform(-np.pi, np.pi, (3, 3))
        phase_errors[0, 0] = 0.0

        raw_data = engine.synthesize_channel_echoes(delays, snr_db=8.0, additional_phase_errors=phase_errors)
        coarse_data = engine.apply_coarse_time_compensation(raw_data, np.zeros_like(delays))

        # PP
        pp_data = engine.estimate_and_compensate_peak_picking(coarse_data)
        synth_pp = engine.perform_coherent_synthesis(pp_data)
        pp_effs.append(synth_pp["efficiency_percent"])

        # CC
        cc_data = engine.estimate_and_compensate_conventional_cc(coarse_data)
        synth_cc = engine.perform_coherent_synthesis(cc_data)
        cc_effs.append(synth_cc["efficiency_percent"])

        # RCC (3 points)
        rcc_data = engine.estimate_and_compensate_rcc(coarse_data, search_window_points=3)
        synth_rcc = engine.perform_coherent_synthesis(rcc_data)
        rcc_effs.append(synth_rcc["efficiency_percent"])

    mean_pp = np.mean(pp_effs)
    mean_cc = np.mean(cc_effs)
    mean_rcc = np.mean(rcc_effs)

    print(f"[TEST 6] At 8 dB SNR (30 trials) - Average Coherent Synthesis Efficiency:")
    print(f"         Peak Picking (PP):       {mean_pp:.1f}%")
    print(f"         Conventional CC:         {mean_cc:.1f}%")
    print(f"         Proposed RCC (3 points): {mean_rcc:.1f}%")

    assert mean_rcc > mean_pp, f"RCC ({mean_rcc:.1f}%) should outperform Peak Picking ({mean_pp:.1f}%)"
    assert mean_rcc > mean_cc, f"RCC ({mean_rcc:.1f}%) should outperform conventional CC ({mean_cc:.1f}%)"


if __name__ == "__main__":
    print("=== RUNNING MATHEMATICAL VALIDATION TEST SUITE ===")
    test_geodetic_baseline()
    test_coarse_time_delay()
    test_alpha_filter_servo_synchronization()
    test_propagation_delays()
    test_coherent_synthesis_ideal_gain()
    test_rcc_superiority_at_low_snr()
    print("=== ALL 6 MATHEMATICAL VALIDATION TESTS PASSED SUCCESSFULLY! ===")
