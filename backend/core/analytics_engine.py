"""
Advanced Aerospace & DSP Analytics Engine for DARAR Simulator.

Computes:
1. 9-Channel Complex I/Q Phasor Vectors (Before vs. After RCC alignment)
2. 2D Range-Doppler Detection Matrix (Single Channel vs. 9-Channel Coherent Synthesis)
3. 3D Joint Time-Phase Synthesis Loss Surface (Reproducing Figure 5a from Paper)
4. 32-Element Linear Array Antenna Pattern (Reproducing Figure 2a from Paper)
"""

import math
import numpy as np
from typing import Dict, List, Any

C_LIGHT = 299792458.0


class DARARAnalyticsEngine:
    @staticmethod
    def compute_iq_phasors(raw_data: Dict[str, Any], rcc_data: Dict[str, Any],
                           target_bin: int = 30) -> Dict[str, Any]:
        """
        Extracts complex I/Q phasors at the target range bin for all 9 channels
        both before compensation and after RCC fine compensation.
        """
        raw_sigs = raw_data["signals"]
        rcc_sigs = rcc_data["signals"]
        L, K, N = raw_sigs.shape

        before_phasors = []
        after_phasors = []

        for l in range(L):
            for k in range(K):
                ch_name = f"S{l+1}{k+1}"
                # Before compensation: sample at each channel's peak or target bin
                raw_val = raw_sigs[l, k, target_bin]
                # Use peak sample in raw signal to show phase dispersion
                raw_peak_idx = int(np.argmax(np.abs(raw_sigs[l, k, :])))
                raw_peak = raw_sigs[l, k, raw_peak_idx]

                before_phasors.append({
                    "channel": ch_name,
                    "i": float(np.real(raw_peak)),
                    "q": float(np.imag(raw_peak)),
                    "mag": float(np.abs(raw_peak)),
                    "phase_deg": float(math.degrees(np.angle(raw_peak)))
                })

                # After RCC compensation: sample at aligned target bin
                rcc_val = rcc_sigs[l, k, target_bin]
                after_phasors.append({
                    "channel": ch_name,
                    "i": float(np.real(rcc_val)),
                    "q": float(np.imag(rcc_val)),
                    "mag": float(np.abs(rcc_val)),
                    "phase_deg": float(math.degrees(np.angle(rcc_val)))
                })

        # Resultant vector magnitudes
        sum_before = complex(sum(p["i"] for p in before_phasors), sum(p["q"] for p in before_phasors))
        sum_after = complex(sum(p["i"] for p in after_phasors), sum(p["q"] for p in after_phasors))

        return {
            "before": before_phasors,
            "after": after_phasors,
            "resultant_before": {
                "i": float(sum_before.real),
                "q": float(sum_before.imag),
                "mag": float(abs(sum_before))
            },
            "resultant_after": {
                "i": float(sum_after.real),
                "q": float(sum_after.imag),
                "mag": float(abs(sum_after))
            }
        }

    @staticmethod
    def compute_range_doppler_map(snr_db: float, radial_velocity_ms: float,
                                  efficiency_pct: float, carrier_freq_hz: float = 5.5e9) -> Dict[str, Any]:
        """
        Generates a 2D Range-Doppler Detection Matrix (25 Doppler bins x 64 Range bins)
        comparing Single Radar detection vs. 9-Channel RCC Coherent Synthesis.
        """
        n_doppler = 25
        n_range = 64
        vel_axis = np.linspace(-250.0, 250.0, n_doppler)
        range_axis = np.arange(n_range)

        target_r_bin = 30
        # Clamp radial velocity to display grid
        clamped_vr = max(-230.0, min(230.0, radial_velocity_ms))
        target_v_idx = int(np.argmin(np.abs(vel_axis - clamped_vr)))

        # Single channel SNR linear & noise floor
        snr_lin = 10.0 ** (snr_db / 10.0)
        amp_single = math.sqrt(snr_lin)

        # Build 2D Gaussian/Sinc target response in Range-Doppler
        rr, vv = np.meshgrid(range_axis, np.arange(n_doppler))
        target_profile = np.exp(-0.5 * (((rr - target_r_bin) / 1.4) ** 2 + ((vv - target_v_idx) / 1.1) ** 2))

        # Single channel map (normalized noise power = 1 -> 0 dB floor)
        noise_single = np.random.rayleigh(scale=0.7, size=(n_doppler, n_range))
        rd_single = 20.0 * np.log10(np.maximum(0.1, amp_single * target_profile + noise_single))

        # 9-Channel Coherent Synthesis map: signal amplitude scales by 9 * sqrt(eta/100), noise by sqrt(9)=3
        # Net SNR gain = 9 * (eta / 100)
        eff_frac = max(0.05, efficiency_pct / 100.0)
        amp_synth = amp_single * 3.0 * math.sqrt(eff_frac)
        noise_synth = np.random.rayleigh(scale=0.7, size=(n_doppler, n_range))
        rd_synth = 20.0 * np.log10(np.maximum(0.1, amp_synth * target_profile + noise_synth))

        return {
            "range_bins": range_axis.tolist(),
            "velocity_bins": vel_axis.tolist(),
            "single_channel_db": rd_single.tolist(),
            "coherent_synth_db": rd_synth.tolist(),
            "target_range_bin": target_r_bin,
            "target_velocity_ms": float(clamped_vr)
        }

    @staticmethod
    def compute_loss_surface_fig5(current_time_err_norm: float = 0.05,
                                  current_phase_err_deg: float = 8.0) -> Dict[str, Any]:
        """
        Computes the 3D Synthesis Gain Loss Surface (Fig. 5a from Paper):
        Loss(dB) as a function of normalized time error (-0.6 to +0.6 in 1/B)
        and phase error (-60° to +60°).
        """
        t_errs = np.linspace(-0.6, 0.6, 25)
        p_errs = np.linspace(-60.0, 60.0, 25)
        T, P = np.meshgrid(t_errs, p_errs)

        # Two-node synthesis gain loss formula:
        # |1 + sinc(t_err) * exp(j * p_err)|^2 / 4
        sinc_term = np.sinc(T)
        phase_rad = np.radians(P)
        mag_sq = (1.0 + sinc_term ** 2 + 2.0 * sinc_term * np.cos(phase_rad)) / 4.0
        loss_db = -10.0 * np.log10(np.maximum(1e-4, mag_sq))

        # Current operating point loss
        cur_sinc = float(np.sinc(current_time_err_norm))
        cur_cos = math.cos(math.radians(current_phase_err_deg))
        cur_mag_sq = (1.0 + cur_sinc ** 2 + 2.0 * cur_sinc * cur_cos) / 4.0
        cur_loss_db = -10.0 * math.log10(max(1e-4, cur_mag_sq))

        return {
            "time_error_axis": t_errs.tolist(),
            "phase_error_axis": p_errs.tolist(),
            "loss_surface_db": loss_db.tolist(),
            "current_point": {
                "time_err_norm": float(current_time_err_norm),
                "phase_err_deg": float(current_phase_err_deg),
                "loss_db": float(cur_loss_db)
            }
        }

    @staticmethod
    def compute_antenna_pattern_fig2() -> Dict[str, Any]:
        """
        Computes the 32-element linear array normalized antenna gain pattern (Fig. 2a).
        """
        N = 32
        angles = np.linspace(-4.0, 4.0, 161)
        gains_db = []
        for a in angles:
            # Normalized target angle mapping
            arg = 0.5 * math.pi * math.sin(math.radians(a * 1.8))
            if abs(arg) < 1e-9:
                g = 1.0
            else:
                g = abs(math.sin(N * arg) / (N * math.sin(arg)))
            gains_db.append(float(max(-70.0, 20.0 * math.log10(max(1e-4, g)))))

        return {
            "angles_deg": angles.tolist(),
            "gains_db": gains_db
        }
