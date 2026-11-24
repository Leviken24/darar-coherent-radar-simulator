"""
Signal Processing Engine for Distributed Aperture Rotating Array Radar (DARAR).

Implements:
1. Orthogonal phase-encoded baseband signal generation (K=3, L=3, 9 channels)
2. Pulse compression & matched filtering with range-sidelobe suppression windowing
3. Physical channel propagation delays, carrier phases, and calibrated AWGN noise injection
4. Stage 1: Coarse time compensation based on prior geometric baseline
5. Stage 2: Regional Cross-Correlation (RCC) fine time & phase parameter estimation
6. Peak Picking (PP) and conventional Cross-Correlation (CC) for comparative evaluation
7. Coherent synthesis of all 9 channels
8. Exact computation of single-channel SNR, synthesized SNR, synthesis gain (dB), and synthesis efficiency eta (%)
   strictly based on Equations (9), (19), (20)-(23) from the paper.
"""

import math
import numpy as np
from typing import Dict, List, Tuple, Any, Optional

C_LIGHT = 299792458.0


class DARARSignalEngine:
    def __init__(self,
                 k_units: int = 3,
                 l_units: int = 3,
                 carrier_freq_hz: float = 5.5e9,  # C-band (5.5 GHz)
                 bandwidth_hz: float = 5.0e6,     # 5 MHz bandwidth
                 sampling_rate_hz: float = 20.0e6,# 20 MSa/s sampling rate
                 snr_db: float = 10.0,            # Default SNR = 10 dB
                 rcc_window_points: int = 5,      # Search range: 3, 5, or 7 points
                 num_samples: int = 128):
        self.K = k_units
        self.L = l_units
        self.fc = carrier_freq_hz
        self.B = bandwidth_hz
        self.fs = sampling_rate_hz
        self.Ts = 1.0 / self.fs  # 50 ns per sample
        self.snr_db = snr_db
        self.rcc_window = rcc_window_points
        self.N_samples = num_samples

        # Baseline reference channel: Radar 1 self-transmitting / self-receiving
        self.ref_k = 0
        self.ref_l = 0

    def synthesize_channel_echoes(self, delays: np.ndarray, snr_db: Optional[float] = None,
                                  additional_phase_errors: Optional[np.ndarray] = None) -> Dict[str, Any]:
        """
        Synthesizes the 9-channel matched-filter outputs s_lk^m(t) (Section 2.1, Eq. 6)
        driven by physical propagation delays, carrier phase shifts, and calibrated Gaussian noise.
        """
        if snr_db is None:
            snr_db = self.snr_db

        K, L = self.K, self.L
        time_axis = np.arange(self.N_samples) * self.Ts  # in seconds
        ref_delay = delays[self.ref_l, self.ref_k]

        # Target signal amplitude A
        A_target = 20.0
        snr_linear = 10.0 ** (snr_db / 10.0)
        sigma_w = math.sqrt((A_target ** 2) / (2.0 * snr_linear))

        raw_signals = np.zeros((L, K, self.N_samples), dtype=complex)
        true_delays_relative = np.zeros((L, K), dtype=float)
        true_phases = np.zeros((L, K), dtype=float)

        center_sample = 30  # Nominal target range bin in range gate window

        for l in range(L):
            for k in range(K):
                tau_lk = delays[l, k]
                rel_delay = tau_lk - ref_delay
                true_delays_relative[l, k] = rel_delay

                # Carrier phase shift: phi_lk = 2 * pi * fc * tau_lk
                base_phase = (2.0 * math.pi * self.fc * tau_lk) % (2.0 * math.pi)
                if additional_phase_errors is not None:
                    base_phase += additional_phase_errors[l, k]
                true_phases[l, k] = base_phase

                # Discrete sample offset for target peak
                sample_offset = center_sample + (rel_delay / self.Ts)

                # Generate matched-filter compressed pulse with tapered window to suppress range sidelobes
                for n in range(self.N_samples):
                    t_diff = (n - sample_offset) * self.Ts
                    sample_dist = abs(n - sample_offset)
                    if sample_dist <= 12.0:
                        taper = 0.5 * (1.0 + math.cos(math.pi * sample_dist / 12.0))
                        envelope = np.sinc(self.B * t_diff) * taper
                        raw_signals[l, k, n] = A_target * envelope * np.exp(-1j * base_phase)
                    else:
                        raw_signals[l, k, n] = 0.0

                # Add complex zero-mean white Gaussian channel noise
                noise = np.random.normal(0, sigma_w, self.N_samples) + 1j * np.random.normal(0, sigma_w, self.N_samples)
                raw_signals[l, k, :] += noise

        return {
            "signals": raw_signals,
            "time_axis": time_axis,
            "center_sample": center_sample,
            "true_delays": true_delays_relative,
            "true_phases": true_phases,
            "sigma_w": sigma_w,
            "A_target": A_target,
            "snr_db": snr_db
        }

    def apply_coarse_time_compensation(self, raw_data: Dict[str, Any],
                                        coarse_delays: np.ndarray) -> Dict[str, Any]:
        """
        Performs first-level coarse time compensation based on prior INS baseline information (Section 3.2).
        Shifts each channel by -Delta t_lk^coarse along the time axis.
        """
        signals = raw_data["signals"].copy()
        L, K, N = signals.shape
        compensated_signals = np.zeros_like(signals)
        applied_shifts_samples = np.zeros((L, K), dtype=int)

        for l in range(L):
            for k in range(K):
                tau_coarse = coarse_delays[l, k]
                shift_samples = int(round(tau_coarse / self.Ts))
                applied_shifts_samples[l, k] = shift_samples

                if shift_samples != 0:
                    compensated_signals[l, k, :] = np.roll(signals[l, k, :], -shift_samples)
                    if shift_samples > 0:
                        compensated_signals[l, k, -shift_samples:] = 0
                    elif shift_samples < 0:
                        compensated_signals[l, k, :-shift_samples] = 0
                else:
                    compensated_signals[l, k, :] = signals[l, k, :]

        return {
            "signals": compensated_signals,
            "applied_shifts_samples": applied_shifts_samples,
            "coarse_delays": coarse_delays,
            "raw_data": raw_data
        }

    def estimate_and_compensate_rcc(self, coarse_data: Dict[str, Any],
                                    search_window_points: Optional[int] = None) -> Dict[str, Any]:
        """
        Performs second-level fine time and phase compensation using
        Regional Cross-Correlation (RCC) (Section 3.3, Eq. 16-18).
        Reference signal: s_11^m(t).
        Peak search restricted to [-search_window_points, +search_window_points] around nominal center.
        """
        if search_window_points is None:
            search_window_points = self.rcc_window

        signals = coarse_data["signals"]
        L, K, N = signals.shape
        ref_signal = signals[self.ref_l, self.ref_k, :]

        fine_compensated_signals = np.zeros_like(signals)
        estimated_time_shifts = np.zeros((L, K), dtype=float)
        estimated_phases = np.zeros((L, K), dtype=float)
        time_errors = np.zeros((L, K), dtype=float)
        phase_errors = np.zeros((L, K), dtype=float)
        correlation_curves = {}

        w = search_window_points
        true_delays = coarse_data.get("raw_data", {}).get("true_delays", np.zeros((L, K)))
        true_phases = coarse_data.get("raw_data", {}).get("true_phases", np.zeros((L, K)))
        ref_phase = true_phases[self.ref_l, self.ref_k]

        for l in range(L):
            for k in range(K):
                sig = signals[l, k, :]
                if l == self.ref_l and k == self.ref_k:
                    fine_compensated_signals[l, k, :] = sig
                    estimated_time_shifts[l, k] = 0.0
                    estimated_phases[l, k] = 0.0
                    time_errors[l, k] = 0.0
                    phase_errors[l, k] = 0.0

                    corr = np.correlate(sig, ref_signal, mode='full')
                    lags = np.arange(-N + 1, N)
                    correlation_curves[f"S{l+1}{k+1}"] = {
                        "lags": lags.tolist(),
                        "corr_abs": np.abs(corr).tolist(),
                        "peak_lag": 0,
                        "search_window": [-w, w],
                        "peak_phase_rad": 0.0
                    }
                    continue

                # Cross-correlation with reference: x_lk(tau) = int s_lk(t) * s_11^*(t+tau) dt
                corr = np.correlate(sig, ref_signal, mode='full')
                lags = np.arange(-N + 1, N)
                zero_lag_idx = N - 1

                # Restrict search region to [-w, +w] sampling points around zero lag
                min_idx = max(0, zero_lag_idx - w)
                max_idx = min(len(corr), zero_lag_idx + w + 1)

                regional_corr = corr[min_idx:max_idx]
                regional_lags = lags[min_idx:max_idx]

                peak_local_idx = int(np.argmax(np.abs(regional_corr)))
                peak_lag = int(regional_lags[peak_local_idx])
                peak_val = regional_corr[peak_local_idx]

                # Estimated residual time delay and phase
                est_tau = peak_lag * self.Ts
                est_phase = float(np.angle(peak_val))

                estimated_time_shifts[l, k] = est_tau
                estimated_phases[l, k] = est_phase

                # Estimation errors (Equation 9)
                true_rel_tau = true_delays[l, k] - (coarse_data["applied_shifts_samples"][l, k] * self.Ts)
                time_errors[l, k] = est_tau - true_rel_tau
                true_rel_phase = (true_phases[l, k] - ref_phase + np.pi) % (2.0 * np.pi) - np.pi
                phase_errors[l, k] = (est_phase + true_rel_phase + np.pi) % (2.0 * np.pi) - np.pi

                # Apply fine shift
                shifted_sig = np.roll(sig, -peak_lag)
                if peak_lag > 0:
                    shifted_sig[-peak_lag:] = 0
                elif peak_lag < 0:
                    shifted_sig[:-peak_lag] = 0

                # Phase compensation
                fine_compensated_signals[l, k, :] = shifted_sig * np.exp(-1j * est_phase)

                correlation_curves[f"S{l+1}{k+1}"] = {
                    "lags": lags.tolist(),
                    "corr_abs": np.abs(corr).tolist(),
                    "peak_lag": peak_lag,
                    "search_window": [-w, w],
                    "peak_phase_rad": est_phase
                }

        return {
            "signals": fine_compensated_signals,
            "estimated_time_shifts": estimated_time_shifts,
            "estimated_phases": estimated_phases,
            "time_errors": time_errors,
            "phase_errors": phase_errors,
            "correlation_curves": correlation_curves,
            "search_window": w,
            "raw_data": coarse_data.get("raw_data")
        }

    def estimate_and_compensate_conventional_cc(self, coarse_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Conventional Cross-Correlation (CC) method (searches across entire full range gate).
        Used for comparison against RCC to demonstrate noise-induced false peak failure at low SNR.
        """
        signals = coarse_data["signals"]
        L, K, N = signals.shape
        ref_signal = signals[self.ref_l, self.ref_k, :]

        comp_signals = np.zeros_like(signals)
        estimated_time_shifts = np.zeros((L, K), dtype=float)
        estimated_phases = np.zeros((L, K), dtype=float)
        time_errors = np.zeros((L, K), dtype=float)
        phase_errors = np.zeros((L, K), dtype=float)

        true_delays = coarse_data.get("raw_data", {}).get("true_delays", np.zeros((L, K)))
        true_phases = coarse_data.get("raw_data", {}).get("true_phases", np.zeros((L, K)))
        ref_phase = true_phases[self.ref_l, self.ref_k]

        for l in range(L):
            for k in range(K):
                sig = signals[l, k, :]
                if l == self.ref_l and k == self.ref_k:
                    comp_signals[l, k, :] = sig
                    continue

                corr = np.correlate(sig, ref_signal, mode='full')
                lags = np.arange(-N + 1, N)

                # Full unrestricted search over entire correlation range
                peak_idx = int(np.argmax(np.abs(corr)))
                peak_lag = int(lags[peak_idx])
                peak_val = corr[peak_idx]

                est_tau = peak_lag * self.Ts
                est_phase = float(np.angle(peak_val))

                estimated_time_shifts[l, k] = est_tau
                estimated_phases[l, k] = est_phase

                true_rel_tau = true_delays[l, k] - (coarse_data["applied_shifts_samples"][l, k] * self.Ts)
                time_errors[l, k] = est_tau - true_rel_tau
                true_rel_phase = (true_phases[l, k] - ref_phase + np.pi) % (2.0 * np.pi) - np.pi
                phase_errors[l, k] = (est_phase + true_rel_phase + np.pi) % (2.0 * np.pi) - np.pi

                shifted = np.roll(sig, -peak_lag)
                if peak_lag > 0:
                    shifted[-peak_lag:] = 0
                elif peak_lag < 0:
                    shifted[:-peak_lag] = 0
                comp_signals[l, k, :] = shifted * np.exp(-1j * est_phase)

        return {
            "signals": comp_signals,
            "estimated_time_shifts": estimated_time_shifts,
            "estimated_phases": estimated_phases,
            "time_errors": time_errors,
            "phase_errors": phase_errors,
            "raw_data": coarse_data.get("raw_data")
        }

    def estimate_and_compensate_peak_picking(self, coarse_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Peak Picking (PP) method (finds amplitude peak of each channel independently).
        Used for comparison against RCC.
        """
        signals = coarse_data["signals"]
        L, K, N = signals.shape
        ref_signal = signals[self.ref_l, self.ref_k, :]
        ref_peak_idx = int(np.argmax(np.abs(ref_signal)))
        ref_phase_meas = np.angle(ref_signal[ref_peak_idx])

        comp_signals = np.zeros_like(signals)
        estimated_time_shifts = np.zeros((L, K), dtype=float)
        estimated_phases = np.zeros((L, K), dtype=float)
        time_errors = np.zeros((L, K), dtype=float)
        phase_errors = np.zeros((L, K), dtype=float)

        true_delays = coarse_data.get("raw_data", {}).get("true_delays", np.zeros((L, K)))
        true_phases = coarse_data.get("raw_data", {}).get("true_phases", np.zeros((L, K)))
        ref_true_phase = true_phases[self.ref_l, self.ref_k]

        for l in range(L):
            for k in range(K):
                sig = signals[l, k, :]
                if l == self.ref_l and k == self.ref_k:
                    comp_signals[l, k, :] = sig
                    continue

                sig_peak_idx = int(np.argmax(np.abs(sig)))
                sig_phase_meas = np.angle(sig[sig_peak_idx])

                lag = sig_peak_idx - ref_peak_idx
                phase_diff = float(sig_phase_meas - ref_phase_meas)

                estimated_time_shifts[l, k] = lag * self.Ts
                estimated_phases[l, k] = phase_diff

                true_rel_tau = true_delays[l, k] - (coarse_data["applied_shifts_samples"][l, k] * self.Ts)
                time_errors[l, k] = (lag * self.Ts) - true_rel_tau
                true_rel_phase = (true_phases[l, k] - ref_true_phase + np.pi) % (2.0 * np.pi) - np.pi
                phase_errors[l, k] = (phase_diff + true_rel_phase + np.pi) % (2.0 * np.pi) - np.pi

                shifted = np.roll(sig, -lag)
                if lag > 0:
                    shifted[-lag:] = 0
                elif lag < 0:
                    shifted[:-lag] = 0
                comp_signals[l, k, :] = shifted * np.exp(-1j * phase_diff)

        return {
            "signals": comp_signals,
            "estimated_time_shifts": estimated_time_shifts,
            "estimated_phases": estimated_phases,
            "time_errors": time_errors,
            "phase_errors": phase_errors,
            "raw_data": coarse_data.get("raw_data")
        }

    def perform_coherent_synthesis(self, proc_data: Any,
                                   target_bin: int = 30) -> Dict[str, Any]:
        """
        Synthesizes the KL signals (Equation 9 and 19) and calculates exact performance metrics
        strictly derived from Equations (19)-(23) in the paper:
        1. Single-channel average SNR (SNRs)
        2. Synthesized signal waveform y(t)
        3. Synthesized SNR (SNRc)
        4. Actual SNR gain in dB
        5. Ideal theoretical gain (KL = 9 times -> 9.54 dB)
        6. Coherent synthesis efficiency eta (%) based on Equation (23) & Section 4
        """
        if isinstance(proc_data, dict):
            channel_signals = proc_data["signals"]
            time_errors = proc_data.get("time_errors")
            phase_errors = proc_data.get("phase_errors")
            raw_data = proc_data.get("raw_data")
        else:
            channel_signals = proc_data
            time_errors = None
            phase_errors = None
            raw_data = None

        L, K, N = channel_signals.shape
        KL = K * L

        # Synthesized waveform y(t)
        synthesized_signal = np.sum(channel_signals, axis=(0, 1))

        # Noise-only region away from target peak
        noise_region = np.concatenate([np.arange(0, 12), np.arange(55, N)])

        # Single channel metrics
        raw_data = proc_data.get("raw_data")
        if raw_data:
            snr_s_db = float(raw_data.get("snr_db", self.snr_db))
            snr_s_linear = 10.0 ** (snr_s_db / 10.0)
        else:
            channel_peak_powers = [np.max(np.abs(channel_signals[l, k])) ** 2 for l in range(L) for k in range(K)]
            channel_noise_powers = [np.mean(np.abs(channel_signals[l, k, noise_region])) ** 2 for l in range(L) for k in range(K)]
            snr_s_linear = np.mean(channel_peak_powers) / max(1e-12, np.mean(channel_noise_powers))
            snr_s_db = 10.0 * math.log10(max(1e-6, snr_s_linear))

        # Target synthesis vector using Equation (19):
        # y(t_target) = sum_{l,k} A_lk(Delta tau'_lk) * exp(j * Delta phi'_lk)
        time_errors = proc_data.get("time_errors")
        phase_errors = proc_data.get("phase_errors")

        if time_errors is not None and phase_errors is not None:
            # Analytical evaluation of coherent target amplitude from estimation errors
            s_components = []
            for l in range(L):
                for k in range(K):
                    d_tau = time_errors[l, k]
                    d_phi = phase_errors[l, k]
                    # Sinc envelope degradation due to timing error
                    amp_factor = np.sinc(self.B * d_tau)
                    # Complex phasor contribution
                    s_components.append(amp_factor * np.exp(1j * d_phi))
            # Normalized synthesis vector (ideal sum = KL = 9)
            s_target_sum = np.sum(s_components)
            # Synthesis efficiency eta = |sum / KL|^2 * 100% (Equation 23)
            efficiency_percent = float(min(100.0, max(0.0, (np.abs(s_target_sum) / float(KL)) ** 2 * 100.0)))
        else:
            # Fallback to direct waveform measurement
            target_window = slice(max(0, target_bin - 2), min(N, target_bin + 3))
            synth_peak_power = np.max(np.abs(synthesized_signal[target_window])) ** 2
            synth_noise_power = np.mean(np.abs(synthesized_signal[noise_region])) ** 2
            measured_gain_lin = (synth_peak_power / max(1e-12, synth_noise_power)) / snr_s_linear
            efficiency_percent = float(min(100.0, max(0.0, (measured_gain_lin / float(KL)) * 100.0)))

        # Actual gain and synthesized SNR
        ideal_gain_linear = float(KL)
        ideal_gain_db = 10.0 * math.log10(ideal_gain_linear)  # 9.54 dB

        actual_gain_linear = (efficiency_percent / 100.0) * ideal_gain_linear
        actual_gain_db = 10.0 * math.log10(max(1e-6, actual_gain_linear)) if actual_gain_linear > 0 else -30.0
        snr_c_db = snr_s_db + actual_gain_db

        return {
            "synthesized_signal_abs": np.abs(synthesized_signal).tolist(),
            "synthesized_signal_real": np.real(synthesized_signal).tolist(),
            "synthesized_signal_imag": np.imag(synthesized_signal).tolist(),
            "snr_s_db": float(snr_s_db),
            "snr_c_db": float(snr_c_db),
            "actual_gain_db": float(actual_gain_db),
            "ideal_gain_db": float(ideal_gain_db),
            "efficiency_percent": float(efficiency_percent),
            "target_bin": target_bin,
            "kl_channels": KL,
            "time_errors_ns": (time_errors * 1e9).tolist() if time_errors is not None else [],
            "phase_errors_deg": np.degrees(phase_errors).tolist() if phase_errors is not None else []
        }
