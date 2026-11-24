"""
Monte Carlo Benchmark Engine for Distributed Coherent Detection (DARAR).

Reproduces Figure 13 from the research paper:
"Performance comparison of different processing methods"
Compares:
1. Peak Picking (PP)
2. Conventional Cross-Correlation (CC)
3. Regional Cross-Correlation (RCC, 7 points)
4. Regional Cross-Correlation (RCC, 5 points)
5. Regional Cross-Correlation (RCC, 3 points)
6. Theoretical upper bound
over SNRs from 5 dB to 25 dB.
"""

import numpy as np
from typing import Dict, List, Any
from core.signal_engine import DARARSignalEngine


class DARARBenchmarkEngine:
    def __init__(self, trials_per_snr: int = 25):
        self.trials = trials_per_snr
        self.snr_values = [5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25]

    def run_benchmark(self, progress_callback=None) -> Dict[str, Any]:
        """
        Runs Monte Carlo simulation comparing PP, CC, and RCC (3, 5, 7) across SNR range.
        Returns coherent synthesis efficiency (%) curves.
        """
        results = {
            "snr_values": self.snr_values,
            "PP": [],
            "CC": [],
            "RCC_7": [],
            "RCC_5": [],
            "RCC_3": [],
            "Theoretical": []
        }

        # Simulated baseline propagation delay matrix (small residual delay after coarse alignment)
        base_delays = np.zeros((3, 3))
        # Add slight sub-sample / residual offsets simulating INS error
        res_offsets_samples = np.array([
            [0.0, 1.2, -0.8],
            [1.0, 0.4, 1.6],
            [-1.1, -0.5, 0.2]
        ])

        for idx, snr in enumerate(self.snr_values):
            eff_pp_list = []
            eff_cc_list = []
            eff_rcc7_list = []
            eff_rcc5_list = []
            eff_rcc3_list = []

            for trial in range(self.trials):
                # Random phase errors for this trial
                phase_errors = np.random.uniform(-np.pi, np.pi, (3, 3))
                phase_errors[0, 0] = 0.0

                engine = DARARSignalEngine(snr_db=float(snr), num_samples=128)
                delays = res_offsets_samples * engine.Ts

                # Generate noisy echo signals
                raw_data = engine.synthesize_channel_echoes(delays, snr_db=float(snr), additional_phase_errors=phase_errors)

                # Coarse compensation (leaves small residual)
                coarse_data = engine.apply_coarse_time_compensation(raw_data, np.zeros_like(delays))

                # 1. Peak Picking (PP)
                pp_data = engine.estimate_and_compensate_peak_picking(coarse_data)
                synth_pp = engine.perform_coherent_synthesis(pp_data)
                eff_pp_list.append(synth_pp["efficiency_percent"])

                # 2. Conventional Cross Correlation (CC)
                cc_data = engine.estimate_and_compensate_conventional_cc(coarse_data)
                synth_cc = engine.perform_coherent_synthesis(cc_data)
                eff_cc_list.append(synth_cc["efficiency_percent"])

                # 3. RCC 7 points
                rcc7_data = engine.estimate_and_compensate_rcc(coarse_data, search_window_points=7)
                synth_rcc7 = engine.perform_coherent_synthesis(rcc7_data)
                eff_rcc7_list.append(synth_rcc7["efficiency_percent"])

                # 4. RCC 5 points
                rcc5_data = engine.estimate_and_compensate_rcc(coarse_data, search_window_points=5)
                synth_rcc5 = engine.perform_coherent_synthesis(rcc5_data)
                eff_rcc5_list.append(synth_rcc5["efficiency_percent"])

                # 5. RCC 3 points
                rcc3_data = engine.estimate_and_compensate_rcc(coarse_data, search_window_points=3)
                synth_rcc3 = engine.perform_coherent_synthesis(rcc3_data)
                eff_rcc3_list.append(synth_rcc3["efficiency_percent"])

            results["PP"].append(float(np.mean(eff_pp_list)))
            results["CC"].append(float(np.mean(eff_cc_list)))
            results["RCC_7"].append(float(np.mean(eff_rcc7_list)))
            results["RCC_5"].append(float(np.mean(eff_rcc5_list)))
            results["RCC_3"].append(float(np.mean(eff_rcc3_list)))
            # Theoretical upper bound approaches ~100% asymptotically
            theoretical_eff = 100.0 / (1.0 + np.exp(-(snr - 6.0) / 1.8))
            results["Theoretical"].append(float(min(100.0, theoretical_eff)))

            if progress_callback:
                progress_callback(idx + 1, len(self.snr_values))

        return results
