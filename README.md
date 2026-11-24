# DARAR Coherent Radar Digital Twin & Simulation Environment
### Interactive 3D Engineering Simulator for Distributed Aperture Rotating Array Radar

Based on the research paper:  
**"Distributed coherent detection method for rotating array radar"**  
*Liu Zhongwei, Zhou Dongming, Xiang Rongrong, Gao Hongwei, and Chen Jie*  
*Journal of Systems Engineering and Electronics, Vol. 37, No. 5, October 2026, pp. 1490–1501.*  
*DOI: 10.23919/JSEE.2026.000023*

---

## 1. Project Overview

This project is an **interactive engineering simulation and digital twin**, designed to allow researchers, defense engineers, and students to **observe and experiment with the complete distributed coherent detection methodology in action**.

Rather than displaying static presentation slides or abstract text, this simulation lets you:
1. Watch **3 independent radar stations** rotating and adjusting their arrays using the **parallel control model and $\alpha$-filter**.
2. See **electromagnetic pulse wavefronts** propagate at physical light-speed ($t = d/c$) to illuminate a moving drone target (UAV) and reflect back to all receiving stations.
3. Observe all **9 received channels ($S_{11} \dots S_{33}$)** simultaneously on a high-speed multi-trace oscilloscope, displaying their physical time delays, carrier phase shifts, and channel noise.
4. Watch the central processor apply **First-Level Coarse Time Compensation** based on geodetic INS baseline coordinates ($D_i$) and beam pointing angle ($\theta$), sliding the waveforms into coarse alignment.
5. Experience the central breakthrough: **Regional Cross-Correlation (RCC)**. Watch the algorithm restrict its search window to $\pm n$ sampling points ($n = 3, 5, 7$), rejecting outer noise spikes that cause conventional cross-correlation and peak picking to fail under low SNR.
6. Witness the **Coherent Synthesis Chamber**: compare the destructive interference before compensation against the constructive summation after RCC, achieving **$+8.69\text{ dB}$ to $+9.54\text{ dB}$ SNR gain** and **$82.3\%$ coherent synthesis efficiency** (matching the outfield experimental results).
7. Run live **Monte Carlo benchmarks** dynamically replicating **Figure 13** of the research paper.

---

## 2. Architecture & Engine Separation

The project is structured into three strictly separated engines:

```
darar-coherent-radar-simulator/
│
├── backend/
│   ├── core/
│   │   ├── physics_engine.py       # Engine 1: Physical Kinematics & Geometry
│   │   ├── signal_engine.py        # Engine 2: Radar Signal Processing & RCC
│   │   └── benchmark_engine.py    # Monte Carlo Engine (Fig. 13 Validation)
│   ├── tests/
│   │   └── test_radar_engines.py   # Comprehensive Mathematical Validation Suite
│   └── main.py                     # FastAPI REST & WebSocket Streaming Server
│
├── frontend/                       # Engine 3: Interactive 3D Visualization
│   ├── index.html                  # Aerospace Mission Control Interface
│   ├── app.js                      # Three.js 3D Engine, 9-Ch Oscilloscope & Plotly
│   └── style.css                   # HUD Phosphor Styling & Theme
│
├── run_simulation.py               # One-click Launcher
└── README.md                       # Complete Engineering Reference
```

### Engine 1: Physical Simulation Engine (`core/physics_engine.py`)
- Models 3 distributed radar stations arranged along a baseline ($D = 60\text{ m}$ to $400\text{ m}$) with geodetic coordinates $(L_i, B_i, H_i)$.
- Implements the **Parallel Control Model** (Eq. 10–12):
  $$\Delta \theta_i = \theta_i^0 - \theta_1^0$$
  $$\theta_i^k = \theta^k + \Delta \theta_i$$
  $$\psi_i^k = \alpha \cdot \theta_i^k + (1 - \alpha) \cdot \psi_i^{k-1}$$
  Dynamically adapts $\alpha \in [0.85 \to 0.05]$ until rotation error is $< 0.1^\circ$ ($< 0.15\text{ dB}$ gain loss).
- Kinematics of cooperative UAV drone target moving across the radar field of view.
- Calculates exact physical 2-way propagation times:
  $$\tau_{lk} = \frac{R_k + R_l}{c}$$

### Engine 2: Signal Processing Engine (`core/signal_engine.py`)
- Emits $K=3$ orthogonal phase-encoded pulse signals ($s_k(t)$) in C-band ($f_c = 5.5\text{ GHz}$) with bandwidth $B = 5\text{ MHz}$ and pulse width $T_p = 100\ \mu\text{s}$ ($> 20\text{ dB}$ cross-correlation isolation).
- Generates 9-channel matched-filter outputs $s_{lk}^m(t)$ with calibrated AWGN channel noise.
- **First-Level Coarse Time Compensation** (Eq. 14–15):
  $$\Delta t_i = \text{sgn}(L_i - L_1) \cdot \frac{D_i \sin\theta}{c}$$
- **Second-Level Regional Cross-Correlation (RCC)** (Eq. 16–18):
  $$x_{lk}(\tau) = \int s_{lk}^m(t) \cdot s_{11}^{m*}(t + \tau) dt$$
  $$\hat{\Delta \tau}_{lk} = \arg\max_{|\tau| \le n T_s} |x_{lk}(\tau)|, \quad \hat{\Delta \phi}_{lk} = \angle x_{lk}(\hat{\Delta \tau}_{lk})$$
  Search window restricted to $\pm 3, \pm 5, \pm 7$ sampling points ($150\text{ ns}$ to $350\text{ ns}$).
- Comparative implementations of **Peak Picking (PP)** and **Conventional Cross-Correlation (CC)**.
- **Coherent Synthesis** (Eq. 19–23):
  $$y(t) = \sum_{l=1}^L \sum_{k=1}^K s_{lk}^{\text{comp}}(t)$$
  $$\Delta \text{SNR}_{s,\text{ideal}} = KL = 9 \implies 9.54\text{ dB}$$
  $$\eta = \frac{\Delta \text{SNR}_{s,\text{actual}}}{KL} \times 100\%$$

### Engine 3: 3D Visualization & Diagnostic Engine (`frontend/`)
- Interactive **Three.js WebGL 3D environment** with orbital camera controls, top-down tactical map, follow-drone view, and station close-ups.
- Animated 32-element phased array panels, RF radiation cones, and expanding/reflecting electromagnetic pulse wavefronts.
- Real-time **9-Channel Oscilloscope Deck** showing envelopes, phases, and arrival times.
- Interactive **RCC Diagnostic Plot** showing the full cross-correlation curve and highlighted search region.
- **Synthesis Comparison Chamber** comparing uncompensated destructive signal vs coherent constructive signal.
- Live **Figure 13 Monte Carlo graph** verifying SNR vs Efficiency against paper findings.

---

## 3. Two Operating Modes

### Mode A: Guided Research Walkthrough
Guides you through the 6 continuous stages of the research paper:
- **Stage 1:** Spatial Synchronization & Servo $\alpha$-Filter Convergence
- **Stage 2:** Signal Transmission & Propagation Delay ($t = d/c$)
- **Stage 3:** Signal Processing Lab & 9 Raw Echoes
- **Stage 4:** First-Level Coarse Time Compensation
- **Stage 5:** Regional Cross-Correlation (RCC) Fine Compensation
- **Stage 6:** Coherent Signal Synthesis & Verification

### Mode B: Experimental Digital Twin Laboratory
Gives you full parametric sandbox control:
- Radar Baseline: $20\text{ m}$ to $400\text{ m}$
- Target Distance: $5\text{ km}$ to $25\text{ km}$
- Target Velocity: $0$ to $100\text{ m/s}$
- Single Echo SNR: $5\text{ dB}$ to $25\text{ dB}$
- RCC Window: $\pm 3, \pm 5, \pm 7$ points
- INS Measurement Error: $0\text{ m}$ to $5\text{ m}$
- Phase Synchronization Jitter: $0^\circ$ to $45^\circ$
- One-click presets: *Outfield UAV (17km, 60m)*, *Low SNR Stress (8 dB)*, *Max Baseline (400m)*, *High-Gain Ideal*.

---

## 4. Running the Simulator

### Prerequisites
- Python 3.9+ with `numpy`, `scipy`, `fastapi`, and `uvicorn`.

### Quick Start
To launch the complete simulation environment:

```bash
cd "darar-coherent-radar-simulator"
python run_simulation.py
```

Then open your browser to:
```
http://127.0.0.1:8000
```

The simulation will load with the interactive 3D virtual environment, 9-channel oscilloscope deck, RCC peak inspector, and live synthesis chamber.

### Running the Mathematical Validation Suite
To execute the automated mathematical and physical validation test suite:

```bash
python backend/tests/test_radar_engines.py
```

**Results:**
- `[TEST 1]` Geodetic Baseline Calculation ($D_2 \approx 59.93\text{ m}$ vs $60.0\text{ m}$) — **PASSED**
- `[TEST 2]` Coarse Delay Calculation ($\Delta t = 100.07\text{ ns}$) — **PASSED**
- `[TEST 3]` $\alpha$-Filter Servo Convergence ($< 0.1^\circ$ error in 60 cycles) — **PASSED**
- `[TEST 4]` Two-way Speed of Light Propagation ($\tau_{11} = 66.796\ \mu\text{s}$) — **PASSED**
- `[TEST 5]` Ideal Coherent Synthesis Gain ($+9.54\text{ dB}$, $\eta = 100\%$) — **PASSED**
- `[TEST 6]` Low-SNR RCC Superiority (RCC 59.1% vs PP 55.6% vs CC 18.9% at 8 dB) — **PASSED**

---

## 5. Key Research Paper Takeaways Verified by Simulation

1. **Spatial Synchronization:** The servo parallel control model with $\alpha$-filtering reduces rotation errors below $0.1^\circ$ ($1/10$ of beamwidth), limiting beam pointing loss to $< 0.15\text{ dB}$.
2. **Two-Stage Compensation:** Coarse compensation aligns echoes to within a few sampling points; RCC then searches only within this small window.
3. **RCC Noise Rejection:** At low SNR ($8-13\text{ dB}$), conventional cross-correlation and peak picking lock onto false noise spikes across the range gate. RCC restricts search to $\pm 3$ to $\pm 7$ points, improving synthesis efficiency by **$> 20\%$**.
4. **Outfield UAV Results:** A 3-node C-band DARAR system tracking a drone at $17\text{ km}$ achieves an SNR gain of **$+8.69\text{ dB}$** (ideal $+9.54\text{ dB}$) with **$82.3\%$** efficiency, increasing detection power by **$65\%$**.
