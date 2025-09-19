# Signed-Bias Dead-Zone DC Motor Identification & PI Control (Arduino + L298N)

**What this repo delivers**
A reproducible pipeline that turns raw hardware logs from a low-cost DC drive (Arduino + L298N + geared DC motor) into a predictive plant model and a simple PI controller that works on the real motor.

**Key idea**
Cheap drivers show a hard **dead-zone**, large voltage drop, and **direction-dependent static bias**. We model the actuator as:

> PWM → **Voltage LUT** → **Dead-Zone** (±V_DB) → **Signed Bias** (B₊/B₋) → 1-pole plant \( G(s)=K/(\tau s+1) \)

A fast **three-simulation trick** precomputes responses to: (1) \(u_\mathrm{eff}\), (2) \(1\{u_\mathrm{eff}>0\}\), (3) \(1\{u_\mathrm{eff}<0\}\). Any \((B₊,B₋)\) is then scored by a cheap linear combo, enabling wide grid search without re-simulating.

---

## Highlights
- **Actuator linearization:** PWM→Voltage lookup (per direction) to cancel L298N losses  
- **Model:** Dead-zone + **signed static bias** + \(K,\tau\) 1-pole dynamics (delay ≈ 0)  
- **ID & CV:** Robust smoothing/step-fits + fast grid search for \((B₊,B₋,D)\)  
- **Controller:** Feedforward inverse (LUT + DZ + biases) + IMC-PI on \(G(s)\)  
- **Artifacts:** Python notebook, MATLAB exporter, Simulink block diagram, figures

Typical numbers from our run: \(K \approx 34.2\) rpm/V, \(\tau \approx 0.241\) s, \(V_{DB}\approx \pm 4.5\) V, \(B_+\sim 2.3\) V, \(B_-\sim 2.6\) V.

---

## Repo layout
data/ # CSV logs (run_minimal.csv, designed_stair.csv, step_index.csv)
notebooks/ # DC_Motor_ID_SignedBias_DeadZone_IMCPI.ipynb
simulink/ # Minimal .slx: LUT + DZ + signed-bias + TransferFcn
figs/ # full_overlays_top3.png, final_residuals.png, combined_scores_bar.png, residuals_heatmap.png
id_outputs/ # step_fits.csv, cross_val_scores.csv, recommended_tf_with_bias_delay.csv
matlab/ # stair generator + CSV exporter (export_minimal_run.m)
