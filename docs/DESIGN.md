# Design notes

This document explains *why* each part of the system is the way it is. The README shows what it does. Numbers refer to the default `radar_params.m`.

---

## 1. Mission and requirements

A short-range counter-UAS (C-UAS) sensor has to find small, slow, low-flying drones against a background of trees, buildings and poles. Those requirements drive the design:

| Requirement | Why it matters | Design response |
|---|---|---|
| Detect ~0.02 m² targets (small quadcopter) out to ~300 m | Small drones are among the hardest radar targets | X-band, 15 ms coherent dwell, 8-channel array, 54 dB processing gain |
| Separate drones from static clutter | Clutter returns are 40–70 dB stronger than a drone | Doppler processing plus an MTI clutter canceller |
| Measure speeds up to ~40 m/s without ambiguity | Fast fixed-wing UAVs | PRI = 120 µs gives ±62.5 m/s |
| Resolve slow targets from clutter | Quadcopters cruise at 5–15 m/s | 128 chirps give 0.98 m/s velocity resolution |
| Hold tracks through missed detections and blind zones | Fluctuating RCS, tangential flight | M-of-N logic, notch-aware coasting |

## 2. Waveform (FMCW)

| Quantity | Formula | Value |
|---|---|---|
| Wavelength | λ = c / f_c | 3.0 cm (10 GHz) |
| Chirp slope | S = B / T_chirp | 3 MHz/µs |
| Sampled bandwidth | B_eff = S · N_s / f_s | 76.8 MHz |
| Range resolution | ΔR = c / (2 B_eff) | **1.95 m** |
| Max range (complex IQ sampling) | R_max = f_s c / (2S) | 500 m |
| Velocity resolution | Δv = λ / (2 N_c T_PRI) | **0.98 m/s** |
| Max unambiguous speed | v_max = λ / (4 T_PRI) | ±62.5 m/s |
| Coherent dwell (CPI) | N_c · T_PRI | 15.4 ms |
| Angle resolution | ≈ 2/N_rx rad at boresight | ~14° |

**Design iteration:** the first version used 64 chirps with a 60 µs PRI, which gave 3.9 m/s velocity resolution. That made the MTI blind zone ±6 m/s wide, and a crossing drone vanished for **5 seconds**. Doubling the dwell to 128 chirps and lengthening the PRI (drones don't need ±125 m/s) shrank the blind zone to ±1.5 m/s, cut the crossing drone's gap to about 1–1.5 s, and added 3 dB of integration gain.

## 3. Link budget

The radar equation gives the per-sample SNR:

```
SNR = Pt·Gt·Gr·λ²·σ / ((4π)³ · R⁴ · k·T0·f_s·NF·L)
```

With Pt = 0.5 W, Gt = 20 dBi, Gr = 12 dBi per element, NF = 6 dB and L = 4 dB, for a **0.02 m²** quadcopter:

| Range | SNR per sample | After ideal integration (+54.2 dB) |
|---|---|---|
| 100 m | −7.5 dB | 46.7 dB |
| 200 m | −19.5 dB | 34.7 dB |
| 300 m | −26.5 dB | 27.6 dB |
| 400 m | −31.5 dB | 22.6 dB |

Real losses reduce this. The Hann windows cost ~3.5 dB in total, and summing the 8 channels non-coherently gains less than a coherent sum would. Targets also **fluctuate** (Swerling I: the RCS is redrawn every CPI from an exponential distribution). A fluctuating target needs roughly 8–10 dB more SNR than a steady one to reach Pd = 0.9, which is why the Pd curve drops earlier than the table suggests.

## 4. Signal processing

### 4.1 MTI (static-clutter canceller)
Stationary objects return the same phase on every chirp, so their contribution is the slow-time **mean**. Subtracting that mean removes them. Scatterers moving slowly in the wind (0.1 m/s RMS) leave a small residue in the ±1 bins around zero Doppler, so those bins are also blanked.

In one test frame this cuts the detections from 53 to 3.

**Cost:** any target whose *radial* velocity is inside ±1.46 m/s is also removed, even if it is moving fast tangentially. The tracker has to handle that (§5.5).

### 4.2 Range-Doppler map
A Hann window is applied in both dimensions (−31 dB sidelobes, which keep clutter sidelobes from masking drones), followed by a range FFT and a Doppler FFT. The power is then summed across the 8 elements (non-coherent integration).

### 4.3 CFAR: the non-obvious bit
Cell-averaging CFAR estimates the local noise from training cells and multiplies it by a factor α. The textbook formula α = N(Pfa^(−1/N) − 1) assumes each cell is **exponential**, i.e. one channel. This map sums 8 channels, so noise-only cells are **Gamma(8)**-distributed and much less spread out.

| Threshold choice | α | True Pfa |
|---|---|---|
| Textbook (single channel) | 11.5 (10.6 dB) | **1.2 × 10⁻³⁰** |
| Correct for Gamma(8), solved with `fzero` + `gammainc` | 3.27 (5.1 dB) | 1 × 10⁻⁵ |

The textbook formula silently throws away **5.5 dB** of sensitivity. The first version of this project missed the smallest drone because of it. `test_cfar_false_alarm_rate.m` now checks the measured Pfa against the design value.

### 4.4 Peak grouping, interpolation, angle
- **Peak grouping:** only CFAR hits that are local maxima in a 3×3 neighbourhood are kept, giving one detection per target.
- **Interpolation:** parabolic interpolation on log-power gives sub-bin range and velocity estimates.
- **Angle:** a 256-point zero-padded FFT across the 8 elements at the detected cell, followed by the same interpolation.
- **SNR:** each detection's SNR is peak power over the CFAR's local noise estimate, and the tracker uses it.

## 5. Tracking

### 5.1 State and models
- State **x** = [p_x, v_x, p_y, v_y], a nearly-constant-velocity model with white-noise acceleration (q = 3 m/s²).
- Measurement **z** = [range, azimuth, radial velocity], which is nonlinear in x, so this is an **Extended Kalman Filter**. The Jacobian is analytic (`meas_model.m`) and checked against finite differences in `test_meas_model_jacobian.m`.
- The covariance update uses the Joseph form, for numerical robustness.

### 5.2 Association
- Gating uses the Mahalanobis distance with a χ² gate (3 dof, 99.9%).
- Assignment is greedy **global nearest neighbour**. It is good enough for well-separated targets. For closely spaced ones, Hungarian/auction assignment or JPDA would be the upgrade.

### 5.3 Track management
- **Initiation:** position comes from the detection. The radial velocity is measured, but the tangential velocity is unknown, so it gets σ = 30 m/s along the tangential direction only.
- **Confirmation:** 3 hits in 5 scans (M-of-N). In every Monte Carlo run this rejected all false alarms.
- **Deletion:** after 2 consecutive misses (tentative tracks) or 5 (confirmed tracks).

### 5.4 SNR-adaptive measurement noise
Estimation theory says measurement error scales as σ ∝ resolution / √SNR. `run_calibration.m` checks this by simulation and fits

```
σ² = (k · resolution / √SNR)² + floor²
```

Each detection then gets its own covariance: a weak, far-away detection is trusted less than a strong, close one.

![calibration](img/calibration.png)

**Radial-velocity floor:** calibration on a point target gives a floor of about 0.01 m/s. Real drones spread their Doppler return by tenths of a m/s through rotor and body micro-Doppler. More importantly, feeding an EKF an over-precise range-rate made it **over-confident**: NEES ≈ 30 when it should be 4, through the nonlinear coupling between range-rate and position. The floor is therefore set to 0.15 m/s on purpose, as a model-mismatch margin.

### 5.5 Doppler-notch-aware coasting
The tracker knows where the MTI blind zone is. If a confirmed track's predicted radial velocity is inside the notch (plus 2σ of its own velocity uncertainty), a miss is *expected*, so it doesn't count towards deletion. This is capped at 4 s. Without it, every tangential crossing breaks the track.

## 6. Evaluation methodology

- **Paired Monte Carlo:** each seed simulates the radar front end once, and all three tracker variants run on *identical* detections. Differences between variants therefore come from the tracker alone, not from luck in the detections.
- **Metrics:**
  - Steady-state RMS position and velocity error (excluding the first second of each track)
  - Coverage: the fraction of frames with a confirmed track on the target
  - Track breaks
  - False confirmed tracks
  - NEES
- **Filter consistency:** NEES = eᵀP⁻¹e. For a consistent 4-state filter it averages 4. Across N runs, the average NEES must lie inside the χ²(4N)/N 95% band.
  - Above the band: the filter is **over-confident**. It will reject correct detections at the gate and lose the track.
  - Below the band: it is **under-confident**. It is noisier than it needs to be and easier to seduce with clutter.

## 7. Assumptions and limitations

The limitations are stated openly, because they are the obvious next steps:

- Point targets with no micro-Doppler signature (so no drone vs bird classification yet)
- 2-D only: azimuth, no elevation
- No multipath, ADC saturation, phase noise or array calibration errors
- Range-Doppler coupling and target motion within a CPI are neglected (both under 0.1 bin here)
- A single constant-velocity motion model. The turning target shows the mismatch (NEES above 4 during the turn), which is what an IMM filter fixes.
- Greedy GNN association and a single sensor
