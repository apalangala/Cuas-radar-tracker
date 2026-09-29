# Interview prep: questions this project invites

You should be able to answer every one of these without notes. If you can't, that's the part of the code to reread. The answers are short on purpose. Expand them in your own words.

---

### Radar fundamentals

**Why FMCW and not a pulsed radar?**
FMCW transmits continuously at low peak power (0.5 W here), so the hardware is cheap and small. Mixing the echo with the transmitted chirp turns range into a low beat frequency, which a slow ADC can sample. That's why automotive and short-range C-UAS radars use it. The downside is transmit/receive isolation, since the radar transmits and receives at the same time.

**Where does range resolution come from?**
It comes from bandwidth: ΔR = c/2B. Specifically, it's the bandwidth you actually *sample*, 76.8 MHz here, not the full 150 MHz sweep. Only 25.6 µs of the 50 µs ramp is digitised.

**Why X-band?**
There are three reasons:
- Shorter wavelength gives better angular resolution for a given antenna size.
- The Doppler shift per m/s is larger, so slow drones are easier to separate from clutter.
- Small-drone RCS is better behaved than at lower frequencies.

The trade-off is more atmospheric and rain loss, which doesn't matter much at 500 m.

**What limits max unambiguous velocity, and why did you change it?**
It's v_max = λ/4T_PRI. The first design had ±125 m/s, which no drone needs. Lengthening the PRI traded that away for a longer dwell and 4× finer velocity resolution. That shrank the MTI blind zone from ±6 m/s to ±1.5 m/s.

### Signal processing

**Why window before the FFT?**
Without a window, the sidelobes of a clutter return 60 dB above the noise would bury a drone in nearby cells. Hann gives −31 dB sidelobes, at the cost of a wider main lobe and about 1.8 dB loss per dimension.

**Explain the CFAR bug.**
The textbook CA-CFAR scale factor assumes exponentially distributed noise, i.e. a single channel. My map is the sum of 8 channels, so the noise is Gamma(8)-distributed and much tighter. The textbook threshold made the true Pfa about 10⁻³⁰ instead of 10⁻⁵, which wasted 5.5 dB. The smallest drone went undetected. The fix was to solve P(Gamma(8) > α·8) = Pfa numerically. There's a unit test that measures the Pfa to prove it.

**What does MTI cost you?**
It blinds you to anything with near-zero radial velocity, including a target flying *across* your line of sight at speed. The tracker has to know about that blind zone.

**How accurate are your measurements, and how do you know?**
I measured it (`run_calibration.m`). The error scales as resolution/√SNR, which matches the Cramér–Rao bound behaviour. Sub-bin interpolation adds a small bias floor at high SNR, so I fitted both terms.

### Tracking

**Why an EKF and not a linear Kalman filter?**
The state is Cartesian but the radar measures range, angle and radial velocity. Those are nonlinear functions of the state, so the filter linearises them with the Jacobian at each step.

**Why does tangential velocity take time to converge?**
The radar only measures the *radial* component of velocity directly. The tangential component only becomes visible as the angle changes over several scans, and angle is the least precise measurement.

**What is NEES and why do you care?**
NEES is the error squared, normalised by the filter's own claimed covariance. A consistent 4-state filter averages 4.
- Much larger means the filter is over-confident. It will reject good measurements at the gate and lose the track.
- Much smaller means it's under-confident and noisier than it needs to be.

My baseline filter was *both*: over-confident on the weak target and under-confident on the strong ones, because it used one fixed noise level for everything.

**Why did you add a floor to the radial-velocity noise?**
With a near-perfect Doppler measurement, the EKF over-trusted it and hit NEES around 30. Real drones have micro-Doppler spread anyway. It's a deliberate model-mismatch margin, and I can show the before and after.

**What's M-of-N confirmation?**
A track is only confirmed after 3 detections in 5 scans. Random false alarms almost never line up like that, so false tracks stayed at zero across all Monte Carlo runs. The cost is about 0.3 s of latency before a new target is reported.

**Why greedy nearest-neighbour, and when does it fail?**
It's simple and fine for well-separated targets. It fails when targets are close together, because a greedy choice can steal the other track's detection. The upgrades are Hungarian assignment (optimal one-to-one) or JPDA (probabilistic).

### Evaluation

**Why Monte Carlo, and why paired?**
One run is an anecdote. Thirty runs give a mean and an error bar. "Paired" means all tracker variants see identical detections, so any difference comes from the tracker, not the random draw.

**What would you do next?**
- IMM filter, for the turning target
- Micro-Doppler features to tell drones from birds
- Elevation angle with a 2-D array
- Validate against real data, e.g. a TI mmWave dev kit or a public radar dataset

**What are the biggest simplifications?**
Point targets, 2-D geometry, no multipath and ideal hardware. See DESIGN.md §7.
