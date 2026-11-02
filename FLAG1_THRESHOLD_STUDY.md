# Making Flag 1 work with sensor noise

## The problem
Flag 1 comes from the first observer. Its threshold band is the one in Table 2 of the paper (Lien, Peng and Chen, 2020):
half-width 0.0004 rad for roll and pitch, 0.002 m for x, y and z. The paper's simulations have exact measurements, so the band only
covers errors from the fault and from disturbances. It does not include sensor noise.

I measured the residual (measured value minus estimated value) in healthy flight. Its spread (standard deviation) is about the same as the
sensor noise, so it grows with the noise level:

| Noise level | Residual spread (roll, pitch) | Residual spread (x, y, z) | Time outside the band |
|---|---|---|---|
| 0 | 0.0002 to 0.0003 rad | 0.00001 to 0.0001 m | 0% |
| 1 | 0.005 rad | 0.02 m | 87 to 95% |
| 5 | 0.025 rad | 0.1 m | 97 to 99% |
| 10 | 0.05 rad | 0.2 m | 98 to 100% |

At noise level 1 the band is already 10 to 12 times too narrow. So Flag 1 (and `Fault_True`) stays on all the time.

## The fix
Make the margin wide enough for the noise: **Ku3 = Kl3 = max(paper value, k × sensor noise size)**.
`fd_noise_thresholds.m` does this. Nothing else in the detector changes.

## What I tested (30 runs per setting, gust wind, all four motors, 2 random cycles)
Healthy flights with noise (Fault_True on, as a share of the time) and failures with noise (found in time):

| Setting | Healthy, Fault_True on | Failure found in time | Median detection time | Right motor | Recovered |
|---|---|---|---|---|---|
| noise 1, paper values | 100% | 100% | 0.01 s | 100% | 100% |
| noise 1, k = 4 | **0%** | 100% | 0.02 s | 100% | 100% |
| noise 5, paper values | 100% | 100% | 0.01 s | 100% | 100% |
| noise 5, k = 2 | 7.2% | 100% | 0.03 s | 100% | 100% |
| noise 5, k = 3 | 0.03% | 100% | 0.06 s | 100% | 100% |
| noise 5, k = 4 | **0%** | 100% | 0.07 s | 100% | 100% |
| noise 5, k = 6 | 0% | 100% | 0.07 s | 100% | 100% |
| noise 5, k = 10 | 0% | 100% | 0.07 s | 100% | 100% |
| noise 10, paper values | 100% | 100% | 0.01 s | 100% | 100% |
| noise 10, k = 4 | **0%** | 100% | 0.06 s | 100% | 100% |
| noise 20, paper values | 100% | 0% | no value | 62.5% | 87.5% |
| noise 20, k = 4 | 0.4% | 50% | 0.05 s | 100% | 100% |
| noise 5, 60% thrust loss, k = 4 | 0% | 100% (partial loss found) | 0.07 s | 100% | 100% |

Also: without noise, the old band let `Fault_True` flicker on for 2% of the healthy time at the 6 s and 9 s reference steps. With the wider band it was 0%.

## What it means
* **k = 4 is the right choice.** k = 2 is not enough (7% false alarms), k = 3 is borderline (0.03%), k = 4 and above is clean.
* **Flag 1 now means something.** `Fault_True` is off in healthy noisy flight and rises when a failure happens, so the raw flag can be used as the first alarm.
* **Cost: slower detection.** The residual must now leave a wider band, so the median detection time goes from about 0.01 s to 0.02 s (noise 1) and 0.06 to 0.07 s (noise 5 and 10). It is still fast compared with the 0.5 s needed to confirm.
* **The band must match the real sensor noise.** If the band is designed for noise level 1 but the real noise is 5, Flag 1 will be on again. Measure your sensors and use their real noise sizes.
* **At 20 times noise it only partly works.** The margin becomes 0.4 rad (23 degrees) for attitude, so only half of the failures were found in time, and the healthy runs were only 50% fully clean. That noise level is more than a real sensor should have.
* **Not tested here:** noise-adaptive bounds that estimate the noise during flight, and the effect of the wider band on slow, weak faults.
