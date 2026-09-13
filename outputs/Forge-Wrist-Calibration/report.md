# Wrist known-load calibration

All eight load cases passed: **True**.

Loads are applied at the measured peg COM in world axes, outside socket contact. Each case lasts three seconds; statistics use its final second. Static vector bias checks sign, axes, scale and the moment shift. Instantaneous residuals also include distal-link and grasp dynamics, and are bounded separately. This does not certify contact-force convergence across timesteps.

Limits: static force bias < 0.05 N, static torque bias < 0.002 Nm; mean instantaneous residual norm < 0.2 N and < 0.01 Nm; socket contact < 0.001 N.

| Load case | Force bias (N) | Torque bias (Nm) | Mean force residual norm (N) | Mean torque residual norm (Nm) | Passed |
|---|---:|---:|---:|---:|---|
| zero | 0.000008 | 0.000104 | 0.000014 | 0.000104 | True |
| force_0 | 0.000222 | 0.000443 | 0.002593 | 0.000755 | True |
| torque_0 | 0.000155 | 0.000179 | 0.002185 | 0.000394 | True |
| force_1 | 0.000014 | 0.000367 | 0.005980 | 0.000992 | True |
| torque_1 | 0.000227 | 0.000068 | 0.026256 | 0.005341 | True |
| force_2 | 0.000176 | 0.000078 | 0.000333 | 0.000085 | True |
| torque_2 | 0.000234 | 0.001362 | 0.002367 | 0.001393 | True |
| mixed | 0.000728 | 0.000609 | 0.022941 | 0.005236 | True |
