# Wrist calibration at native 120 Hz

The known-load runs confirm the force axis/sign transform. In particular,
positive world-Y and world-Z applied forces produce negative world-Y and
world-Z wrist reactions after the joint-frame rotation. The raw child-frame
components have the opposite signs on those axes.

The first criterion used the mean norm of the instantaneous residual, with a
0.002 Nm torque tolerance. Six of eight cases passed; Y torque and the mixed
load failed. This criterion combines calibration bias with distal-hand/grasp
inertial fluctuations. For example, the mean transformed Y reaction under
+0.010 Nm applied Y torque was approximately -0.010002 Nm, despite a mean
instantaneous torque residual norm of 0.00534 Nm.

The next calibration records signed residual components and separately checks
mean vector bias (0.05 N / 0.002 Nm) and mean instantaneous residual magnitude
(0.2 N / 0.01 Nm, 1% of the operational limits). Both remain reported. The
original failed criterion and raw results are preserved in calibration.json.
This separates a static frame/scale check from fluctuations in the raw reaction;
it does not make those fluctuations disappear or establish contact convergence.
