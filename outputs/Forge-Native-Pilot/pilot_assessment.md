# Pilot assessment

All three GPU runs completed: 7,200 finite samples in total, with continuous
120 Hz logging, all five commanded phases and no numerical-guard exit.
The collector uses the upstream FORGE/Panda environment and control loop.
The script SHA-256 matches the saved manifest. See validation.json for checks.

These runs reached the socket entrance but did not achieve insertion. Maximum
reported peg-base depth ranged from -0.014 to +0.031 mm, versus the intended
20 mm. Peak socket-on-peg forces ranged from 8.835 to 10.330 N; maximum reported
contact overlap ranged from 0.01074 to 0.01931 mm. Those overlaps are appreciable
relative to the nominal 0.057 mm radial clearance, so they do not establish a
converged contact model. No numerical-validity or safe-recovery labels were set.

The nominally centered command had about 5.26 mm actual lateral displacement
at its greatest attained depth. The native controller dead zone and pose/grasp
randomization make these commanded offsets unsuitable as isolated experimental
factors. The withdrawal command also did not guarantee a 10 mm achieved gap.
This is entrance contact under uncertain tracking, not evidence of deep-bore
jamming or a quantitative recovery-cost comparison.

The recommendation is to keep the upstream SDF scene and Panda, establish a
controlled nominal controller/grasp baseline, then implement roll/pitch targets
and matched checkpoint recovery probes. Separate wrist and peg/socket forces
are already available. The full 100-trajectory collection has not been started.
