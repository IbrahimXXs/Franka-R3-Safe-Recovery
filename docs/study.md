# Insertion and recovery force study

This benchmark asks which **tested scenario and recovery policy** require more
withdrawal force, contact-resisting work, or time. It does not assume that every
insertion blockage is hard to reverse, or that one failed retreat proves a state
is unrecoverable under all policies.

## Run

```bash
# Five scenarios, each with two recovery policies
./run.sh --study --headless

# Inspect one experiment in the GUI; the window stays open afterward
./run.sh --study --scenario loaded_deep --recovery straight

# Choose a destination and compare both recoveries for the deep loaded case
./run.sh --study --headless --scenario loaded_deep --recovery both \
  --output-dir outputs/my-comparison

# Parameter changes for a fresh experiment
./run.sh --study --headless --scenario loaded_deep --recovery straight \
  --friction 0.6 --clearance-mm 0.3 --contact-offset-mm 0.025
```

Study runs follow a complete trial schedule; `--steps` applies only to the ordinary
hold/demo launcher. A study directory contains a manifest, one raw CSV per trial,
`summary.csv`, `report.md`, and PNG/PDF force-profile plots. Existing study results
are not overwritten. A guarded trial is recorded as `numerically_invalid`, including the offending
sample in its partial CSV, and the next trial starts from the prepared state.
Its recovery costs are blank in `summary.csv` and it is excluded from comparison
plots. The manifest lists it under `invalid_trials`; a batch that finishes with
such trials has `status: complete_with_invalid_trials`. This means the schedule
finished, not that every experiment was valid. Unexpected errors or interruption
still stop the batch with `status: incomplete`.

## Scenarios and policies

| Scenario | Lateral offset | Tilt during insertion | Tilt imposed after insertion | Commanded depth |
| --- | ---: | ---: | ---: | ---: |
| aligned | 0 mm | 0° | 0° | 20 mm |
| offset | 0.4 mm | 0° | 0° | 20 mm |
| tilted | 0 mm | 3° | 3° | 20 mm |
| loaded_shallow | 0 mm | 0° | 5° | 10 mm |
| loaded_deep | 0 mm | 0° | 5° | 20 mm |

Offsets are along world X, tilts about world Y. The commanded rotation is about
the peg tip. These are target poses, not teleported states: contacts and robot
compliance determine the actual achieved depth and orientation, which are logged.
The round peg's axial yaw is not a geometric misalignment variable here.

Each run starts from the same arm joint state and settles above the opening.
The recorded sequence is 0.5 s free baseline, 6 s insertion, 0.5 s hold, 2 s
rotation/loading, and 0.5 s loaded hold. Recovery is either:

- **straight:** withdraw vertically over 6 s while retaining the commanded tilt
  and offset, then hold clear for 0.5 s.
- **realign:** first remove tilt and offset over 2 s at the commanded depth,
  then perform the same vertical withdrawal. This adjustment is included in
  recovery time and work. It is a fixed candidate policy, not guaranteed optimal.

The commanded retreat ends with the peg tip 25 mm above the opening. A clear
state requires the entire cylindrical peg to be at least 2 mm above the opening
and total normal contact load below 0.1 N for 0.2 s. A case that never enters by
at least 1 mm is reported separately as `did_not_enter`.

## Contact and robot model

The synthetic baseline has an 8 mm × 60 mm peg, 0.5 mm radial clearance, a 25 mm
blind bore, and a 1 mm entry chamfer. The socket has 192 facets. These dimensions
are experimental choices, not dimensions of a real test specimen.

Static and dynamic friction are both explicitly set to 0.3. Contact uses an
implicit compliant normal model with stiffness 100,000 N/m and damping 100 Ns/m;
these are simulation parameters, not identified elastic properties or a claim
that the rendered metal has those properties. The default contact offset is
0.05 mm per shape, rest offset is zero. Friction offset and correlation distance
are explicit in the manifest. Compliant contact permits overlap as an approximate
deflection; the raw minimum separation is therefore always recorded.

The 30 g peg is part of the hand's compound rigid body. The stock hand's mass,
COM, and inertia are queried before adding the peg, then combined with the
analytical solid-cylinder payload using the parallel-axis theorem. This retains
the ideal rigid grasp without an ill-conditioned tiny fixed leaf link. There is
no finger slip or release. Gravity is disabled on the arm and compound payload
as an ideal gravity-compensation assumption.

The study uses Cartesian impedance with 1,500 N/m translation stiffness,
20 N m/rad rotation stiffness, and damping of 80 Ns/m and 2 Nms/rad. Joint torques
are capped by the configured arm limits. An implicit PD prediction uses the
current articulation mass matrix to prevent explicit wrist damping from becoming
unstable at finite timestep. The implementation linearizes the task Jacobian per
step and approximates velocity-dependent dynamics; it is not a hardware controller
or a claim of unconditional stability in contact. The numerical idea follows
[Tan, Liu and Turk's stable PD formulation](https://faculty.cc.gatech.edu/~turk/my_papers/stable_pd.pdf).

Physics, torque control, and raw force logging run at 480 Hz by default; rendering
runs at 30 Hz. TGS uses at least 64 position and 4 velocity iterations. Study
parameters and payload mass properties are saved in `study.json`.

## What the force profiles measure

The signal is **fixture-on-tool contact wrench**, including normal and tangential
friction contributions, in world coordinates. Torque is calculated about the
peg center. It is privileged simulator contact data, not a noisy wrist sensor,
not commanded actuator effort, and not an inferred force from pose error.

The custom readout explicitly enables `/physics/disableContactProcessing = false`,
as Isaac Lab's standard contact sensor does. This is required for CPU contact
reporting; switching to `--device cpu` without it previously produced unusable
zero-force diagnostics. The manifest now records the physics device and this
setting. GPU physics remains the default.

The sensor body is the hand/peg compound and the filters include only the socket
and hole-bottom pedestal. In the prescribed motion envelope the palm remains
clear, so these contacts are on the peg. If future tasks let the palm touch the
fixture, those contacts would also be included and must be distinguished.

The readout uses PhysX `get_contact_data(dt)` and `get_friction_data(dt)` and sums
only the valid pair ranges. This matters because the installed Lab 2.3 standard
contact sensor reports normal force, which omits the friction central to this
study. Moments use contact locations, and impulses are divided by the actual
physics timestep. See the [PhysX tensor API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physics.tensors/docs/api/python.html).

Raw CSV fields include `fx/fy/fz` in N; `taux/tauy/tauz` in N m; their normal and
friction force components; contact count and total normal load; peg COM velocity
`vx/vy/vz` in m/s and angular velocity `omegax/omegay/omegaz` in rad/s; actual depth,
tilt, orientation quaternion, and arm joint positions; phase and commanded depth
and tilt. Equal opposing wall forces can cancel in the net force, so total normal
load is recorded separately.

## Recovery costs and interpretation

- **Insertion resistance:** max(Fz, 0), since insertion is downward.
- **Retreat resistance:** max(−Fz, 0), since retreat is upward.
- **Peak resistance:** both raw and 50 ms moving-average peaks are reported.
- **Impulse:** integral of axial retreat resistance over time, in N s.
- **Work proxy:** integral of max(0, −(F·v + τ·ω)) during the full recovery.
  Force, torque, velocities, and moment origin are consistent. This includes
  rotational recovery cost but is not motor energy or pure frictional dissipation.
- **Recovery time:** time from recovery start until sustained clearance.
- **Force/torque budgets:** classify whether the completed recovery stayed within
  20 N net force and 1 N m torque by default. These are evaluation thresholds;
  the controller does not enforce them.

Always compare these together. A stalled peg can have zero work while requiring
high force. A policy may withdraw successfully while exceeding a force budget.
Different achieved insertion depths are not matched-depth experiments. Reports
state those depths explicitly rather than attributing all differences to tilt.

The penetration screen flags overlaps exceeding 25% of radial clearance. This is
a conservative review flag, not a physical material limit or a convergence proof.
A trial stops at 500 N net force or 1 mm overlap and is marked numerically
invalid. Other trials continue; the invalid one never receives a recovery ranking.
A numerical guard failure is not evidence of physical jamming. The guard is not
relaxed automatically to make a run finish.

## Validation and next experiments

```bash
conda activate franka-safe-recovery
python tests/check_contact.py --headless
python tests/check_scene.py
python -m unittest discover -s tests -v
```

The contact probe checks zero force in free space and, for a 1 kg resting body,
a 9.81 N normal reaction, 2 N friction reaction, and 0.02 N m torque against known
applied loads, using settled time averages and a per-step Newton force-balance
check against measured acceleration. Metric tests cover failed withdrawal, zero-work stalls, force
signs, and inclusion of rotational recovery cost.

Before interpreting a ranking, compare the same case at `--physics-hz 480` and
`--physics-hz 960`, then refine contact offsets and `--socket-segments`. Friction
ablations (`--friction 0`) check the tangential contribution. Material friction,
contact stiffness/damping, controller stiffness, clearance, approach errors,
and depth remain study parameters. There is no sim-to-real validation or learned
recovery model yet.

## First saved pilot

The [pilot report](../outputs/recovery_pilot/report.md) compares aligned and
loaded-deep cases with both recovery policies. Read its
[validation and timestep comparison](../outputs/recovery_pilot/validation.md):
loaded cases exceed the overlap review screen, and work changes by about 19%
between 480 and 960 Hz. These are workflow demonstrations, not validated cost
rankings or physical jamming thresholds.

## Offset-trial failure investigation (2026-09-13)

The original default sweep stopped during `offset / straight` around 3.875 s,
near the 1 mm chamfer shoulder. The commanded 0.4 mm offset reached about
0.515 mm actual tip offset before the failure, so commanded clearance alone did
not ensure collision-free entry. The guard reported 1.3121 mm penetration.

A separate original-mesh run at 960 Hz also hit the guard (1.2980 mm), and a
480 Hz experiment that subdivided axial/radial mesh edges to at most 1 mm still
hit it (1.2399 mm). Long narrow triangles are a documented
[PhysX contact-stability risk](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/collision.html),
but these experiments do not establish them as this failure's cause. Neither
change was adopted as a cure. Physics parameters and the guard remain unchanged.
The physical/numerical origin of this contact transient needs further isolation;
the implemented fix is failure isolation and honest reporting of invalid data.

The full default suite has ten trials: five scenarios × two recovery policies.
Use `./run.sh --study --headless` to run it in a fresh timestamped directory.
Existing interrupted runs are preserved; the launcher does not resume them.

After enabling CPU contact processing, all ten trials completed on CPU. Their
[evaluation notes](../outputs/full_suite_cpu_contacts_enabled/validation.md)
explain why completion is not numerical validation: all non-aligned cases
exceeded the overlap screen, and mesh material-face-index warnings remain.
Known-load probes passed on both CPU and GPU. The simpler probe does not validate
the peg/bore collision response.

The [full GPU sweep after the runner fix](../outputs/full_suite_guard_fix/report.md)
attempted all ten trials and exited successfully: eight completed, and the two
offset trials were numerically invalid. Their final guard samples are retained,
with blank costs in the summary. See the [verification record](../outputs/full_suite_guard_fix/validation.md).
