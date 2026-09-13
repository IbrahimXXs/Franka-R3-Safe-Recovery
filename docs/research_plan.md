# Preserving Withdrawal Feasibility in Uncertain Robotic Assembly

Research and implementation plan | Franka + Isaac Sim / Isaac Lab | 11 September 2026

Prepared from ROB8501_Safe_Recovery_Project_Brief.pdf, supplied in this conversation, especially pp. 1-3. This is a proposed research program, not a report of completed experiments. Numerical settings below are initial design choices unless explicitly attributed to a source.

## 1. Recommended scope and paper claim

Use a Cartesian impedance controller, a small library of feedback motion primitives, a learned model of constrained withdrawal outcomes, and a short-horizon planner that checks whether candidate insertion motions preserve a usable recovery policy. Start with a pregrasped rigid peg, known CAD, a fixed fixture, and uncertainty in relative alignment, friction, and grasp offset. Keep perception and grasp acquisition outside the first experiment.

The central question is: **At the same force/torque limits and retry budget, does anticipating loss of withdrawal feasibility reduce manual intervention while preserving assembly completion?**

Working paper title: **Preserving Force-Constrained Withdrawal Feasibility in Belief-Space Assembly**. The proposed contribution is a contact-specific prediction and decision criterion; impedance control, recovery policies, belief-space planning, and backup safety filters are established ideas [1-9].

Three candidate contributions:

1. An action-conditioned estimate of whether an available feedback recovery maneuver can clear a held part within a fixed time and measured wrench budget, including unsuccessful and out-of-distribution cases.
2. A computationally tractable belief-space action filter that preserves a single implementable backup policy across uncertain states, with explicit assumptions for any theoretical statement.
3. Evidence that present contact wrench and predicted insertion success can be insufficient: states with similar immediate measurements can have different future withdrawal outcomes, and this distinction improves real assembly reliability.

An 8-week effort can plausibly produce a complete prototype and preliminary real results. Plan for 12 weeks for a credible paper draft, assuming one substantially dedicated researcher, regular advisor feedback, an available Franka/FCI setup, machining access, and a compatible GPU workstation. Acceptance at RSS cannot be promised. The brief itself notes that a broader algorithmic insight or principled result plus physical evidence is likely needed.

As of this check, the official RSS site announces RSS 2027 in Athens, July 6-11, 2027, but the submission dates displayed on its main page are still for RSS 2026. Use this schedule as a research deadline; verify the eventual RSS 2027 call separately [18]. Starting Monday, September 14 gives an 8-week checkpoint on November 8 and a 12-week draft on December 6, 2026.

## 2. Literature: what to reuse and what to distinguish

| Work | Relevant established idea | Project implication |
|---|---|---|
| TacDiffusion, ICRA 2025 [1] | Learned 6D wrench actions and filtering between policy and control rates. | Optional nominal policy if demonstrations and the controller interface are already available. A diffusion model is not necessary for the recovery question. |
| FORGE, revised 2025 [2] | Force-aware sim-to-real assembly, dynamics randomization, force conditioning, and success prediction. | Closest force-aware nominal-skill reference. Predict withdrawal feasibility separately from insertion success. |
| BILBA, 2024 [3] | Contact schedules, particle beliefs, and compliant motion selection; its formulation seeks a conformant sequence over initial uncertainty. | Closest compliant planning reference. Our proposed replanning incorporates new wrench observations and explicitly evaluates backup withdrawal. |
| Recovery RL, 2021 [4] | Separation of task and learned recovery behavior. | A recovery classifier or task/recovery switch alone is insufficient novelty. Compare against a recovery-switching baseline. |
| TAMPURA, RSS 2024 [5] | Planning with uncertain outcomes, information gathering, and risk. | Do not claim uncertainty-aware planning or avoidance of irreversible outcomes as a new general idea. |
| Flow-based domain randomization, 2025 [6] | Learning operating distributions and using them for uncertainty-aware skill sequencing. | Explicitly distinguish recovery feasibility from a skill's success region and OOD score. |
| Backup CBFs; predictive safety filtering [7,8] | Preserving access to backup behavior or a safe terminal set. | A terminal recovery constraint is established control structure. The new work must address contact uncertainty, estimation, computation, or stronger evidence. |
| FORGE-plus, July 2026 preprint [9] | A force-signature-based recovery menu selected after failure; simulation-only evaluation with an LLM supervisor. | Additional recent comparison. Our target is prospective withdrawal feasibility, with a fixed operational budget and physical validation. An LLM is unnecessary for the core experiment. |

Read [1-3] for implementation, [4-8] for novelty and formulation, and [9] before freezing the contribution. This is a targeted literature check, not a claim of exhaustive novelty clearance. Recheck citing/new papers in Week 1 and before submission. Distinguish authors' reported results from results independently reproduced by this project.

## 3. Task, state, observations, and actions

Use a hole coordinate frame H, with +z pointing out of the hole. Define insertion depth d as positive downward. Every pose, twist, and wrench used together must be expressed at a specified origin and in a consistent frame.

Let q and dq be the seven joint positions and velocities; T_HP the peg pose relative to the hole; nu the peg twist; c a latent contact mode; and theta the uncertain parameters. A practical theta includes fixture pose error, grasp offset, friction, and effective compliance. Geometry and clearance can be known per fixture; treat manufacturing variation as uncertain only when measured variation justifies it.

$$s_t=(q_t,\dot q_t,T_{HP,t},\nu_t,c_t,\theta),\qquad s_{t+1}=f(s_t,a_t,\eta_t).$$

For the rigidly held-part approximation, the underlying manipulator/contact model is:

$$M(q)\ddot q+c(q,\dot q)+g(q)=\tau+J_c(q)^T\lambda.$$

$$\phi_i(q)\geq0,\quad\lambda_{n,i}\geq0,\quad\phi_i\lambda_{n,i}=0,\quad\|\lambda_{t,i}\|_2\leq\mu_i\lambda_{n,i}.$$

Here phi is the signed contact gap, lambda the environment-on-robot/held-part contact load, J_c the contact Jacobian, and mu the friction coefficient. These are unilateral-contact and Coulomb-cone constraints; sliding also needs an opposing-motion or maximum-dissipation law, and impact requires an impulse/time-stepping model. Multiple contacts and uncertain friction make the relation between insertion progress and withdrawal effort history-dependent. Use Isaac's solver for data generation and a reduced quasi-static model for analysis; do not make solving full contact complementarity online the first implementation target.

The observation contains joint state, TCP pose/twist, estimated or measured external wrench, gripper width, the commanded equilibrium/gains, and a short history. True hole pose, friction, and contact labels may supervise simulation learning but must not become unacknowledged deployment inputs.

$$o_t=h(s_t)+\epsilon_t,\qquad b_t(s)=P(s_t=s\mid o_{0:t},a_{0:t-1}).$$

Start with 16-32 weighted particles over alignment and friction and a 0.1-0.5 s history of wrench and motion. This is an approximate belief: similar force traces can arise from different contacts. An analytic Gaussian observation model is a starting approximation, not a claim that friction is exactly observable.

$$b_{t+1}(s')\propto p(o_{t+1}\mid s')\int p(s'\mid s,a_t)b_t(s)\,ds.$$

Implement each action as a closed-loop option: an equilibrium displacement/rotation, a fixed gain preset, duration, and termination conditions. Suggested library: advance, unload, retreat, lateral slide in four directions, bounded roll/pitch correction, and low-amplitude probing. Keep each planned segment short, initially 0.1-0.25 s; use a separate total recovery horizon such as 3 s, adjusted to the travel distance and commissioned speed.

Recovery policies can be sequences with feedback: unload then retreat; unload, counter-tilt, then retreat; or unload, lateral release, then retreat. A single failed upward pull is not evidence of physical irrecoverability.

Define G_clear using the entire peg being above the hole entrance with geometric margin, low residual contact, small velocity, and a confirmed grip. Define G_seated using depth, alignment, and a settled verification interval. Use pose/depth evidence as well as force: a stall is not proof of seating. Until a terminal release is deliberately modeled, keep the part grasped and require recovery feasibility even at the seated pose. Exclude snap-fits and irreversible press-fits initially.

## 4. Operational constraints and recoverability

Let F_t be external force in newtons, M_t the moment in newton-metres about the specified peg/TCP origin, and L_F, L_M the selected operational limits. Joint torque/velocity and workspace limits remain additional constraints. Use separate normalized terms; adding raw N and N m has no meaning.

$$g(s_t)=\max\left(\frac{\|F_t\|_2}{L_F},\frac{\|M_t\|_2}{L_M}\right)-1.$$

Let S be the states satisfying g <= 0 plus joint, workspace, grasp, and collision requirements. Set the actual numerical limits from the robot, sensor, fixture, and part characterization; the stock simulator gains and thresholds are not hardware approvals. Net wrench can hide opposing internal contacts, so budget compliance does not prove absence of part damage. Log contact-level loads in simulation as additional diagnostics where available.

For a recovery policy pi_r in a finite library Pi_R, define success over H_R samples:

$$Y_r=\mathbf{1}\{\exists k\leq H_R:s_k\in G_{clear},\ s_j\in S\ \forall j\leq k\}.$$

Then belief-level recoverability is:

$$p_R(b)=\max_{\pi_r\in\Pi_R}P(Y_r=1\mid b,\pi_r),\qquad \mathcal{R}_\delta=\{b:p_R(b)\geq1-\delta\}.$$

The order of operations matters. Choose one observation-based policy for the current belief, then evaluate it across particles. Taking each particle's individually best maneuver and averaging assumes the robot knows the hidden true state. A feedback policy can branch on future measured signals, but not on simulator-only variables.

Use delta as a tunable research risk parameter, initially explore 0.05-0.20 in simulation. It is not a certified hardware failure rate. Fix the selected operating point before testing, and present a recovery/completion/time tradeoff rather than choosing a favorable threshold afterward.

For a deterministic reduced model, define a constrained backup-reachable set over augmented state (including unknown-but-fixed parameters). It is a set reachable to G_clear under a specified backup policy and disturbance model, not automatically the maximal viability kernel. A finite-library failure label means **not recovered by this library within the budget**, not **no possible recovery exists**.

## 5. Learning a useful recovery predictor

Train an ensemble of five small models, initially two 128-unit hidden layers or a compact temporal encoder with an MLP head. Start with a logistic classifier and a force-threshold feature baseline first; keep the neural model only if it adds measurable value. Input deployable history/belief features, action parameters, recovery-policy ID, and the operational budgets.

The action-conditioned model predicts the outcome of **execute candidate a, then execute recovery policy pi_r**. This directly answers the planning question and avoids making a full high-dimensional learned simulator the first critical dependency. Add a short transition model for depth, observation features, and belief propagation only if a longer search horizon helps.

Targets: binary constrained recovery success Y, peak force, peak moment, and recovery time or a timeout indicator. Optional auxiliary targets are depth, contact-mode label, and progress. Use simulator labels only as training targets; retain a strictly deployable observation interface.

$$\mathcal{L}=\operatorname{BCE}(Y,\hat p)+\lambda_F\ell_{quant}(F_{peak},\hat F)+\lambda_M\ell_{quant}(M_{peak},\hat M).$$

A useful continuous diagnostic is the observed normalized withdrawal load max(F_peak/L_F, M_peak/L_M) for a completed maneuver. Timeouts need a separate failure target; they do not have a valid successful-withdrawal load. If a rollout is stopped at a limit, record the measured prefix peak and mark the true eventual peak as right-censored. Do not train it as the complete peak of an unrestricted future rollout.

Collection procedure:

1. Generate physically reached insertion prefixes, from easy contact to boundary states, with varied pose, friction, clearance, and control settings.
2. At selected decision points, replay the identical prefix for each candidate continuation and recovery policy. Save controller filter state, action history, and belief state. Pose/velocity cloning alone may not reproduce contact solver memory or friction history.
3. Record complete outcomes, including timeouts, loss of grasp, limit crossings, and unsuccessful recovery.
4. Start with 2,000 decision states x 4 recovery policies = 8,000 recovery rollouts. Grow toward 10,000 x 4 = 40,000 only if throughput and coverage justify it. Candidate-conditioned data increases this budget; use an initial subset of candidates and active sampling.
5. Split by entire insertion prefix/episode and physical condition before extracting windows. Sibling counterfactual rollouts must stay in the same split. Reserve distinct geometry/friction regions and later real sessions for final testing.

Oversample boundary states for training, but evaluate calibration on a deployment-like distribution or account for changed class proportions. Active collection should target ensemble disagreement and false-safe cases; it must not touch the locked test set.

Use held-out data to calibrate scores and choose thresholds. Ensemble mean-minus-spread is a ranking heuristic, not automatically a confidence lower bound. Report false-safe rate among accepted predictions, coverage, reliability curves, Brier score, and precision-recall metrics. Real-world calibration needs its own trials. For a fixed policy and independent episodes, zero observed failures in n trials has a one-sided 95% binomial upper bound 1-0.05^(1/n), approximately 3/n; 30 clean trials are not evidence of a 1% failure rate.

## 6. Planner and principled extension

At each decision, score 10-30 candidate feedback options against the current belief. For each option, evaluate progress and path-limit risk, then its continuation-to-recovery predictions. Start with one-step lookahead and receding execution. This is already predictive; full POMDP tree search is unnecessary for the first paper experiment.

$$a_t^*=\arg\max_{a\in\mathcal{A}_{adm}(b_t)}\left[w_dE(\Delta d)-w_t\Delta t-w_cE(C_{contact})-w_uU\right].$$

$$\mathcal{A}_{adm}(b)=\{a:\hat p_{path}(b,a)\geq1-\alpha,\ \hat p_R(b,a)\geq1-\delta\}.$$

Here p_path concerns the entire candidate segment, not just the endpoint; p_R concerns a subsequent common feedback recovery policy. The hats denote empirical predictions with held-out validation. C_contact can be the time integral of squared normalized force and moment, and U an uncertainty penalty. The action-conditioned p_R model is the practical substitute for propagating a complete posterior. For a particle implementation, aggregate probabilities per recovery policy before maximizing over policies.

Execution logic:

1. Check the measurement monitor and model support. Update the belief from the new history.
2. If seating is verified, complete the task while maintaining the stated terminal convention.
3. Evaluate candidate segment and recovery outcomes. Select a progressing admissible candidate and execute only its short segment.
4. If no candidate is admissible, use the currently validated recovery policy and remaining budget. Recovery choice should come from the same observation interface used during evaluation.
5. If recovery is unsupported or the monitor requires stopping, enter the commissioned stop/assistance procedure. A stationary hold is not assumed safe under sustained contact. Count this as intervention; do not silently reset.

A 20 Hz supervisor with 16 actions, 32 particles, and 4 recovery options requires 2,048 small-model evaluations per decision. Batched inference may fit the 50 ms budget, but measure complete latency including sensing and communication. Use a reduced candidate set or slower execution when deadlines are missed. Do not promise online Isaac rollouts at this rate without measurement.

**Optional principled result, with strict scope.** For a reduced contact model, assume bounded disturbances, a set-valued state estimator that contains the true state, valid reachable-tube bounds, a clear set invariant under its holding controller, and a finite-time backup policy valid for the whole uncertainty set. If each accepted segment has its full reachable tube in S and its endpoint uncertainty set has a validated backup to G_clear, then constraints are preserved during executed segments and a backup remains available at each decision point. A dropout invokes the already validated backup from its specified decision boundary. This follows by induction and backup feasibility; it is an established safety-filter argument, not the novel theorem by itself [7,8].

Continuous interruption requires stronger checks: the segment's intermediate uncertainty sets must also have backups, or execution must be guaranteed to reach the validated endpoint safely. A terminal-only check does not establish recoverability at every instant. Learned probabilities do not satisfy these robust assumptions by default. A useful theoretical contribution would be a tractable contact-specific bound or conservative set approximation, validated against a reduced-model oracle and measured data.

Per-decision marginal calibration does not yield an episode guarantee under adaptive replanning. A union bound of N(alpha+delta) needs conditional bounds valid after every observed history and a bounded decision count; it becomes weak quickly. Prefer honest empirical episode-level evidence unless those assumptions can actually be established.

## 7. Impedance control and real-time execution

Use Cartesian impedance as the initial low-level interface. It exposes equilibrium pose and compliance directly, works naturally with small motion primitives, and avoids needing demonstrations before experiments. The official libfranka example is a useful reference for the spring-damper structure [14].

For a small-angle, frame-consistent pose error e = [p-p_d; Log(R R_d^T)], twist error nu-nu_d, positive stiffness K, and damping D:

$$\tau_{sim}=c(q,\dot q)+g(q)+J^T[-Ke-D(\nu-\nu_d)+w_{ff}]+N^T\tau_{null}.$$

This is a quasi-static impedance form without full inertia shaping. In libfranka's external torque interface, gravity/friction compensation is handled by the robot; send the appropriate external torque command with Coriolis compensation according to that API rather than adding gravity again [14,15]. In a simulator with gravity disabled for the arm, adding a gravity term also double-compensates. Match the actual gravity and actuator settings explicitly.

The proposed rate structure is 1 kHz for the real low-level servo (FCI supports this [15]), 50-100 Hz for smoothly interpolated primitive references, and 10-20 Hz for planning/belief updates. Start simulation at Factory's native settings, then run timestep/solver convergence checks before changing it. Higher simulated physics frequency is not itself evidence of better contact fidelity.

Initial simulation-only tuning ranges: lateral translational stiffness 100-300 N/m, axial 50-150 N/m, and rotational 2-10 N m/rad. Begin with fixed gains and bounded reference motion. For a diagonal effective inertia approximation, D_i = 2 zeta sqrt(K_i Lambda_i), with damping ratio near one. Reassess inertia and coupled behavior; D = 2 sqrt(K) implicitly uses a unit-inertia approximation and is not dimensionally universal.

Two separate effects must be controlled: a small stiffness with a large equilibrium offset still produces a large commanded force; and a clipped commanded wrench does not bound actual contact-force transients. Limit displacement/rotation, speed, acceleration, torque and torque rate, and measure contact wrench continuously. Use a soft trigger below operational limits with a margin based on sensor uncertainty and measured latency/overshoot. All baselines share this monitor.

Keep variable stiffness out of the first result. Switching gains or equilibria can inject energy. If variable compliance becomes a contribution, implement smooth bounded changes and examine a passivity/energy-tank treatment; do not claim fixed-gain passivity automatically carries over to learned gain changes.

If the available hardware interface only accepts pose or velocity, an admittance outer loop is an alternative: M_a dnu_c/dt + D_a nu_c = w_des - w_meas, with a clearly defined wrench sign and a bounded integrated reference. Do not add independent impedance and admittance loops without analyzing the combined response.

## 8. Software and ready-made scene

**Primary choice: Isaac Lab Factory on Isaac Sim.** Use a compatible pinned pair, initially Isaac Lab v2.3.0 with Isaac Sim 5.1, whose version family is listed in the repository compatibility table [10]. This is a reproducibility target, not a claim to be the newest release. If the lab already has a validated installation, use it and pin its exact commit, simulator build, assets, driver, and GPU. The current default repository branch has moved toward the 3.0 beta family; avoid mixing instructions across versions [10].

| Repository / source | Use | Integration decision |
|---|---|---|
| isaac-sim/IsaacLab [10-12] | Factory Franka, insertion objects, batched simulation, task/controller code. | Main simulator and data-generation base. Start from Isaac-Factory-PegInsert-Direct-v0. |
| frankarobotics/libfranka [14,15] | Official Franka low-level interface and impedance example. | Preferred real controller base; select a release compatible with the actual Panda or FR3 firmware. |
| frankarobotics/franka_ros2 [16] | ROS 2 integration and hardware interfaces. | Use if the lab already operates ROS 2. Match the ROS branch to the host OS; do not assume its current default is Humble. |
| popnut123/TacDiffusion [13] | Official model training/inference and controller integration. | Optional published nominal policy; it is not a ready-made Isaac Lab recovery scene. |
| abalakrishna123/recovery-rl [17] | Reference task/recovery architecture. | Conceptual/reimplementation baseline; not a drop-in Franka/Isaac task. |

The FORGE project page currently labels its code link as coming soon [2b]. Factory contains related machinery, but that does not establish a complete faithful FORGE reproduction. Label a reimplementation or FORGE-inspired baseline accurately. BILBA is a close planning comparison; verify its implementation availability and compatibility in Week 1 before committing the schedule to it.

Useful Factory files under source/isaaclab_tasks/isaaclab_tasks/direct/factory/: factory_env.py, factory_env_cfg.py, factory_tasks_cfg.py, and factory_control.py. Inspect the version-pinned source, not only task names.

**Code-inspection findings that matter:**

- The v2.3.0 task registration includes Isaac-Factory-PegInsert-Direct-v0 [11]. Its stock config uses 120 Hz physics and decimation 8, i.e. 15 Hz action updates [12].
- Its configured peg diameter is 7.986 mm and hole diameter 8.100 mm: 0.114 mm diametral clearance, or 0.057 mm radial clearance [12]. Inspect collision geometry and measure the physical parts before treating these as realized tolerances.
- _apply_action fixes roll and pitch to an upright orientation. Remove that restriction in a separate project environment before evaluating tilt correction [12].
- Its action target is recomputed from the current pose during physics substeps. A normalized action is therefore not a single accumulated Cartesian displacement. Implement explicit start-anchored and rate-limited reference trajectories for the proposed primitive semantics [12].
- Default policy observations do not include measured external 6D wrench. The variable applied_wrench is a controller output, not a force/torque sensor measurement. Add and validate the intended sensor path [12].
- The stock assets/control include gravity-disabled bodies and simulator-oriented limits. Audit those assumptions before transfer. Do not copy simulator limits into a real controller [12].

The accompanying launcher opens this existing scene and holds it with zero policy actions. It records diagnostic TCP and joint state. It is intentionally only a scene/integration starter; the learned predictor, compliant insertion primitives, wrench sensor, and hardware controller are subsequent work. Its source/API usage was checked and its Python syntax validated here; Isaac Sim and Franka hardware were not available for execution.

## 9. Scene design and sim-to-real protocol

Scene A is the stock cylindrical peg/hole for initial bring-up. Scene B should be a rectangular peg or a fixture with different insertion length/chamfer, chosen to create a meaningfully different contact/recovery geometry. For an RSS-track argument, multiple clearances of one cylindrical asset alone are a weak generalization test.

Use a rigid, replaceable fixture, a pregrasped part, and a reproducible clear pose. Manufacture or measure at least three clearance settings if feasible. Pilot ranges are lateral misalignment +/-0.5-2 mm, tilt +/-0.5-3 degrees, friction coefficients 0.15-0.6 in simulation, and clearance chosen around measured task tolerance. These ranges are hypotheses for finding informative states, not validated contact regimes or suggested hardware force limits.

Build a true concave hole using suitable collision geometry: SDF where supported and validated, or multiple convex components. A single convex hull over the full fixture can close the hole. Changing a metadata diameter or uniformly scaling both parts does not create an independently controlled clearance. Keep rendering meshes and collision meshes distinguishable.

Validate the simulator using free motion, single-wall contact, insertion, and withdrawal traces. Vary timestep, solver iterations, collision approximation, contact/rest offsets, and friction. Check force peaks and the *ordering* of easy versus difficult withdrawals, not only insertion success. A simulated jam that disappears with a small solver change is not reliable training evidence. The rigid simulator also cannot represent every elastic lock, burr, or wear effect.

Wrench pathway: prefer an external wrist F/T sensor if already available; otherwise assess Franka's estimated external wrench with known loads and unloaded motion before deciding it is adequate. Subtract sensor/tool bias and account for payload gravity and dynamics. Transform moments to the same origin using the lever arm; rotate-only transforms are insufficient. Log raw and filtered signals plus timestamps.

In Isaac Lab, a basic contact-sensor net-force tensor is not a calibrated wrist 6-axis sensor. Confirm whether the selected API includes normal and tangential forces and whether moments are available [19]. For synthesized contact wrench use F = sum f_i and M = sum((r_i-r_TCP) cross f_i + m_i), including friction where the API exposes it. Alternatively use an instrumented constraint/reaction sensor, subtract payload dynamics, and validate signs. Do not silently substitute commanded wrench.

Match simulation to real through controlled measurements: TCP/grasp transform; fixture alignment; free-space tracking and delay; wall push/unload; contact stiffness and damping; and shallow withdrawal at measured offsets. Fit a parameter distribution rather than one exact friction number. Randomize delay, bias, noise, payload/grasp error, and identified contact parameters jointly where measurements indicate correlation.

Collect a separate small real calibration set, initially 50-100 bounded trials spread across the intended operating region, subject to available lab time. Freeze recalibration before final experiments. Keep a later day or fixture subset entirely held out. Report zero-shot transfer and real-calibrated transfer separately; do not call fine-tuned behavior zero-shot.

## 10. Experimental design and evidence for the paper

Use the same nominal controller, observations, operational limits, time budget, retry budget, success verifier, and monitor across methods. A practical episode budget is one initial attempt plus two retries, with an identical wall-clock timeout; choose its exact duration after the pilot and lock it.

| Method | Behavior | What the comparison isolates |
|---|---|---|
| B0: nominal skill | Compliant insertion with the shared monitor; no planned recovery/retry. | Basic insertion ability. |
| B1: reactive recovery | Force/stall detection then fixed unload-retreat-retry. | Prediction versus a practical reactive system. |
| B2: progress planner | Same candidates, belief and current/path force checks; no future recovery criterion. | Incremental value of preserving recovery. |
| B3: reactive learned selector | Use the same learned recovery model only after a force/stall trigger. | Anticipation versus simply choosing a better recovery maneuver. |
| Full method | Candidate-conditioned withdrawal prediction and uncertainty-aware filtering. | Proposed system. |
| Published comparator | Faithful FORGE/TacDiffusion or compatible BILBA reproduction, selected in Week 1. | Position against external work rather than only self-defined ablations. |

If a published system cannot be faithfully ported, document which elements are reproduced and which differ. An inspired proxy is not the full published algorithm. Under a strict 8-week budget, use B0-B3 plus the full method and explicitly reserve the external comparison for the paper extension.

Primary endpoint: episodes requiring manual intervention, under fixed completion time and retry limits. Co-primary task endpoint: assembly completion. Recovery rate is conditional on recovery being attempted; report how many attempts there were, because a preventive method changes their frequency. Secondary endpoints: recoveries completed within budget, peak force and moment, limit violations and duration, total time including unsuccessful attempts, retries, and damage observations under a prespecified inspection protocol.

The decisive experiment is a matched-state counterfactual: find prefixes with similar depth/current wrench but different tilt/contact history, test candidate insertion actions, and replay recovery outcomes. Demonstrate that the future feasibility estimate discriminates them beyond current force, depth, and insertion-success prediction. Also compare a deliberately slower, more conservative baseline; otherwise improved recovery may just reflect less progress.

Simulation budget: begin with at least 100 held-out scenarios per method per difficulty regime and three training seeds for learned methods. For a broad benchmark, 2 geometries x 3 difficulty regimes x 100 scenarios x 5 core methods = 3,000 evaluation episodes for one trained model; evaluate seed variability separately and do not confuse seed count with independent physical trials. Report performance by geometry and regime, not just pooled averages.

Real budget: pilot 10 trials per method to estimate variance and detect broken baselines. A feasible target is 2 geometries x 3 regimes x 15-20 trials x 5 methods = 450-600 episodes, plus the external comparator if integrated. At an illustrative 2 minutes per episode including reset, that is 15-20 hours of active testing before calibration, repairs, and setup. If time is limited, first run the full method versus the strongest reactive baseline, then expand; reduce claim scope when evidence is smaller.

Randomize method order within blocks and account for drift, wear, temperature, and sensor bias. Real matched conditions are repeatable fixture/offset blocks, not exact cloned contact states. Bootstrap by independent episode or fixture/session block, never by overlapping time windows. Use binomial intervals for rates and paired/block bootstrap intervals for method differences. Choose final trial counts using pilot effect sizes; 15-20 trials per cell are a planning budget, not a claim of adequate statistical power for small effects.

Essential ablations: no future-recovery term; no belief/uncertainty; no history; one recovery maneuver; uncalibrated score; same model used reactively; and a depth/current-wrench heuristic. Make the recovery-versus-completion/time Pareto curve a main figure. An oracle with simulator state is an upper-bound diagnostic only and must be labeled privileged.

## 11. Twelve-week execution plan

| Week / dates | Build and investigate | Measurable exit deliverable |
|---|---|---|
| 1 / Sep 14-20 | Freeze scope and hypotheses. Inspect closest papers and comparator availability. Pin simulator/robot stack. Measure parts and verify FCI/sensor access. Order second fixture. | Factory scene opens; version manifest and experiment specification; selected published comparator. |
| 2 / Sep 21-27 | Establish Cartesian impedance, wrench logging and transforms, clear/seated checks. Perform gentle real calibration and free-space tracking. | Nominal insertion/withdrawal traces in sim; real sensor/servo bring-up; no unexplained signal signs. |
| 3 / Sep 28-Oct 4 | Add bounded tilt and start-anchored primitives; build B0/B1. Map jam/withdrawal behavior versus offset/tilt/friction. Test solver sensitivity. | At least 100 simulated pilot trials; evidence that withdrawal outcomes vary beyond a simple current-force threshold. |
| 4 / Oct 5-11 | Implement prefix replay and outcome labels; collect first 8,000 recovery rollouts where feasible. Train simple predictors. Continue weekly hardware checks. | Versioned dataset, grouped splits, prediction/false-safe report; reproducible successful and failed recovery cases. |
| 5 / Oct 12-18 | Train ensemble/action-conditioned heads. Calibrate held-out predictions. Add belief/history features and OOD handling. | Predictor beats current-force/depth baseline at matched acceptance coverage, or a documented pivot. |
| 6 / Oct 19-25 | Integrate one-step recovery-aware planner, B2/B3, and common logging. Measure latency and add deadline handling. | Closed-loop sim ablation: lower intervention at comparable completion/time; bounded execution latency. |
| 7 / Oct 26-Nov 1 | Calibrate dynamics transfer and validate shallow real recovery. Fit only the designated calibration set. Bring up geometry B. | Real withdrawal predictions and traces; frozen real calibration; geometry B working. |
| 8 / Nov 2-8 | Run first locked real comparison against strongest reactive baseline. Produce plots and complete initial draft sections. | Eight-week prototype, comparative evidence, video, and go/no-go decision for the RSS extension. |
| 9 / Nov 9-15 | Finish external comparator and difficult/generalization settings. Tighten reduced-model argument if supported. | Closest-method comparison; model assumptions and failure taxonomy; no new unrelated features. |
| 10 / Nov 16-22 | Run randomized full real campaign across geometries/regimes; complete simulation seeds and tradeoff sweeps. | Locked results tables with uncertainty intervals and complete intervention accounting. |
| 11 / Nov 23-29 | Write results/limitations and finalize main figures. Reproduce the headline result from a fresh checkout; obtain advisor review. | Complete manuscript and supplement draft; reproducible commands and dataset manifest. |
| 12 / Nov 30-Dec 6 | Resolve concrete reviewer-style objections, rerun only necessary checks, finalize video and release package. Verify actual RSS call. | Submission-ready research package, subject to results and venue requirements. |

Weekly working rhythm: review one hypothesis on Monday, spend the central days on the current critical dependency, run one measured comparison on Thursday, and update figures plus the manuscript on Friday. Reserve hardware sessions from Week 2 onward rather than postponing real work until the end.

If limited to eight weeks, keep one core geometry plus a second fixture variant, one-step planning, fixed impedance gains, a four-policy recovery library, and five core methods. Defer diffusion/RL training, full contingent search, variable stiffness, regrasping, and perception. Produce a credible pilot paper draft and state what evidence is still missing for the RSS ambition.

## 12. Gates, risks, and first five days

**Week 3 gate:** Does a meaningful future-recovery problem exist in the chosen geometry? If all budget-respecting insertions trivially withdraw, try measured changes in engagement length, clearance or tilt, and verify simulator fidelity. Do not create an artificial reward penalty just to make the method win. If no informative boundary appears, pivot to uncertainty-aware force prediction or broaden the task with advisor agreement.

**Week 6 gate:** Does the full method outperform B2 and the reactive learned selector at comparable completion/time? If it only inserts less often or moves more slowly, the central claim is unsupported. Refine the representation or reduce the claim instead of adding unrelated neural components.

**Week 8 gate:** Do real predictions and policy rankings transfer? If simulator jam orderings disagree with measured withdrawals, prioritize contact identification, geometry correction and honest real-data calibration. A high simulated success rate cannot substitute for this check.

**RSS gate:** Aim for a clear mechanism/algorithmic insight, at least two informative geometries, strong published and reactive comparisons, calibrated prediction analysis, and physical evidence. A generic combination plus a standard induction proof is unlikely to establish the intended contribution on its own. If results are narrower, keep the complete system and pursue an appropriately scoped venue or workshop paper.

Day 1: inventory Panda versus FR3, firmware/FCI/libfranka, GPU and simulator versions, F/T sensor, gripper/peg mount, and fixture dimensions. Freeze the operational definitions and book hardware time.

Day 2: run the existing Factory task with zero actions using the included launcher; inspect asset scale, clearances, frames, and initial grip. Commit a version manifest in the project's own repository.

Day 3: add real wrench acquisition/compensation and simulator wrench instrumentation in a separate environment module. Validate with unloaded motion and known-direction contact.

Day 4: implement approach, unload and retreat with fixed gains; verify controlled seating and clear-pose termination in easy conditions.

Day 5: collect a small offset/tilt pilot and plot current wrench versus eventual recovery outcome. Use the plot to choose the Week 3 regime and quantify whether the central distinction is observable.

## References and inspected implementation sources

All links were checked on 11 September 2026. GitHub source/API inspection does not imply that the software was executed here.

- [1] Wu et al. TacDiffusion: Force-domain Diffusion Policy for Precise Tactile Manipulation. ICRA 2025. https://arxiv.org/abs/2409.11047
- [2] Noseworthy et al. FORGE: Force-Guided Exploration for Robust Contact-Rich Manipulation under Uncertainty. Revised 2025. https://arxiv.org/abs/2408.04587
- [2b] FORGE project page and code availability. https://noseworm.github.io/forge/
- [3] Chintalapudi, Kaelbling, Lozano-Perez. Bi-Level Belief Space Search for Compliant Part Mating Under Uncertainty. 2024. https://arxiv.org/abs/2409.15774
- [4] Thananjeyan et al. Recovery RL: Safe Reinforcement Learning with Learned Recovery Zones. 2021. https://arxiv.org/abs/2010.15920
- [5] Curtis et al. Partially Observable Task and Motion Planning with Uncertainty and Risk Awareness. RSS 2024. https://arxiv.org/abs/2403.10454
- [6] Curtis et al. Flow-based Domain Randomization for Learning and Sequencing Robotic Skills. 2025. https://arxiv.org/abs/2502.01800
- [7] Chen et al. Backup Control Barrier Functions: Formulation and Comparative Study. 2021. https://arxiv.org/abs/2104.11332
- [8] Wabersich and Zeilinger. A predictive safety filter for learning-based control of constrained nonlinear dynamical systems. https://arxiv.org/abs/1812.05506
- [9] Rah and Oh. FORGE-plus: Force-Budgeted Recovery for Contact-Rich Assembly with a Frozen LLM Supervisor. July 2026 preprint. https://arxiv.org/abs/2607.21227
- [10] Isaac Lab repository and version compatibility. https://github.com/isaac-sim/IsaacLab
- [11] Factory task registration, v2.3.0. https://github.com/isaac-sim/IsaacLab/blob/v2.3.0/source/isaaclab_tasks/isaaclab_tasks/direct/factory/__init__.py
- [12] Factory implementation, v2.3.0: https://github.com/isaac-sim/IsaacLab/tree/v2.3.0/source/isaaclab_tasks/isaaclab_tasks/direct/factory
- [13] Official TacDiffusion repository. https://github.com/popnut123/TacDiffusion
- [14] libfranka Cartesian impedance example. https://frankarobotics.github.io/libfranka/latest/cartesian_impedance_control_8cpp-example.html
- [15] Franka Control Interface overview. https://frankarobotics.github.io/docs/overview.html
- [16] Franka ROS 2. https://github.com/frankarobotics/franka_ros2
- [17] Recovery RL repository. https://github.com/abalakrishna123/recovery-rl
- [18] RSS official conference site. https://roboticsconference.org/
- [19] Isaac Lab contact-sensor data documentation. https://isaac-sim.github.io/IsaacLab/main/_modules/isaaclab/sensors/contact_sensor/contact_sensor_data.html
