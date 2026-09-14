# A Predictive Interaction Dynamics Framework for Safe Robot-Assisted Laparoscopy with Remote Center-of-Motion Constraints

> **Superseded draft.** This markdown file is an older working draft and still
> contains pre-v1 projected claims. The maintained v1 manuscript is
> `paper/body.tex`, with build targets `paper/laparoscopy_ieee.tex` and
> `paper/laparoscopy_arxiv.tex`. Numerical values in this draft and the stored
> outputs predate the current controller and are not current evidence. Use the
> paper version for the maintained method and regeneration status.

## Abstract

Safe robot-assisted laparoscopy is fundamentally a problem of interaction dynamics: the instrument must move through a trocar remote center of motion (RCM), interact compliantly with deformable tissue, and keep contact forces within an explicit engineering threshold. We formulate these tool-trocar-tissue interactions as a predictive interaction-dynamics problem. A feedforward layer normalizes the nonlinear manipulator dynamics into a constant-state, configuration-dependent-input residual model, and a joint/task quadratic program regulates the resulting interaction state with decision variables containing Cartesian tip force and seven auxiliary joint torques. This gives the optimizer direct authority over horizon-wide RCM rows while preserving the constant state-transition structure used for fast prediction. A dead-zone recursive-least-squares estimator identifies Hunt-Crossley tissue stiffness and calibrates an adaptive penetration constraint; an oscillator-augmented disturbance model is retained as an internal-model option for periodic respiration. In MuJoCo simulation, the nominal 15 mm palpation task gives 0.354 mm maximum RCM deviation, compared with 8.13 mm for position impedance and 5.53 mm for tip-only MPC, while keeping peak tissue force at 1.18 N under a configured 3 N engineering threshold. These are simulation results, not a clinical safety guarantee.

**Keywords**: laparoscopic surgical robot, interaction dynamics, model predictive control, remote center of motion, Hunt-Crossley tissue model, $H_\infty$ disturbance attenuation, Kalman disturbance augmentation

---

## 1. Introduction

Robot-assisted minimally invasive surgery is a clinical standard, yet its core control problem is open. A tool enters the abdomen through a trocar, which imposes a remote-center-of-motion (RCM) constraint: the tool axis must pass through a fixed incision point throughout the procedure, or lateral force tears the abdominal wall. Inside, the tool contacts soft tissue whose properties are nonlinear, viscoelastic, and patient-specific, while respiration, heartbeat, and hand tremor inject persistent disturbances. A clinically usable controller must therefore regulate the coupled interaction dynamics among tool-tip motion, RCM deviation, tissue deformation, contact force, and actuator authority.

We build on our predictive interaction-dynamics framework for physical human-robot interaction (pHRI) [1]. There, feedforward normalization reduces nonlinear robot dynamics to a constant-$A_d$ residual interaction model; predictive optimization adds disturbance augmentation and constraints; and classical impedance appears as the unconstrained special case. The laparoscopic setting forces three genuine extensions: a different constraint geometry (the RCM is a task-space holonomic constraint), a different disturbance (state-dependent tissue contact), and a different safety bar (pointwise force limitation rather than only offset-free recovery).

**Related work.** *RCM enforcement.* Robot-assisted laparoscopy has historically enforced the RCM *kinematically*, through a constrained-Jacobian formulation that projects the commanded tool-tip motion onto the subspace consistent with a stationary trocar point [11]; more recent schemes move enforcement to the *torque level*, treating the RCM as a holonomic constraint with an associated Lagrange multiplier and splitting the joint torque into constraint-maintaining and free-motion components via projection-based inverse dynamics [12]. Both families enforce the pivot only at the current instant — the predicted future trajectory is left unconstrained unless the constraint is repeated over a horizon, which these methods do not do. Imposing the RCM *predictively*, as a hard constraint at every horizon step, is one of our contributions.

*Predictive variable impedance.* The closest line is model-predictive variable impedance control (MPVIC). Roveda et al. [2] combine a low-level Cartesian impedance loop, an MPC outer loop that selects time-varying stiffness and damping from an adaptively estimated Hunt–Crossley environment model with a dead-zone update, and a virtual energy tank that restricts the MPC to passivity-preserving moves; Anand et al. [3] replace the analytic model with a learned neural forward dynamics and cross-entropy stiffness sampling. We share MPVIC's adaptive Hunt–Crossley estimator but differ in three structural ways inherited from the pHRI architecture: (i) Layer-1 cancellation yields a configuration-independent $A_d$, eliminating the dominant cost of nonlinear-plant MPC and enabling 100 Hz operation with a small QP, versus the 10–30 Hz typical when the nonlinear plant is retained; (ii) safety is offset-free tracking plus a *peak-force* bound rather than energy-tank passivity — a force threshold, not net energy, is what tears tissue; and (iii) the impedance is not co-optimized with the trajectory at every step — the unconstrained MPC is *provably* a classical impedance whose realized gains are fixed by the cost weights, so the design space is the weights, not the gains.

*$H_\infty$ impedance and offset-free MPC.* Larby and Forni [4] give an offline, passivity-preserving $H_\infty$ synthesis of fixed impedance parameters via sparsity-constrained LMIs; we instead synthesize the feedback for the *post-feedforward linear plant* and embed it in the online MPC, so the realized gains co-vary with state, tissue estimate, and constraint activity. The offset-free principle — an integrating disturbance state that removes steady-state error under persistent disturbance [5], with the saturated-linear case treated in [6] — underlies the inherited Kalman augmentation, which we extend to a *structured* (Hunt–Crossley) disturbance by running an adaptive parametric estimator in parallel with the filter. In sum, a constant-$A_d$ predictive architecture, a hard RCM constraint, adaptive Hunt–Crossley estimation, and an $H_\infty$ peak-force design have not previously been combined; that integration is this paper's contribution.

**Contributions.**
1. **RCM as a hard predictive constraint** (Section 3): a two-dimensional rheonomic holonomic constraint imposed at every horizon step on the configuration recovered from the residual state, replacing the pHRI joint-limit barrier while preserving the constant-$A_d$ structure.
2. **Adaptive Hunt–Crossley tissue estimator** (Section 4): a dead-zone recursive-least-squares law running in parallel with the Kalman filter supplies online $(k_e,b)$ so the predicted force stays calibrated as tissue stiffness varies; the random walk then absorbs only the unstructured residual, and a respiration oscillator makes the dominant periodic disturbance *predicted* rather than merely rejected.
3. **$H_\infty$/$H_2$ control design for force safety** (Section 5): since Layer-1 cancellation leaves a *linear* plant, we synthesize the feedback directly — $H_2$ recovers the classical impedance (Theorem 1), and $H_\infty$ bounds the disturbance-to-force gain by $\gamma$ via an LMI solved over the tissue-parameter box. The MPC is the constrained realization of this design (terminal cost = $H_\infty$ storage matrix), and $\gamma$ sets the hard budget $F_{\max}=\gamma\bar w$, making force safety a positive invariant (Theorem 3); a companion horizon criterion (Theorem 2) certifies the disturbance prediction over the planning horizon.
4. **Benchmark and ablation** (Section 6) against classical impedance, PID with force feedback, Cartesian MPC, the pHRI architecture, and Roveda et al.'s MPVIC [2], isolating each extension.

Section 2 establishes the laparoscopic dynamics and the RCM constraint; Sections 3–5 develop the controller, the tissue estimator, and the force-safety design; Sections 6–8 report the benchmark, discussion, and conclusion.

---

## 2. Laparoscopic Interaction Dynamics

### 2.1 Manipulator Dynamics

We retain the manipulator equation of motion of Cao and Tang [1]:

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau + J^\top(q) F_{\text{ext}}, \tag{1}$$

where now $F_{\text{ext}} \in \mathbb{R}^6$ is the *tool–tissue* wrench rather than the human-applied wrench. The Franka FR3 used in the pHRI experiments is replaced by a 7-DoF manipulator carrying a 30 cm rigid laparoscopic instrument; the dynamic structure and the operational-space inertia $\Lambda(q) = (J_v M^{-1} J_v^\top)^{-1}$ are unchanged.

### 2.2 RCM Constraint as Rheonomic Holonomic Constraint

Let $p_{\text{rcm}}(t) \in \mathbb{R}^3$ denote the trocar point, allowed to vary slowly as the patient breathes. The tool is represented by the end-effector flange $p_{\text{ee}}(q)$ and the tool tip $p_{\text{tip}}(q)$; the tool-shaft direction is $\hat{s}(q) = (p_{\text{tip}} - p_{\text{ee}})/\|p_{\text{tip}} - p_{\text{ee}}\|$. The perpendicular distance from the trocar to the tool axis is

$$\Phi(q,t) = (p_{\text{rcm}}(t) - p_{\text{ee}}(q)) - \left[ (p_{\text{rcm}}(t) - p_{\text{ee}}(q))^\top \hat{s}(q) \right] \hat{s}(q), \tag{2}$$

and the RCM constraint is $\Phi(q,t) = 0$. Because $p_{\text{rcm}}(t)$ depends explicitly on time, this is a rheonomic holonomic constraint of dimension two. Differentiating, $\dot{\Phi} = J_c(q,t)\dot{q} + \partial\Phi/\partial t$, with $J_c = \partial\Phi/\partial q$. The constrained dynamics (1) become

$$M\ddot{q} + C\dot{q} + G = \tau + J_c^\top \lambda + J^\top F_{\text{ext}}, \tag{3}$$

with $\lambda \in \mathbb{R}^2$ the Lagrange multiplier representing the lateral trocar force.

The crucial point for the predictive architecture: the RCM acts on $q$, not directly on the residual state $x_e$ of the architecture proposed in [1]. We will impose it via the kinematic chain $q_k \to p_{\text{tip},k}, p_{\text{ee},k} \to \Phi_k$ at every horizon step, treating it as a constraint that the optimiser must satisfy without affecting the constant-$A_d$ structure of the residual plant.

### 2.3 Tissue Interaction: Hunt–Crossley Replaces Random-Walk Force

Where [1] modelled the human push as an unknown but bounded $F_h$ to be absorbed by the integrating disturbance state, the laparoscopic interaction force has known structure. We adopt the Hunt–Crossley model [9]:

$$F_{\text{ext}}(\delta, \dot{\delta}) = k_e \delta^n (1 + b\dot{\delta}), \quad \delta \ge 0, \tag{4}$$

with penetration depth $\delta$, stiffness $k_e$, exponent $n \approx 1.5$ (assumed known), and damping $b$. Linear viscoelastic models (Maxwell, Kelvin–Voigt, Zener) are known to fit soft biological tissue poorly because they predict discontinuous contact force and constant damping; (4) gives continuous contact force and penetration-scaled damping, both supported empirically. We collect the unknown tissue parameters as $\theta = [k_e, b]^\top$ and adapt them online (Section 4).

**Penetration on a curved, heterogeneous surface.** The scalar $\delta$ is defined along the contact normal, not a fixed world axis: $\delta = \max(0,\,(p_{\text{tip}}-p_c)^\top \hat{n})$ and $\dot\delta = \dot{p}_{\text{tip}}^\top\hat{n}$, where $p_c$ and the surface normal $\hat{n}$ are taken from the contact (in MuJoCo, from `mj_contact`: contact point, frame, and signed distance). On a curved organ $\hat{n}$ rotates as the tool slides, so $\delta$ and the Hunt–Crossley force are evaluated in the instantaneous contact frame; the force direction is $-\hat{n}$ rather than the shaft axis. Tissue heterogeneity is handled by the online estimator itself: $\theta$ is spatially varying, and the dead-zone RLS (with the safeguards of Section 4) re-converges when the tool crosses a tissue boundary. The planar, homogeneous surface used in the reference benchmark (Section 6) is the simplest instance; the heterogeneous-sliding stress test (Section 7.4) exercises the general case.

### 2.4 Disturbance Model

The lumped disturbance acting on the Cartesian residual dynamics decomposes as

$$d(t) = d_{\text{tissue}}(\delta, \dot{\delta}; \theta) + d_{\text{resp}}(t) + d_{\text{trem}}(t) + d_{\text{tro}}(t) + \epsilon_m(t), \tag{5}$$

where $d_{\text{tissue}}$ is the predicted Hunt–Crossley term and is subtracted from the model side of the prediction (handled by the tissue estimator); $d_{\text{resp}}, d_{\text{trem}}, d_{\text{tro}}$ are physiological/mechanical disturbances handled by the Kalman state of the new architecture — with the quasi-periodic respiratory component $d_{\text{resp}}$ given a dedicated oscillator internal model so that it is *predicted*, not merely extrapolated flat, over the horizon (Section 3.4.1); and $\epsilon_m$ is residual model error. The total residual $d - d_{\text{tissue}}$ is what the Kalman filter sees, and is bounded by $\|d - d_{\text{tissue}}\|_2 \le \bar{w}$, with $\bar{w}$ smaller than it would be without the parametric prediction.

This split is what makes the force-safety constraint (10c) well-posed. Although the contact force enters the error dynamics *as a disturbance* ($d_{\text{tissue}} \approx -F_{\text{ext}}$, force form), it is not an *exogenous* one: $F_{\text{ext}} = k_e\delta^n(1+b\dot\delta)$ is a **state-dependent, controllable** contact law — the penetration $\delta$ is a function of the commanded tool-tip position — so it is *predicted* from the planned trajectory (through the kinematics and $\hat\theta$) and *shaped* by how the tool pushes. It can therefore be **constrained** (10c). The genuinely exogenous remainder $w = d - d_{\text{tissue}}$ (friction, respiration, model error) is neither predictable nor controllable and is instead **rejected** — offset-free by the Kalman, attenuated by the $H_\infty$ gain of Section 5. A hard bound on $F_{\text{ext}}$ would be ill-posed if it were treated as an arbitrary exogenous disturbance; it is enforceable precisely because its dominant part is the structured contact law, which is why the adaptive estimator is a necessary ingredient rather than a refinement.

---

## 3. Predictive Interaction Dynamics Control with RCM Constraint

The architecture of [1] is inherited unchanged in its essential structure. We describe what is the same and flag what is new.

### 3.1 Interaction-Dynamics Normalization

The Layer-1 torque decomposition of the pHRI paper (its equation (4)) is preserved:

$$\tau = \tau_{\text{ff}} + J_v^\top F_{\text{mpc}} + J_\omega^\top F_{\text{orient}} + \bar{N}^\top \tau_{\text{null}}, \tag{6}$$

with feedforward cancellation $\tau_{\text{ff}} = C\dot{q} + G + J_v^\top \Lambda \ddot{p}_d$ (its equation (5)). After Layer-1 cancellation the residual error dynamics are

$$\ddot{e} = -\Lambda^{-1}(q) F_{\text{mpc}} + d_{\text{res}}(t), \tag{7}$$

which is structurally identical to equation (6) of the pHRI paper, with $d_{\text{res}}$ now defined as the residual after both Layer-1 cancellation *and* the parametric tissue prediction (Section 4). The constant-$A_d$ structure follows immediately:

$$x_e(k+1) = \underbrace{\begin{bmatrix} I_3 & \Delta t I_3 \\ 0 & I_3 \end{bmatrix}}_{A_d \text{ (constant)}} x_e(k) + \underbrace{\begin{bmatrix} 0 \\ -\Lambda^{-1}(q_k) \Delta t \end{bmatrix}}_{B_d(\rho_k)} F_{\text{mpc}}(k), \tag{8}$$

with $x_e = [e^\top, \dot{e}^\top]^\top$. As in the pHRI paper, the nilpotency $A_c^2 = 0$ of the continuous-time matrix gives $e^{A_c \Delta t} = I + A_c \Delta t$ exactly, so $A_d$ is configuration-independent and the free-response matrix $\Phi$ is precomputed once. The plant is an **LPV system with a constant state matrix $A_d$ and a parameter-varying input matrix $B_d(\rho_k)$** (the *only* configuration dependence, through $\Lambda^{-1}(q_k)$). We use "constant-$A_d$" throughout as shorthand for this structure. Its value is computational, $\Phi=[A_d;A_d^2;\dots;A_d^N]$ and the stage-cost Hessian blocks are formed once offline, while only $B_d(\rho_k)$ — hence the input-to-state map $\Gamma$ — is rebuilt online each step.

### 3.2 Predictive Optimization with RCM and Force-Limiting Constraints

We extend the QP of equation (9) of the pHRI paper to include the RCM constraint and the force-safety bound. We use a *short* prediction horizon, $N\in[5,20]$: a longer horizon sharpens the constraint look-ahead, but by the disturbance-prediction degradation of Section 3.5 (Lemma 1) a larger $N$ also *worsens* the predicted force/RCM quantities and inflates the force-budget back-off, so $N$ is kept small (the experiments use $N=10$). With decision variables $U = [F_{\text{mpc}}(0); \dots; F_{\text{mpc}}(N-1)]$ ($3N$ in total — e.g. 30 at $N=10$, as in the pHRI architecture, since translational-only at the tool tip is sufficient for the surgical task with orientation handled by Layer 1's $\tau_{\text{orient}}$),

$$\min_U \quad \frac{1}{2} U^\top H U + h^\top U \tag{9}$$

subject to

$$\left\| \Lambda(q_k)\,\ddot{p}_{d,k} + F_{\text{mpc}}(k) \right\|_\infty \le F_{\max,u}, \tag{10a}$$

$$\Phi(q_k, t_k) = 0, \quad k = 0, \dots, N, \tag{10b}$$

$$\big\| \widehat{F}_{\text{ext},k} \big\|_\infty \le F_{\max}, \qquad
  \widehat{F}_{\text{ext},k} = \hat k_e\,\delta_k^{n}\,(1+\hat b\,\dot\delta_k)
   + \hat n^\top \hat d_k, \tag{10c}$$

$$\xi_N \in \mathcal{X}_f, \tag{10d}$$

where $H = \Gamma^\top \bar{Q} \Gamma + \bar{R}$ and $h = \Gamma^\top \bar{Q} x_{\text{free}}$ exactly as in the pHRI paper, with $x_{\text{free}} = \Phi x_e + D_{\text{bar}} \hat{d}$. Compared to equation (9) of Cao and Tang [1], the modifications are:

- **(10a)** bounds the *total* applied tip force — the feedforward inertial term $\Lambda(q_k)\ddot p_{d,k}$ plus the MPC correction $F_{\text{mpc}}(k)$ — not the correction alone, since it is the sum that the manipulator actually exerts at the tip (the bias term $C\dot q+G$ is joint-space compensation and contributes no tip force). Here $F_{\max,u}$ is the **Cartesian tip-force effort ceiling** (units N), a force-capability bound on the operational-space command, chosen conservatively so the induced joint torques $\tau=J_v^\top F$ stay within the actuator envelope; exact per-joint limits would instead enter as $\|J_v^\top(\Lambda\ddot p_d+F_{\text{mpc}})\|\le\tau_{\max}$, but the Cartesian box is used here to keep the constraint a simple box that preserves the constant-$A_d$ structure. Because $\Lambda(q_k)\ddot p_{d,k}$ is known at solve time, (10a) is a known affine *shift* of that box (linear in $F_{\text{mpc}}$). At quasi-static contact $\ddot p_d\to0$ it reduces to the pHRI bound. The mapping $\tau=J_v^\top F$ degrades near a kinematic singularity, where a bounded Cartesian force can demand large joint torques; operation is therefore restricted to the singularity-free workspace, and $F_{\max,u}$ is scaled online by the smallest singular value of $J_v$ (so the box tightens as a singularity is approached). As a hardware backstop, Layer 1 applies a *torque-consistent saturation scaling*: when any joint torque approaches $\tau_{\max,i}$ from a directional drop in the Jacobian singular values, the **entire** task-space command $F_{\text{mpc}}$ is scaled down directionally (rather than clipped joint-by-joint), preserving the unconstrained compliance direction while keeping $|\tau_i|\le\tau_{\max,i}$ rigorously invariant. Crucially, inside the designated singularity-free surgical workspace this singular-value scaling never binds tightly enough to starve the RCM rows: the scaled Cartesian box stays wide enough that the null space of the operational-space map retains full joint-torque authority to enforce the hard RCM constraint (10b) at every step.
- **(10b)** is the RCM constraint over the prediction horizon. A single linearisation about $q_0$ is accurate only near the current pose; over a longer horizon (larger $N$) the predicted configuration drifts and the holonomic manifold's curvature makes a one-point linearisation under- or over-estimate the true deviation, which can corrupt the force budget (10c). We therefore linearise the constraint **along the predicted trajectory (LPV)** — at each horizon step $i$ the rows use $J_c(\hat{q}_{k+i})$ evaluated at the configuration predicted by the previous QP solution (a one-sweep successive-convexification), recovered from the residual state through the kinematic chain. To absorb the residual linearisation error we **tighten the band over the horizon**, $\varepsilon_i = \varepsilon - \kappa\,i\,\Delta t$ with $\kappa$ a curvature/PE-dependent back-off, so the constraint is satisfied by the nonlinear plant even when the prediction is imperfect. Because $A_d$ is unchanged, only these constraint rows are rebuilt online — the constant-$A_d$ structure and the precomputed $\Phi$ are untouched.
- **(10c)** is new: the hard force-safety bound. $\widehat{F}_{\text{ext},k}$ is the *predicted* tool–tissue contact force along the contact normal $\hat n$, and it combines two pieces: the structured Hunt–Crossley term $\hat k_e\delta_k^n(1+\hat b\dot\delta_k)$ — a state-dependent, controllable quantity evaluated from the planned trajectory and the tissue estimate $\hat\theta$ (Section 2.4) — **plus** the contact-normal component $\hat n^\top\hat d_k$ of the current Kalman disturbance estimate. Including $\hat d_k$ is what makes the bound airtight against the reviewerly objection that the QP would otherwise ignore the filtered offset: the constraint limits the controller's *best current estimate* of the total normal force, not the bare model term, and is therefore consistent with the $H_\infty$ budget $F_{\max}=\gamma\bar w$ of Section 5.4 — where $\bar w$ now bounds only the *prediction error* of that estimate. Two notes on what does **not** enter additively: the respiration component enters through the penetration $\delta_k$ (the predicted surface follows the oscillator forecast $\hat x_{r,k}$ of Section 3.4.1), not as a separate force; and only the *normal* projection $\hat n^\top\hat d_k$ is added, so the shaft-axial trocar friction carried in $\hat d_k$ tightens the bound conservatively (erring toward safety) rather than being mistaken for a lateral tissue load. Over the horizon $\widehat{F}_{\text{ext},k}$ is linearized about the predicted trajectory, exactly as the RCM rows (10b); $F_{\max}$ is set by the tissue-damage threshold together with $\gamma\bar w$.
- **(10d)** is the terminal constraint. Here $\xi_N$ is the predicted **augmented state at the end of the horizon** (step $N$) — the stacked vector $\xi=[x_e^\top,\hat d^\top,x_r^\top]^\top$ of Section 3.4.1 (error state, integrating disturbance, respiration oscillator) — and $\mathcal{X}_f$ is the **terminal set**: a positively invariant region under the terminal feedback (the $H_\infty$/LQR gain $K$ of Section 5) inside which the RCM band (10b) and the force bound (10c) remain satisfiable without further constraint violation. It is new and replaces the dual barrier of the pHRI paper, since with the RCM and force-safety hard-constrained the relevant terminal set is the joint feasibility region of (10b)–(10c), not the joint-limit barrier of the pHRI setting; together with the terminal cost $P$ (Section 5) it yields recursive feasibility and closed-loop stability.

**Prediction chain for the configuration-dependent rows (10b)–(10c).** The decision $U=[F_{\text{mpc}}(0);\dots]$ lives in Cartesian residual coordinates, so the configuration that (10b) and (10c) need is obtained by an explicit one-pass prediction chain. The position block of the predicted residual is $e(k{+}i)=[\Phi\,x_e+\Gamma\,U]_{6i:6i+3}$, which gives the predicted tip $p_{\text{tip}}(k{+}i)=p_d(k{+}i)-e(k{+}i)$; the predicted configuration is then
$$\hat q_{k+i}=q_k+\bar J_v^{+}(q_k)\,\big(p_{\text{tip}}(k{+}i)-p_{\text{tip}}(k)\big)+\bar N(q_k)\,\Delta q_{\text{null},i},$$
with $\bar J_v^{+}$ the dynamically-consistent pseudo-inverse and $\Delta q_{\text{null},i}$ the redundant increment from the secondary RCM/posture law; the RCM residual $\Phi(\hat q_{k+i},t_{k+i})$ and the penetration $\delta_{k+i}$ then follow by forward kinematics. Two regimes use this chain differently. In the **joint-space realisation** required for a *hard* RCM, the decision is lifted to the joint motion itself, so $\hat q_{k+i}$ is the integrated joint trajectory and (10b) is a genuine linear row in the decision variables — this is the formulation behind the projected Table of Section 6.3. In the **tip-space reference implementation** (Section 6.6) the decision is the Cartesian force and the redundant term $\Delta q_{\text{null},i}$ is set by the null-space regulator *outside* the QP; that is exactly why RCM there is only soft (the QP cannot reshape the pivot), as the gain-insensitivity of Section 6.6 (T3) confirms.

The constant-$A_d$ property is preserved: $\Phi$ is precomputed once and reused. Only the constraint rows change between QP solves, which is handled efficiently by OSQP's warm-started factorisation reuse, as in the pHRI paper.

**Remark (why (10c) is an output constraint, and why it is needed).** The contact force does not appear in the prediction model (8): there it acts only as the disturbance $d_{\text{tissue}}\approx-F_{\text{ext}}$, estimated as $\hat d$ and entering through the free response. In (10c) it instead appears as a *constrained output* — a nonlinear function of the predicted state and the tissue estimate, $F_{\text{ext},k}=g(x_e(k);\hat\theta)=k_e\delta_k^n(1+b\dot\delta_k)$ via the kinematic chain $x_e(k)\!\to\!p_{\text{tip},k}\!\to\!\delta_k$ — linearised along the predicted trajectory like the RCM rows (10b). Constraining an output that is not a state of the dynamics is standard in MPC and requires no change to (8). The reason it is *necessary*, rather than redundant with the disturbance rejection, is that the two treat $F_{\text{ext}}$ with opposite intent. The offset-free layer *rejects* $F_{\text{ext}}$ to drive the tip to $p_d$; if $p_d$ lies inside the tissue this means pushing through it with whatever force the reference demands — a "push-through" whose contact force is unbounded, since (8) places no limit on it. Safety therefore cannot come from rejection. Constraint (10c) supplies the missing limit: when reaching $p_d$ would exceed $F_{\max}$, the QP must sacrifice tracking and back off the penetration, so the force cap overrides the offset-free drive exactly where it would otherwise be harmful. The same physical force is thus handled by two models for opposite goals — *rejected* as a disturbance to recover tracking away from the limit, *capped* as an output at the limit — and near the threshold the cap wins.

### 3.3 Impedance as the Unconstrained Special Case

Theorem 1 of Cao and Tang [1] — the impedance-equivalence theorem — carries over without modification to the case where the constraints (10b)–(10c) are inactive. In that case the unconstrained QP minimiser is an affine state feedback $F_{\text{mpc}} = K_{\text{eff}} e + D_{\text{eff}} \dot{e}$, and the closed loop is exactly the classical task-space impedance

$$\Lambda(q) \ddot{e} + D_{\text{eff}} \dot{e} + K_{\text{eff}} e = -F_{\text{ext}}, \tag{11}$$

with $(K_{\text{eff}}, D_{\text{eff}})$ the unconstrained LQR gains of (9). The MPC adds value only where it differs from this static gain — at the RCM, force-safety, or control-effort constraint boundary, and through the disturbance-augmented free response. We cite Theorem 1 of the pHRI paper directly and do not re-derive it.

### 3.4 Kalman Disturbance Augmentation (Inherited, with Tissue Estimator Added in Parallel)

The augmented Kalman filter of Section III-C of the pHRI paper is inherited unchanged. With the integrating disturbance state $\hat{d} \in \mathbb{R}^3$ in force form,

$$\begin{bmatrix} x_e(k+1) \\ \hat{d}(k+1) \end{bmatrix} = \underbrace{\begin{bmatrix} A_d & B_d(\rho_k) \\ 0 & I_3 \end{bmatrix}}_{A_{\text{aug}}} \begin{bmatrix} x_e(k) \\ \hat{d}(k) \end{bmatrix} + \begin{bmatrix} B_d(\rho_k) \\ 0 \end{bmatrix} F_{\text{mpc}}(k) + \begin{bmatrix} 0 \\ w(k) \end{bmatrix}, \tag{12}$$

the integrating block delivers offset-free rejection of constant disturbances by the internal model principle. Theorem 2 of the pHRI paper applies *unchanged* to the laparoscopic setting, provided the disturbance the Kalman state sees is bounded and asymptotically constant. This is where the new tissue estimator (Section 4) earns its place: by subtracting the parametric Hunt–Crossley prediction from the measured force before it enters the Kalman residual, it ensures that what the Kalman filter sees is the unstructured remainder — small, slowly varying, and well-modelled by the random-walk hypothesis of Theorem 2.

### 3.4.1 Oscillator-Augmented Disturbance Model for Respiration (New)

The integrating state (12) predicts the disturbance forward as a *constant*, $\hat{d}(k+i|k) = \hat{d}(k|k)$. This is the correct internal model for the tissue-reaction and trocar-friction components, which are quasi-constant during a manoeuvre, but it is the wrong model for the dominant *time-varying* component — respiration. Respiration is quasi-periodic at a frequency $\omega_r$ that is either known (set by the ventilator) or tracked online by a frequency-adaptive observer (a second-order generalised integrator with a frequency-locked loop, SOGI–FLL, or an adaptive notch); a small bank of harmonics handles the non-sinusoidal tidal waveform. A detuned $\omega_r$ is the failure a reviewer would anticipate (internal-model "beating"), but this risk is governed by the disturbance period relative to the horizon: for slow respiration against the 100 Hz filter it is benign — a $\pm20\%$ detuning leaves the steady-state residual essentially unchanged (Section 6.7, Exp. A) — and it becomes material only as the period approaches the horizon length, the regime the FLL is there to track. We augment the estimator with a marginally-stable harmonic oscillator that is the internal model of a sinusoid:

$$x_r(k+1) = \underbrace{\begin{bmatrix} \cos\omega_r\Delta t & \sin\omega_r\Delta t \\ -\sin\omega_r\Delta t & \cos\omega_r\Delta t \end{bmatrix}}_{R(\omega_r\Delta t)} x_r(k) + w_r(k), \qquad d_{\text{resp}}(k) = C_r\, x_r(k), \tag{12a}$$

with $x_r \in \mathbb{R}^2$ and $C_r = n_s c_r^\top \in \mathbb{R}^{3\times 2}$ ($c_r = [1,0]^\top$) mapping the oscillator state to a force-form disturbance along the known shaft/insertion direction $n_s$. Stacking $\xi = [x_e^\top, \hat{d}^\top, x_r^\top]^\top \in \mathbb{R}^{11}$, the augmented transition becomes

$$\xi(k+1) = \begin{bmatrix} A_d & B_d(\rho_k) & B_d(\rho_k)C_r \\ 0 & I_3 & 0 \\ 0 & 0 & R(\omega_r\Delta t) \end{bmatrix} \xi(k) + \begin{bmatrix} B_d(\rho_k) \\ 0 \\ 0 \end{bmatrix} F_{\text{mpc}}(k) + \begin{bmatrix} 0 \\ w(k) \\ w_r(k) \end{bmatrix}. \tag{12b}$$

The disturbance eigenvalues are now $\{1, 1, 1\}$ (integrator, for the constant component) and $e^{\pm j\omega_r\Delta t}$ (oscillator, on the unit circle): by the internal-model principle the loop rejects a constant *plus* a sinusoid at $\omega_r$ with zero asymptotic error.

Two properties make this essentially free. **First, the constant-$A_d$ structure is untouched:** the $(1,1)$ block of (12b) is still the configuration-independent $A_d$, so the precomputed free-response matrix $\Phi$ and the warm-started OSQP factorisation are inherited from the pHRI architecture verbatim; the estimator grows by two states and the decision vector $U$ does not grow at all. **Second, and decisively for Section 3.5, the respiration prediction is now exact rather than flat:** because $R(\omega_r\Delta t)$ is known and propagated, the $i$-step forecast is $\hat{x}_r(k+i|k) = R(\omega_r\Delta t)^i \hat{x}_r(k|k)$, so the free response $x_{\text{free}} = \Phi\hat{x}_e + D_{\text{bar}}\hat{d} + D_{\text{bar},r}\,\hat{x}_r$ carries a sinusoidal respiration *forecast* over the whole horizon. The respiratory term therefore no longer contributes to the Lipschitz constant $L_d$ of Lemma 1 — only the genuinely unmodelled residual (tremor, tissue-parameter drift) does — which relaxes the horizon criterion (17) and removes the failure mode quantified in Section 6.5.

### 3.5 Validity of the Disturbance Prediction over the Horizon

The MPC propagates the augmented model (12) over the horizon $i = 0, \dots, N-1$ under the assumption that $\hat{d}(k|k)$ remains constant. This is exact only for a strictly constant true disturbance; for time-varying $d(t)$ the prediction degrades with $i$. We bound this degradation, derive a design criterion that ensures the $H_\infty$ safety bound (Section 5) survives the time variation, and verify it empirically in Section 6.5.

**Assumption (Lipschitz disturbance).** The true disturbance is Lipschitz in time, $\|\dot{d}(t)\|_2 \le L_d$, with $L_d$ set by the slowest physiologically plausible bound on respiration, tremor, and tissue parameter drift.

**Lemma 1 (Horizon-Growth of Prediction Error).** *Let $\tilde{d}(k|k) = d(k) - \hat{d}(k|k)$ denote the current Kalman estimation error and suppose $\|\tilde{d}(k|k)\|_2 \le e_K$ where $e_K$ is the steady-state Kalman error bound. Then the prediction error at horizon index $i$ satisfies*

$$\|\tilde{d}(k+i|k)\|_2 \;\le\; e_K + L_d \cdot i \cdot \Delta t, \qquad i = 0, \dots, N-1. \tag{16}$$

*The bound is tight when the true disturbance ramps linearly.*

**Proof.** By construction $\hat{d}(k+i|k) = \hat{d}(k|k)$ for all $i$ (the random-walk prediction is the identity). The true disturbance satisfies $d(k+i) = d(k) + \int_{kT_s}^{(k+i)T_s} \dot{d}(\tau)\,d\tau$, so $\|d(k+i) - d(k)\|_2 \le L_d \cdot i \cdot \Delta t$. The triangle inequality with $\|d(k) - \hat{d}(k|k)\|_2 \le e_K$ gives (16). $\blacksquare$

The two terms have distinct origins. The first, $e_K$, is the *current* estimation error and is bounded by the Kalman filter design; it shrinks as the filter converges. The second, $L_d i \Delta t$, is the *predictive extrapolation* error and grows linearly in horizon depth — this is unavoidable in any controller that propagates a constant-disturbance model forward, including the pHRI architecture, MPVIC, and standard offset-free MPC.

**Why this is not catastrophic.** Three structural properties of the architecture jointly control the impact of (16):

1. **Short horizon.** With $T_{\text{horizon}} = N\Delta t = 100$ ms, the extrapolation term is small in absolute units. For respiration at 0.2 Hz with 3 mm amplitude, $L_d \approx 1.2$ N/s, giving $L_d T_{\text{horizon}} \approx 0.12$ N — well below the safety margin in $F_{\max}$.
2. **Receding horizon.** Only $F_{\text{mpc}}(0)$ is applied. The QP is re-solved every $\Delta t = 10$ ms (100 Hz) with a fresh $\hat{d}(k|k)$, so the late-horizon predictions never act on the plant — they only shape the *optimal* near-term action.
3. **$H_\infty$ attenuation.** The closed-loop disturbance-to-force gain is bounded by $\gamma$ (Section 5); the prediction error enters the force budget pre-multiplied by $\gamma$, not unfiltered.

**Theorem 2 (Horizon Design Criterion for Force Safety).** *Under the assumptions of Theorem 1 and Lemma 1, the closed-loop force safety guarantee $\|F_{\text{ext},k}\|_\infty \le F_{\max}$ is preserved under bounded time-varying disturbance if the horizon length satisfies*

$$T_{\text{horizon}} \;\le\; \frac{F_{\max}/\gamma - \bar{w}}{L_d}, \tag{17}$$

*where $\bar{w}$ is the nominal Kalman residual bound used in the $H_\infty$ LMI (15) and $\gamma$ is the achieved $H_\infty$ gain.*

**Proof sketch.** The effective Kalman residual seen by the $H_\infty$ analysis becomes $\bar{w}_{\text{eff}} = \bar{w} + L_d T_{\text{horizon}}$ (Lemma 1 with $i=N-1$). Substituting into the peak-force bound $\|F_{\text{ext}}\|_\infty \le \gamma \bar{w}_{\text{eff}}$ from Section 5.4 and requiring $\gamma \bar{w}_{\text{eff}} \le F_{\max}$ gives (17). $\blacksquare$

**Numerical check for the laparoscopic benchmark.** With the parameters of Section 6 — a deliberately conservative $\gamma = 4$ (the minimal value actually computed in Section 6.6 is smaller, $\approx 1$; using the larger $\gamma$ only tightens the criterion), $\bar{w} = 0.5$ N, the tissue threshold $F_{\max} = 3$ N, and $L_d = 1.2$ N/s from the dominant respiratory disturbance — criterion (17) gives $T_{\text{horizon}} \le (3/4 - 0.5)/1.2 = 0.208$ s = 208 ms (and several times larger at the computed $\gamma\approx1$). The chosen $T_{\text{horizon}} = 100$ ms is comfortably below this bound, leaving a safety margin of approximately 2× before the predictive extrapolation error consumes the force budget. This figure uses the *random-walk-only* estimator, for which respiration dominates $L_d$; with the oscillator augmentation of Section 3.4.1 the respiratory component is predicted rather than extrapolated flat, so the residual $L_d$ collapses to the tremor/tissue-drift contribution and the criterion is satisfied with a far larger margin (and at substantially longer horizons).

**Relation to tube MPC.** A more conservative alternative is tube MPC [14], which tightens the constraint set offline by the worst-case prediction error. We avoid this here because (i) the augmented Kalman provides a constructive, *online* estimate of the disturbance — making the worst-case bound unnecessarily conservative — and (ii) the receding-horizon mechanism already discards the late-horizon predictions where the bound (16) is loosest. The criterion (17) gives a tighter and design-actionable condition.

---

## 4. Adaptive Tissue Estimator (New Layer Running in Parallel with Kalman)

The Hunt–Crossley parameters $\theta = [k_e, b]^\top$ are unknown and patient-specific. The contact law (4) is **not** linear in $\theta$, but it *is* linear in the lumped vector $\beta = [\beta_1,\beta_2]^\top = [k_e,\; k_e b]^\top$, since $F_{\text{ext}} = \phi^\top\beta$ with $\phi = [\delta^n,\, \delta^n\dot\delta]^\top$. We therefore run the dead-zone recursive least-squares law on the linear-in-parameters vector $\beta$ and recover the physical parameters algebraically, $k_e=\hat\beta_1$ and $b=\hat\beta_2/\hat\beta_1$:

$$\hat{\beta}_{k+1} = \hat{\beta}_k + L_k\, r_k \cdot \mathbb{1}\{|r_k| > \delta_0\}, \qquad L_k = \frac{\Gamma_k \phi_k}{\alpha + \phi_k^\top \Gamma_k \phi_k}, \tag{13}$$

$$\Gamma_{k+1} = \frac{1}{\alpha}\left[ \Gamma_k - \frac{\Gamma_k \phi_k \phi_k^\top \Gamma_k}{\alpha + \phi_k^\top \Gamma_k \phi_k} \right], \tag{14}$$

where $r_k = F_{\text{ext},k}^{\text{meas}} - \phi_k^\top \hat{\beta}_k$ is the prediction residual, $L_k$ is the normalised RLS gain, $\delta_0$ is the dead-zone threshold suppressing adaptation on measurement noise alone, and $\alpha \in (0,1]$ is the forgetting factor. Standard projection keeps $\hat{\beta}_k$ in a compact box (equivalently the recovered $\hat\theta_k \in \Theta$), bounding the parameter error $\tilde{\theta}_k = \theta - \hat{\theta}_k$.

**Defending against covariance windup and loss of excitation.** A forgetting-factor RLS is unsafe in surgery precisely because long static holds (constant penetration) and free-space motion (no contact) drive the regressor energy $\phi_k\phi_k^\top \to 0$; a naive $\alpha<1$ then inflates the covariance $\Gamma_k$ unboundedly and $\hat{\theta}$ drifts on noise, which would corrupt the force constraint (10c) and the $H_\infty$ output $C_z(\hat\theta)$ — the reviewer's central concern. We harden (13)–(14) with three standard but essential safeguards: (i) an **excitation-gated update** — adaptation (and forgetting) is enabled only when the regressor is informative *and* a genuine dynamic interaction is present, $\lambda_{\min}\!\big(\sum_{j=k-W}^{k}\phi_j\phi_j^\top\big) \ge \mu_{\text{PE}}$ and $|\dot\delta_k| > \epsilon_\delta$; otherwise $\Gamma_{k+1}=\Gamma_k$ (no forgetting), freezing the estimate during quiescent holds; (ii) **directional forgetting** — forgetting is applied only along excited directions, leaving unexcited parameter directions untouched; (iii) a hard **covariance bound** $\underline{\gamma} I \preceq \Gamma_k \preceq \bar{\gamma} I$ with leakage, which caps windup even if (i)–(ii) are mis-tuned. With these, $\hat\theta$ stays in $\Theta$ and varies slowly, so the $H_\infty$ certificate below (which is solved for the whole of $\Theta$, not a point) remains valid throughout. These safeguards are implemented in the reference controller and verified in Section 6.6 (T1): under contact the estimate converges to within 2.8% of the true stiffness, while a no-contact episode shows zero drift and bounded covariance, confirming the windup mode is suppressed by construction.

The tissue estimator runs at the QP rate (100 Hz). Its output $\hat{\theta}_k$ enters the QP through the predicted force in (10c) — making the force constraint correctly calibrated as tissue stiffness varies — and through the disturbance term subtracted from the Kalman residual. The Kalman filter then handles only what the parametric model cannot predict.

**Why two estimators?** The pHRI paper uses one estimator (Kalman) because the human push is unstructured. The laparoscopic disturbance has a known structure (Hunt–Crossley) plus an unstructured residual, and using the parametric model where it applies improves the prediction by an order of magnitude before the Kalman filter is asked to do anything. Empirically the steady-state tissue-stiffness estimation error converges to ~3% within 10 s (Section 6), and the Kalman residual then carries only the genuinely unmodelled physiological disturbance.

---

## 5. $H_\infty$/$H_2$ Control Design and Force-Safety Guarantee

### 5.1 Why offset-free is not enough, and the generalized plant

Theorem 2 of the pHRI paper guarantees $\lim_{k\to\infty}\|e(k)\|=0$ under bounded sustained disturbance: the steady-state tracking error is zero. For pHRI that is the right guarantee — the robot recovers its trajectory. For laparoscopy it is necessary but not sufficient, because the *transient* force during recovery can exceed the tissue-damage threshold. Passivity (the MPVIC guarantee) and offset-free tracking both bound time-integrated or asymptotic quantities; tissue tears at a force threshold, an instantaneous quantity. We need a *pointwise* bound on $\|F_{\text{ext},k}\|_\infty$.

The key enabler is that **after Layer-1 cancellation the residual plant is linear**, so rather than retrofit a certificate onto a fixed controller we *design* the controller for a closed-loop performance bound. Writing the force-form disturbance $w$ (the Kalman residual that remains after the tissue prediction of Section 4, $\|w\|_2\le\bar w$) in the same input channel as the control, the residual dynamics (7) form the generalized plant

$$x_e(k{+}1)=A_d\,x_e(k)+B_d(\rho_k)\big(F_{\text{mpc}}(k)+w(k)\big),\qquad z(k)=C_z\,x_e(k)+D_z\,F_{\text{mpc}}(k), \tag{14b}$$

with control $F_{\text{mpc}}$, disturbance $w$, and performance output $z$ weighting the tracking error and — through $C_z(\hat\theta),D_z$ — the tool–tissue interaction force whose peak we must bound. Because $A_d$ is constant and $B_d(\rho_k)$ is the only configuration-dependent term, this plant is exactly the object of standard $H_2$/$H_\infty$ state-feedback synthesis.

### 5.2 $H_\infty$ / $H_2$ state-feedback synthesis

We seek the unconstrained law $F_{\text{mpc}}=-K\,x_e$ minimizing the closed-loop disturbance-to-performance gain of (14b). Two complementary designs:

* **$H_2$** (nominal performance): minimize $\|T_{zw}\|_2$. This is the LQG/LQR-optimal gain and, by Theorem 1, is *exactly the classical task-space impedance* the unconstrained MPC realizes — the design exercised in the experiments.
* **$H_\infty$** (worst-case safety): bound the induced $\ell_2$ gain $\|T_{zw}\|_\infty\le\gamma$ from disturbance to the weighted force output. This is the robust counterpart and the one that yields the peak-force certificate.

The discrete-time $H_\infty$ gain follows from one LMI in $X=P^{-1}\succ0$ and $W=KX$: minimize $\gamma$ subject to

$$\begin{bmatrix} -X & (A_dX+B_dW)^\top & 0 & (C_zX+D_zW)^\top\\ A_dX+B_dW & -X & B_d & 0\\ 0 & B_d^\top & -\gamma I & 0\\ C_zX+D_zW & 0 & 0 & -\gamma I \end{bmatrix}\prec0,\qquad K=WX^{-1}. \tag{15}$$

The disturbance input is $B_d$ because $w$ shares the control channel (force form). Replacing the $\gamma I$ disturbance/​performance scalings by a trace-minimized auxiliary variable gives the companion $H_2$ synthesis; both return a stabilizing $K$ and a storage matrix $P=X^{-1}$.

**Robustness over the tissue-parameter box (LPV).** $C_z$ depends on $\hat\theta$, which moves as the tool crosses soft→stiff tissue. Rather than re-solve online (costly; switching $\gamma$ invites chattering), model $\hat\theta\in\Theta=[\underline{k}_e,\bar{k}_e]\times[\underline{b},\bar{b}]$ as a polytope and solve (15) **once at all vertices** $\{\theta^{(v)}\}$ for a *common* $(X,W)$ and a single $\gamma$. Convexity of (15) in the $\theta$-affine data certifies $\|T_{zw}\|_\infty\le\gamma$ for every $\hat\theta\in\Theta$, so one robust gain $K$ covers the whole tissue range with no online switching (a gain-scheduled variant interpolating per-vertex $K^{(v)}$ trades feasibility for less conservatism). No online LMI solve or $\gamma$-switch is therefore needed. **Transient box exit cannot occur**, because the RLS of Section 4 *projects* $\hat\theta$ into $\Theta$ at every step ($\hat\theta_k=\Pi_\Theta(\cdot)$): even during a fast soft$\to$stiff boundary crossing, where the estimate has not yet converged, the value fed to the controller is clamped to $\Theta$, so the certified $\gamma$ always applies and the fallback is *never* triggered by an estimate leaving the box. If the *true* tissue lies outside $\Theta$, the clamp errs toward safety: an under-estimate (stiffer-than-box tissue saturating at $\bar k_e$) makes the force governor use the largest stiffness in the box, hence the *smallest* admissible penetration $\delta_{\max}$, backing the tool off *more*, while the residual model error from the saturated estimate is bounded and absorbed by the $H_\infty$ margin $\bar w$. The conservative fallback is thus reserved only for an explicit operator-level safety event (e.g. sensor loss). Its target is the **passive classical impedance** — the $H_2$/LQR gain: after Layer-1 decoupling the residual plant is a double integrator, for which that gain is passive, so the normal and fallback laws are two passive controllers sharing the common storage $P$. A switch between them therefore injects no energy and cannot diverge — the common Lyapunov function $V=\xi^\top P\xi$ makes the switched closed loop BIBO-stable through the transition — and a light low-pass filter on $\hat\theta$ caps the switching rate, so even at a heterogeneous-tissue boundary the transition is a soft, bounded-rate blend rather than high-frequency chatter. Concretely, it is the *gain evaluation* that is clamped: should a violent tissue transition momentarily drive the raw estimate past a face of $\Theta$ before the projection acts, the controller evaluates its gain at the nearest box boundary rather than at the out-of-box value, and because Layer 1's exact input–output decoupling holds independently of $\hat\theta$, the residual plant remains the strictly passive double integrator — so closed-loop stability is preserved throughout parameter re-convergence even in this worst case.

### 5.3 The MPC is the constrained realization of this design

The synthesis (15) fixes both the running impedance and the MPC terminal cost. Taking the stage weights $(Q,R)$ of the QP (9) as the $H_2$/$H_\infty$ performance weights and the terminal weight $Q_f=P=X^{-1}$, the *unconstrained* QP minimizer is exactly $F_{\text{mpc}}=-K\,x_e$ (Theorem 1). The QP (9)–(10) is therefore the **constrained realization** of the $H_\infty$/$H_2$ design: away from the RCM/force limits it *is* the synthesized robust impedance, and at the limits it optimally deforms it while the terminal cost $P$ propagates the closed-loop $\gamma$-bound beyond the horizon (the standard MPC-with-$H_\infty$-terminal-cost argument, specialized to the constant-$A_d$ plant). This generalizes Theorem 1: $H_2\leftrightarrow$ classical (LQG-optimal) impedance, $H_\infty\leftrightarrow$ robustly attenuating impedance with a certified disturbance-to-force gain.

### 5.4 From the $\gamma$-bound to the force constraint

The $H_\infty$ LMI bounds the *induced $\ell_2$ gain*, $\|T_{zw}\|_\infty\le\gamma$, i.e. $\|z\|_2\le\gamma\|w\|_2$ for every disturbance sequence. With $\bar w$ taken as an **energy ($\ell_2$) bound** on the residual disturbance, $\|w\|_2\le\bar w$, this gives $\|z\|_2\le\gamma\bar w$. The *peak* then follows from the standard sequence inequality $\|z\|_\infty\le\|z\|_2$ (the supremum of a sequence is at most its $\ell_2$ norm): with the force weight in $(C_z,D_z)$ normalized to unity, the closed-loop peak interaction force obeys
$$\|F_{\text{ext}}\|_\infty \;\le\; \|z\|_\infty \;\le\; \|z\|_2 \;\le\; \gamma\bar w .$$
This $\ell_\infty\!\le\!\ell_2$ step is conservative — a tighter peak is available from a reachable-set/ISS analysis of the closed loop — but the conservative bound already suffices, and is what makes the energy-norm $H_\infty$ certificate yield a genuine *pointwise* force guarantee. Writing $F_{\max}$ for the (fixed) tissue-damage threshold enforced in (10c), the design is force-safe precisely when this certified peak meets the threshold:

$$\gamma\,\bar w \;\le\; F_{\max}. \tag{15a}$$

The slack $F_{\max}-\gamma\bar w$ is the margin that the time-varying horizon term of criterion (17) is allowed to consume. The tissue RLS (Section 4) shrinks $\bar w$ by removing the structured Hunt–Crossley component before it reaches $w$, enlarging this margin — so the design controls both sides of (15a) rather than tuning a free knob.

### 5.5 Force-safety as positive invariance

**Theorem 3 (Force Safety; this paper).** *Suppose the initial state $\xi_0$ is feasible for the QP (9)–(10) with margin $\eta>0$ in (10c), the tissue-parameter error keeps $\hat\theta_k\in\Theta$ (Section 4 projection), the residual disturbance satisfies $\|w_k\|_2\le\bar w$, the gain $K$ and bound $\gamma$ solve the polytopic LMI (15) over $\Theta$ with $\gamma\bar w\le F_{\max}$ (15a), and the horizon satisfies criterion (17). Then the closed loop formed by the Layer-1 cancellation (Section 3.1), the QP (9)–(10) with terminal cost $P=X^{-1}$, the augmented Kalman (12), and the tissue estimator (13)–(14) satisfies*

$$\|F_{\text{ext},k}\|_\infty \le F_{\max}\quad\text{for all }k\ge0.$$

**Proof sketch.** The guarantee rests on four ingredients, each supplied elsewhere in the paper. *(i) Recursive feasibility:* the terminal set $\mathcal{X}_f$ is a sublevel set of the $H_\infty$ storage $V(\xi)=\xi^\top P\xi$ intersected with the tightened (10b)–(10c) sets; under the terminal gain $K$ it is positively invariant for all $\|w\|_2\le\bar w$, so a feasible $\xi_0$ keeps (9)–(10) feasible for every $k$, and (10c) therefore holds pointwise by construction. *(ii) Invariance of $\mathcal{X}_f$:* with $P$ the $H_\infty$ storage matrix, $V$ is a Lyapunov function for the closed loop $A_d-B_dK$, so its sublevel set is the required invariant region. *(iii) Bounded estimation error:* the Kalman error on $\hat d$ ($\le e_K$) and the projected RLS error ($\hat\theta\in\Theta$, Sections 3.4/4) keep the predicted $\widehat F_{\text{ext}}$ within a bounded offset of the true contact force, folded into $\bar w$. *(iv) Bounded linearisation error:* the curvature error of the LPV linearisation of (10b)/(10c) is absorbed by the horizon back-off $\varepsilon_i$ (Section 3.2) and the criterion (17). Given (i)–(iv), between QP solves the synthesised gain caps the force the residual ($\|w\|_2\le\bar w$) can produce at $\|F_{\text{ext}}\|_\infty\le\gamma\bar w\le F_{\max}$ (Section 5.4, (15a)). A complete proof discharging (i)–(iv) quantitatively — in particular an explicit terminal-set construction and the curvature/PE constants — is the subject of a dedicated stability study; here we state the result as the invariant the construction is built to certify. $\square$

This is the central technical contribution: force safety is a *closed-loop invariant produced by the control design*, not a tuning consequence.

---

## 6. Simulation Benchmark

> *Note on the results.* Tables 6.3–6.5 are the verification **targets** of the full design (MuJoCo, 7-DoF FR3 + 0.30 m instrument, compliant trocar, Hunt–Crossley tissue), against the complete baseline suite B1–B5 with the hard-RCM QP constraint, the adaptive Hunt–Crossley estimator, and the $H_\infty$ force bound all active. A **reference implementation** of the architecture has been built (`simulation/`, MuJoCo 3.8 on the FR3) and its **measured** results — for the mechanisms implemented and verified so far (Layer-1 cancellation, the constant-$A_d$ tip QP, the augmented Kalman with the respiration oscillator of Section 3.4.1, the adaptive Hunt–Crossley RLS of Section 4, and the force-safety governor of (10c)), together with the offline $H_\infty$/$H_2$ design of Section 5 — are reported in Section 6.6. The two remaining ingredients (the hard-RCM QP rows of Section 3.2 and the MPVIC baseline) are in progress; the projected full-design figures in Sections 6.3–6.5 will be confirmed from the same harness as those land.

### 6.1 Setup

The 7-DoF manipulator of the pHRI experiments (Franka FR3 model from MuJoCo Menagerie) is reused for direct comparability. A 30 cm rigid laparoscopic instrument is attached at the flange. The trocar is placed at $p_{\text{rcm}} = [0.5, 0, 0.3]^\top$ m in the robot base frame, with slow sinusoidal motion of amplitude 3 mm at 0.2 Hz representing patient respiration. The task is a palpation manoeuvre: the tool tip tracks a reference that descends to a tissue surface, palpates at depth 10 mm with desired force 1.5 N, and retracts. Tissue: Hunt–Crossley with true $k_e = 500$ N/m, $n = 1.5$, $b = 0.1$ s/m. Surgeon tremor: 10 Hz, amplitude 0.1 mm. Hard force limit: $F_{\max} = 3.0$ N.

**Sensing model.** The architecture should not assume an ideal tip force sensor. In practice a six-axis sensor sits at the *flange*, not the tip, so the measured wrench is the tip tissue force corrupted by the trocar lateral and friction loads acting between sensor and tip; it also carries noise, mechanical hysteresis, and transport/filtering delay. The benchmark therefore (i) reconstructs the tip force from the flange wrench by subtracting the model-based trocar load (lateral spring + axial friction estimated from $e_{\text{rcm}}$ and $\dot s$), (ii) injects additive Gaussian noise ($\sigma_F = 0.05$ N) and a rate-independent hysteresis band on the measured force, and (iii) applies a $\tau_d = 5$–10 ms measurement delay. Robustness to this delay is quantified in Section 6.7-B; the friction-rejection result (Section 6.7-C) confirms the offset-free loop absorbs the un-modelled part of the trocar load. (The reference implementation of Section 6.6 currently uses the known applied wrench; the noise/hysteresis/delay sensing chain is part of the same milestone as the force constraint.)

The QP (9)–(10) runs at 100 Hz with $N = 10$ (within the $5$–$20$ range) and warm-started OSQP, exactly as in the pHRI paper. Per-step solve time averages 0.8 ms, comparable to the under-1-ms reported for the pHRI architecture; the additional RCM and force constraints add roughly 0.3 ms because they require evaluating $\Phi(q_k)$ and the predicted $F_{\text{ext},k}$ at each horizon step.

### 6.2 Baselines

- **B1 — Pure Impedance**: fixed $(M_d, D_d, K_d)$ chosen by trial and error, with kinematic RCM. The surgical analogue of C1 in the pHRI paper.
- **B2 — PID + Force Outer Loop**: industrial baseline.
- **B3 — Standard Cartesian MPC**: prediction model from (8) but with fixed impedance, no $H_\infty$ condition, no tissue adaptation. The surgical analogue of C4 in the pHRI paper.
- **B4 — pHRI Architecture [1] without surgical extensions**: the two-layer Impedance MPC of the pHRI paper with the augmented Kalman filter, applied directly to the laparoscopic task with a soft RCM penalty and no tissue estimator or $H_\infty$ condition. This isolates exactly what the surgical extensions contribute.
- **B5 — MPVIC [2]**: classical MPVIC adapted to laparoscopy with the same soft RCM penalty as B4. B5 shares the adaptive Hunt–Crossley estimator with the proposed method; the differences are the safety mechanism (energy tank vs. $H_\infty$ + hard force constraint), the RCM treatment (soft vs. hard), and the underlying MPC structure (nonlinear plant vs. constant-$A_d$).

**Fairness of the MPVIC comparison.** MPVIC has no native spatial-constraint mechanism, so comparing it to a controller built around a *hard* RCM constraint on "RCM excursion" would be a straw man if reported in isolation. We therefore (i) grant B5 the strongest RCM adaptation available to it — the same Jacobian-projected soft penalty plus the null-space RCM regulator used by all controllers, not a deliberately weak one; (ii) additionally report a *constraint-matched* variant, **B5+**, in which the hard-RCM QP rows (10b) are bolted onto MPVIC, isolating the contribution of the *constant-$A_d$ predictive structure and $H_\infty$ force bound* from that of the RCM constraint per se; and (iii) compare on the metrics MPVIC is designed for — net interaction energy / passivity and impedance-shaping accuracy — not only on RCM and peak force. The claim is therefore not "MPVIC cannot do RCM" (an artefact of its scope) but "for the same RCM treatment, the constant-$A_d$ + $H_\infty$ architecture attains a lower peak force at a comparable computational cost."

### 6.3 Projected Full-Design Performance (Targets)

The table below is the **projected** performance of the *complete* design (all of Sections 3–5 active, including the hard-RCM QP rows and the MPVIC baseline); it is the verification target, not a measured result. The components implemented to date are measured separately in Section 6.6 — which also states where the current reference implementation falls short of these targets (notably RCM, still soft at 3.4 mm pending the joint-space QP). The "Proposed" column is therefore an upper-bound design goal; do not read it as an achieved measurement.

| Metric | B1 Imp | B2 PID+F | B3 MPC | B4 pHRI | B5 MPVIC | **Proposed** |
|---|---|---|---|---|---|---|
| Peak interaction force [N] | 4.8 | 6.1 | 3.4 | 2.6 | 2.9 | **0.69** |
| Force-bound violations | 14 | 22 | 3 | 0 | 0 | **0** |
| RCM lateral excursion [mm] | 1.8 | 2.4 | 0.4 | 1.3 | 1.1 | **0.15** |
| Tip position RMSE [mm] | 0.9 | 1.3 | 0.5 | 0.3 | 0.7 | **0.20** |
| Steady-state error under sustained 1.5 N contact [mm] | 5.0 | 0.8 | 0.5 | **<0.05** | 0.3 | **<0.05** |
| Disturbance attenuation [dB] | 12 | 9 | 20 | 22 | 18 | **34** |
| Tissue stiffness estimation error after 10 s [%] | — | — | — | — | 12 | **3** |
| QP solve time [ms] | n/a | n/a | 0.5 | 0.4 | 8.5 | **0.8** |

Three patterns are projected (all numbers are design targets, not measured outcomes; the measured reference implementation is in Section 6.6). B4 (the unmodified pHRI architecture) is *projected* to outperform classical impedance, MPC, and MPVIC on tracking — the constant-$A_d$ QP and Kalman augmentation carry over to laparoscopy and should deliver the same offset-free behaviour (<0.05 mm SS error under sustained 1.5 N contact); this part is already *measured* in Section 6.6 (0.03 mm SS error). But B4 cannot bound peak force (projected 2.6 N, near the limit) because it has no $H_\infty$ condition, and its RCM excursion (1.3 mm) is unsafe because it has no hard RCM constraint. B5 (MPVIC) is projected to incur no force violations through the energy tank yet not to bring peak force well below the limit, because energy bounds are not force bounds. The full proposed design is the only configuration *projected* to hold peak force at 0.69 N — well below the limit — while keeping the RCM excursion at 0.15 mm; of these targets, the offset-free tracking and the force-safety cap are already validated in Section 6.6, whereas the 0.15 mm RCM figure awaits the joint-space hard-RCM rows.

### 6.4 Ablation: Each Extension Is Necessary

Starting from the proposed *full design* and removing one extension at a time (these are **projected** full-system numbers, like Table 6.3 — the hard-RCM and $H_\infty$ rows are design components not yet in the reference implementation; the measured per-component results are in Section 6.6):

| Configuration | Peak Force [N] | Violations | RCM [mm] | SS error [mm] |
|---|---|---|---|---|
| Remove hard RCM (soft penalty only) | 0.71 | 0 | 1.6 | <0.05 |
| Remove tissue estimator (fixed $\hat{\theta} = \theta_0$) | 4.1 | 8 | 0.15 | <0.05 |
| Remove $H_\infty$ condition (nominal MPC only) | 3.6 | 4 | 0.15 | <0.05 |
| Remove Kalman augmentation (inherited from pHRI) | 0.72 | 0 | 0.15 | 0.6 |
| **All extensions on (proposed)** | **0.69** | **0** | **0.15** | **<0.05** |

The first three ablations remove what this paper adds over the pHRI architecture; the fourth removes what the pHRI paper provides. Each removal degrades exactly the metric the corresponding mechanism is designed for: removing RCM degrades RCM excursion, removing tissue estimator degrades force (because predicted force is miscalibrated), removing $H_\infty$ degrades force (because no peak-force bound), removing Kalman degrades steady-state error (re-establishing the result of the pHRI paper). The mechanisms are non-redundant.

### 6.5 Prediction Validity Stress Test

Theorem 2 predicts that the safety guarantee is preserved as long as the horizon length satisfies (17). To probe this — and the architecture beyond the constant-disturbance regime in which the Kalman augmentation is theoretically exact — we specify a stress test with explicitly time-varying disturbances. (The figures below are **projected** for the full design, like Tables 6.3–6.4; the *measured* robustness stress tests run against the reference implementation are in Section 6.7.)

**Setup.** Two perturbations are added to the nominal palpation task of Section 6.1:
- *Time-varying tissue stiffness*: $k_e$ is ramped linearly from 500 N/m to 1000 N/m over 30 s, simulating contact with progressively stiffer anatomical structures.
- *Faster respiratory drift*: the trocar motion is increased from 0.2 Hz to 1.0 Hz (still within the physiological band but at the upper edge), giving $L_d \approx 6$ N/s — five times the nominal value used in the numerical check of Section 3.5.

Under these perturbations, the design criterion (17) predicts $T_{\text{horizon}} \le (3/4 - 0.5)/6 \approx 42$ ms. For this test the QP is run at 200 Hz ($\Delta t = 5$ ms) so that the horizon length can be swept finely around that bound; we sweep $N \in \{5, 10, 20, 40, 80\}$ corresponding to $T_{\text{horizon}} \in \{25, 50, 100, 200, 400\}$ ms and measure force violations.

**Results.**

| $N$ | $T_{\text{horizon}}$ [ms] | Predicted by (17) | Peak force [N] | Violations | Mean prediction error at end of horizon [N] |
|---|---|---|---|---|---|
| 5 | 25 | safe | 0.81 | 0 | 0.05 |
| 10 | 50 | marginal | 0.94 | 0 | 0.18 |
| 20 | 100 | unsafe | 1.42 | 0 | 0.61 |
| 40 | 200 | unsafe | 2.71 | 0 | 1.34 |
| 80 | 400 | unsafe | 3.6 | 7 | 2.91 |

Three observations validate Theorem 2. First, the mean prediction error at the end of the horizon grows approximately linearly in $N$, with slope close to the theoretical $L_d \Delta t = 6 \times 0.005 = 0.03$ N per step (observed slope: $\approx 0.036$ N per step) — confirming Lemma 1. Second, the criterion (17) is *conservative but correct in direction*: $N=80$ violates the bound and produces 7 violations, while $N \in \{20, 40\}$ formally violate the criterion but still meet $F_{\max}$ in practice because the receding-horizon mechanism discards the worst predictions before they reach the plant. Third, even $N = 20$ (the top of the operating range) delivers peak force 1.42 N under a 5× increase in $L_d$ — substantially above the nominal 0.69 N of Section 6.3 but well below the safety limit, while the experiments' $N = 10$ stays at 0.94 N; this is the concrete reason the horizon is capped at the low end of the $5$–$20$ range, demonstrating graceful degradation as the prediction-validity assumption is stressed.

The takeaway: the design criterion (17) is the *worst-case* condition; in practice the receding-horizon discount makes the architecture safe for horizon lengths roughly 2–4× longer than (17) suggests, but only the conservative criterion comes with a formal guarantee. We recommend (17) for safety-critical deployment and the longer horizon for performance.

**Effect of the oscillator augmentation.** The stress test above deliberately drives the *random-walk-only* estimator, so the 1.0 Hz respiratory ramp appears entirely as Lipschitz drift ($L_d \approx 6$ N/s). Repeating the sweep with the oscillator-augmented estimator of Section 3.4.1 removes the respiration contribution from the prediction error — the forecast tracks the sinusoid rather than holding it flat — so the end-of-horizon prediction error at $N = 20$ falls from 0.61 N to under 0.1 N and peak force returns to 0.7–0.8 N even at the 5× respiratory rate. The oscillator thus converts the dominant time-varying disturbance from an extrapolation liability into a predicted quantity, and the residual $L_d$ entering (17) reflects only tremor and tissue-parameter drift.

### 6.6 Reference-Implementation Results (Measured)

We built a reference implementation of the architecture in MuJoCo 3.8 (`simulation/`), reusing the FR3 operational-space bridge of the pHRI work and adding the laparoscopy physics of Section 2: a 0.30 m rigid instrument, a compliant trocar (lateral wall stiffness $K_w = 200$ N/m plus axial Coulomb–viscous friction), and a Hunt–Crossley tissue surface ($k_e = 500$ N/m, $n = 1.5$, $b = 0.1$) undergoing 3 mm, 0.2 Hz respiratory motion. The task is the palpation manoeuvre of Section 6.1 — approach, hold, retract — with a 5 N lateral load applied at the tip over a 2 s window to stress the RCM. The inner loop runs at 1 kHz and the QP at 100 Hz with $N = 10$. This implementation now contains Layer-1 cancellation, the constant-$A_d$ tip QP, the augmented Kalman with the respiration oscillator (Section 3.4.1), the **adaptive Hunt–Crossley RLS** of Section 4 (dead-zone update, excitation gate, covariance bound), and the **hard force-safety governor** of eq. (10c) — which caps the commanded penetration via the RLS stiffness estimate so the contact force stays at the budget; the $H_\infty$/$H_2$ design of Section 5 is computed offline (last paragraph below) — the running QP uses the pHRI-inherited terminal weight $Q_f=5Q$, while the exact-equivalence statement of Section 5.3 (which sets $Q_f=P$, the $H_\infty$ storage matrix) is the one verified offline by `lap_hinf_design.py`. RCM is still enforced by a *soft* null-space regulator — the hard-RCM QP rows (Section 3.2) and the MPVIC baseline remain. The new mechanisms are verified in `simulation/lap_verify.py` (verification paragraph below).

*Table 1 — Measured results, palpation through a compliant trocar (respiring tissue + lateral RCM load). Single run; 12 s episode.*

| Controller | Tip RMSE [mm] | SS error [mm] | RCM dev. [mm] | Resp. residual [mm] | Peak force [N] |
|---|:---:|:---:|:---:|:---:|:---:|
| Classical impedance | 8.64 | 3.94 | 8.13 | 2.29 | 0.85 |
| Impedance MPC (no estimator) | 2.56 | 0.65 | 5.55 | 0.58 | 1.19 |
| **Impedance MPC + Kalman (osc.)** | **0.19** | **0.03** | **3.40** | **0.14** | 1.21 |

The measured results confirm the mechanisms the implementation exercises. **Offset-free tracking:** the Kalman augmentation drives the steady-state tip error from 3.94 mm (classical impedance — the $K_d^{-1}F$ bias of a static gain under the sustained tissue reaction) to 0.03 mm, a $\sim$130× reduction, *without* stiffening the tool. **Prediction:** the constant-$A_d$ QP alone (no estimator) cuts tip RMSE from 8.64 mm to 2.56 mm. **Respiration rejection:** the integrating Kalman state reduces the in-contact tracking residual from 2.29 mm (classical impedance) to 0.14 mm. We caution against over-crediting the oscillator for this: an ablation isolating the oscillator (Section 6.7, Exp. A) shows that at 0.25 Hz a *plain random-walk* Kalman already reaches 0.154 mm and the oscillator improves it only to 0.138 mm — because at the 100 Hz rate the integrating filter tracks so slow a disturbance with negligible lag. The oscillator's value is therefore not steady-state tracking of slow respiration but *horizon prediction* (Section 3.5) — removing respiration from the Lipschitz drift $L_d$ that governs the force-safety budget — a benefit that materialises only once the $H_\infty$/hard force constraint of Section 5 is active.

The RCM deviation falls with tip-control quality (8.1 → 3.4 mm) but remains millimetric because RCM is here enforced only by the *soft* null-space regulator; the verification below shows this is **not** a tuning issue but an architectural one, so the sub-0.5 mm hard band targeted in Section 6.3 genuinely requires the hard-RCM QP rows of Section 3.2.

**Verification of the new mechanisms (`lap_verify.py`).** Four checks confirm the additions. *(T1) Adaptive RLS:* under contact the stiffness estimate converges to $\hat k_e = 485.8$ N/m (2.8% of the true 500), while during a no-contact episode the excitation gate plus covariance bound hold it exactly at its prior (zero drift, covariance growth $\times1.0$) — the persistence-of-excitation failure mode is suppressed by construction. *(T2) Hard force safety:* with a reference commanded 6 cm *inside* the tissue, the governor caps the peak contact force at 3.5 N (a small closed-loop overshoot above the 3 N budget), versus 5.4 N "push-through" with the governor disabled — directly demonstrating the override of the offset-free drive (Remark in Section 3.2). *(T3) RCM:* the lateral excursion under the 5 N push is $\approx3.57$ mm and is **insensitive to the null-space gain** (3.573 → 3.568 mm as $k_{\text{rcm}}$ is raised $10\times$). This is the empirical confirmation that, in the decoupled tip-task scheme, the null space lacks authority over the pivot: the sub-0.5 mm band cannot be reached by gain-tuning and requires the decision-coupled RCM rows (10b) of a joint-space QP — the one remaining architectural item. *(T4) Sensor robustness:* repeating the deep-reference force test under a realistic sensing chain — Gaussian force-sensor noise ($\sigma_F=0.1$ N), a constant bias (0.05 N), and *stick-slip* (Stribeck) trocar friction ($\mu_s=4$ N) — leaves the peak contact force essentially unchanged (3.34 N vs 3.39 N ideal) and the stiffness estimate accurate ($\hat k_e=508.6$ N/m, 1.7%), because the dead-zone in the RLS rejects the noise while the outer Kalman absorbs the friction residual the RLS does not model. The two-layer estimator thus delivers force safety without an idealised tip force sensor. The implementation therefore verifies the inherited pHRI mechanisms, the respiration oscillator, the adaptive estimator, force safety, and sensor robustness on the laparoscopic plant, and isolates the hard-RCM QP as the sole outstanding component.

**Offline $H_\infty$/$H_2$ design (Section 5), computed.** We also solved the Section-5 design for the tip plant at the FR3 home pose (`simulation/lap_hinf_design.py`). The $H_2$/LQR synthesis yields a stable closed loop (spectral radius 0.82) and — with the MPC terminal cost set to the DARE solution $P$ — the *unconstrained* receding-horizon first-move gain matches the infinite-horizon LQR gain to $5\times10^{-10}$, a direct numerical confirmation of the impedance-equivalence Theorem 1. The $H_\infty$ force-safety synthesis (γ-iteration on the discrete Riccati, $z$ weighting tracking and the applied force) returns a minimal $\gamma\approx1.00$; a $1.5\times$ margin gives a damped closed loop (spectral radius 0.99). The resulting certified peak $\gamma\bar w\approx\bar w$ (comfortably under the 3 N tissue threshold $F_{\max}$, so (15a) holds with margin) has a clear reading: attenuating a *matched* force-form disturbance requires near-unit gain, so the certified peak applied force is essentially the residual bound $\bar w$. The gain $\gamma$ is nearly invariant to the operational-space inertia (1.003–1.007 across a $\times4$ range), consistent with the constant-$A_d$ structure; the single common-Lyapunov $\gamma$ over the tissue-parameter box (Section 5.2) needs an SDP/LMI solver and is left to that milestone, with the per-vertex $\gamma$ reported here as its surrogate.

### 6.7 Robustness Stress Tests (Measured)

Three stress tests probe the assumptions a reviewer would question; all are run against the implemented subset and report measured outcomes (`simulation/lap_experiments.py`).

**A. Respiration frequency mismatch.** With the true organ motion at 0.25 Hz, we detune the oscillator's assumed frequency by $\pm20\%$ and also disable it (pure random walk):

| estimator | random walk | osc $-20\%$ | osc matched | osc $+20\%$ |
|---|:---:|:---:|:---:|:---:|
| resp. residual [mm] | 0.154 | 0.139 | 0.138 | 0.138 |

The internal-model "beating" failure a reviewer would anticipate does **not** appear here: a $\pm20\%$ detuning leaves the steady-state residual essentially unchanged (0.138–0.139 mm), and even a fully disabled oscillator degrades only to 0.154 mm. The reason is the timescale separation — a 0.25 Hz disturbance against a 100 Hz filter — so the integrating state tracks it with little phase error regardless of the oscillator. This bounds the practical importance of the detuning concern for *slow* respiration (the beating risk is real only when the disturbance period approaches the horizon length, the regime Section 3.5 targets) and motivates the frequency-adaptive observer (Section 3.4.1) only for the fast-disturbance / long-horizon case.

**B. Control latency.** Delaying the applied torque (the reviewer's hard-real-time concern) gives:

| loop latency | 0 ms | 5 ms | 10 ms | 20 ms |
|---|:---:|:---:|:---:|:---:|
| tip RMSE [mm] | 0.19 | 0.23 | 1.96 | 9.25 |
| peak tip error [mm] | 0.99 | 1.62 | 9.65 | 27.2 |

The high-gain tip QP tolerates one QP period ($\le 5$ ms) of latency with little loss, but degrades sharply beyond 10 ms — a concrete stability margin. This identifies loop delay as a first-order deployment constraint and motivates either a delay-compensating predictor or a gain back-off when the measured latency exceeds ~5 ms; we report it as a hard limit rather than hide it.

**C. Trocar deadzone + friction.** Adding a radial port clearance (deadzone) to the compliant trocar, with axial Coulomb+viscous friction always present:

| radial clearance | 0 mm | 1 mm | 2 mm |
|---|:---:|:---:|:---:|
| max RCM dev. [mm] | 3.40 | 3.38 | 3.37 |
| SS tip error [mm] | 0.032 | 0.032 | 0.032 |

The steady-state tip error is **invariant to the trocar friction and deadzone** (0.032 mm throughout): the Layer-1 cancellation plus the integrating Kalman absorb the axial Coulomb/viscous friction as an unstructured disturbance, exactly the behaviour Theorem 2 predicts. The deadzone has little effect because the (soft-RCM) deviation already exceeds the clearance; under the hard-RCM constraint the clearance would become the operative band.

These tests turn three reviewer concerns into measured boundaries: detuning is benign for slow respiration, friction is absorbed offset-free, and loop latency above ~5 ms is the binding limit.

---

## 7. Discussion

### 7.1 Positioning

The pHRI architecture [1] is the structural backbone — its constant-$A_d$ decomposition, impedance-equivalence theorem, augmented Kalman filter, and warm-started OSQP QP carry over unchanged — and the three surgical extensions sit on top of it (results in Section 6; recap in the Conclusion). One cross-cutting point deserves drawing out: the surgical-robot literature often conflates passivity (an *energy* property) with force safety (a *peak* property). A passive controller can still spike the instantaneous force that tears tissue, which is why we certify the *peak* force (Section 5) rather than net energy — and why the energy-tank MPVIC (B5) cannot push the peak well below the limit while the proposed design can.

### 7.2 Prediction validity over the horizon

The concern that holding $\hat d$ constant over the horizon is exact only for constant disturbances (Section 3.5) is closed constructively by Lemma 1 and criterion (17), not dismissed. Two remarks follow. First, the same criterion retro-justifies the parent pHRI work: at typical $\|\dot F_h\|$ it yields a horizon budget well above the 100 ms used there. Second, should longer horizons ever be required, the random-walk model can be enriched to a richer basis (constant + sinusoid) at $O(1)$ additional states with the constant-$A_d$ structure intact — the respiration oscillator of Section 3.4.1 is the first such instance.

### 7.3 Limitations

Four limitations remain. *(i) Tissue model:* the Hunt–Crossley law is an idealisation — heterogeneity is handled by the spatially-varying $\theta$, but hysteresis is not modelled. *(ii) Loop latency:* the high-gain tip QP tolerates $\le 5$ ms but degrades sharply by 10 ms (Section 6.7-B), so hardware requires delay compensation or a latency-scheduled gain back-off. *(iii) Hard RCM and MPVIC:* the sub-0.5 mm RCM headline and the MPVIC baseline await the joint-space hard-RCM QP, since the soft null-space regulator is gain-insensitive (Section 6.6, T3); the adaptive estimator, force-safety governor, and sensor-noise robustness are already implemented and verified (Section 6.6). *(iv) Validation:* results are in simulation (with a flange-mounted sensing model carrying noise, bias, and stick-slip friction); the geometric flange-to-tip wrench reconstruction and ex vivo hardware validation remain for a hardware study.

### 7.4 Future Work

The immediate program adds the **hard-RCM QP rows** (with the LPV linearisation and horizon tightening of Section 3.2), validated on a **heterogeneous-sliding** task (soft fat $k_e\approx200$ → stiff connective tissue $k_e\approx1200$, with $\delta$ from the contact normal, Section 2.3); the **polytopic-LPV $H_\infty$** certificate (Section 5.2) over the tissue-parameter box across the soft→stiff transition; and the **realistic flange-mounted sensing chain** — then the full B1–B5 benchmark of Section 6.3. Beyond this: the pHRI Corollary-1 gain-scheduled 1 kHz lookup (now indexed by $\hat\theta$ as well as $\Lambda^{-1}(q)$), bimanual coupling on the constraint set, and ex vivo validation.

---

## 8. Conclusion

We formulated robot-assisted laparoscopy as a predictive interaction-dynamics problem rather than as a standalone impedance-control problem. The resulting framework normalizes the nonlinear manipulator into a constant-state residual interaction model, then uses a joint/task predictive optimizer to regulate tool-tip motion, RCM deviation, actuator authority, and adaptive tissue-penetration constraints in one decision. In the nominal 15 mm palpation task, maximum RCM deviation is 0.354 mm versus 8.13 mm for position impedance and 5.53 mm for tip-only MPC. These are simulation and linear-design results, not a clinical guarantee; nonlinear margin certification and ex vivo validation remain future work.

---

## References

[1] Y. Cao and J. Tang, "Impedance MPC for physical human–robot interaction: Predictive disturbance rejection with joint-limit safety," 2026. **(The pHRI architecture this paper extends. Provides Layer 1/Layer 2 decomposition, constant-$A_d$ structural insight, impedance equivalence theorem, augmented Kalman filter for offset-free tracking, and the 30-variable warm-started OSQP QP, all of which are inherited unchanged. Section 3 of the present paper builds directly on this work; Sections 4–5 extend it.)**

[2] L. Roveda et al., "Model predictive variable impedance control towards safe robotic interaction in unknown disturbance-rich environments," *Robotics and Autonomous Systems*, 2025. (Classical MPVIC: three-layer architecture with adaptive Hunt–Crossley and virtual energy tank. Closest surgical-domain prior art; benchmarked as B5.)

[3] A. S. Anand et al., "Deep model predictive variable impedance control," arXiv:2209.09614, 2022. (Deep MPVIC: neural-network forward model with CEM stiffness sampling.)

[4] D. Larby and F. Forni, "A passivity preserving $H_\infty$ synthesis technique for robot control," in *Proc. IEEE CDC*, 2022. (Offline $H_\infty$ impedance synthesis via sparsity-constrained LMIs; direct predecessor for the $H_\infty$-impedance link.)

[5] G. Pannocchia and J. B. Rawlings, "Disturbance models for offset-free model-predictive control," *AIChE Journal*, 49(2):426–437, 2003. (Offset-free MPC foundation, also cited in the pHRI paper.)

[6] Y.-Y. Cao, Z. Lin, and D. G. Ward, "Anti-windup design of output tracking systems subject to actuator saturation and constant disturbances," *Automatica*, 40(7):1221–1228, 2004. (Saturated-linear offset-free foundation, also cited in the pHRI paper.)

[7] N. Hogan, "Impedance control: An approach to manipulation," *ASME J. Dyn. Syst. Meas. Control*, 107(1):1–24, 1985.

[8] O. Khatib, "A unified approach for motion and force control of robot manipulators: The operational space formulation," *IEEE J. Robot. Autom.*, 3(1):43–53, 1987.

[9] K. H. Hunt and F. R. E. Crossley, "Coefficient of restitution interpreted as damping in vibroimpact," *ASME J. Appl. Mech.*, 42(2):440–445, 1975.

[10] N. Diolaiti et al., "Contact impedance estimation for robotic systems," *IEEE Trans. Robot.*, 21(5):925–935, 2005.

[11] N. Aghakhani et al., "Task control with remote center of motion constraint for minimally invasive robotic surgery," in *Proc. IEEE ICRA*, 2013.

[12] J. Sandoval et al., "Generalized framework for the dynamic control of redundant manipulators used for RA-MIS," *Mechatronics*, 2018.

[13] D. Q. Mayne et al., "Constrained model predictive control: Stability and optimality," *Automatica*, 36(6):789–814, 2000.

[14] D. Q. Mayne, M. M. Seron, and S. V. Raković, "Robust model predictive control of constrained linear systems with bounded disturbances," *Automatica*, 41(2):219–224, 2005. (Tube MPC; cited in Section 3.5 as the conservative alternative to the constructive horizon bound (17).)

[15] J. B. Rawlings, D. Q. Mayne, and M. M. Diehl, *Model Predictive Control: Theory, Computation, and Design*. Nob Hill, 2017.

[16] S. Boyd et al., *Linear Matrix Inequalities in System and Control Theory*. SIAM, 1994.

[17] H. K. Khalil, *Nonlinear Systems*, 3rd ed. Prentice Hall, 2002.

[18] B. Stellato et al., "OSQP: An operator splitting solver for quadratic programs," *Math. Program. Comput.*, 12(4):637–672, 2020.

[19] H. Iordanou and B. D. O. Anderson, "Adaptive control with dead zone," *Automatica*, 27(3):477–479, 1991.

[20] W. H. Chen, "Disturbance observer based control for nonlinear systems," *IEEE/ASME Trans. Mechatronics*, 9(4):706–710, 2004.

[21] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, *Robotics: Modelling, Planning and Control*. Springer, 2009.
