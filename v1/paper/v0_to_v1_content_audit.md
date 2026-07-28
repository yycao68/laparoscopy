# v0 to v1 content audit

This audit compares `laparoscopy/v0/paper/body.tex` and
`laparoscopy/v1/paper/body.tex`.

## Size change

- v0 IEEE PDF: 10 pages, about 8,926 extracted words.
- v1 IEEE PDF: 8 pages, about 6,335 extracted words.
- Source length did not shrink in the same way: v0 `body.tex` has 564 lines,
  while v1 `body.tex` has 572 lines.

Therefore the page reduction is mainly from prose compression, shorter
bibliography, and replacement of long theoretical paragraphs with compact
corrected statements, not from a simple line-count deletion.

## Content removed or compressed from v0

### 1. Explicit Kalman subsection

v0 had a dedicated subsection:

- `Kalman Disturbance Augmentation`, label `sec:kalman`
- equation label `eq:aug`

v1 no longer has this standalone subsection. The Kalman/disturbance story is
compressed into the disturbance, oscillator, implementation, and discussion
sections. This is a real content compression. If page budget allows, restoring a
short standalone Kalman subsection would improve continuity with the parent pHRI
paper.

### 2. Terminal-set constraint in the online QP

v0 included:

- terminal constraint label `eq:terminal_con`
- explicit statement `xi_N in X_f`
- text claiming recursive feasibility/stability through a terminal set

v1 removed the explicit terminal constraint from the implemented QP and replaced
it with a more accurate statement: the current implementation uses a scaled
terminal penalty and fallback, while an explicit robust terminal set remains
future work. This was a correction, not just a deletion, because the implemented
controller does not yet compute a numerical invariant terminal set online.

### 3. Separate gamma-to-force subsection

v0 had:

- `From the gamma-Bound to the Force Constraint`, label `sec:gamma`
- the chain from induced gain to peak force via
  `||F_ext||_inf <= ||z||_inf <= ||z||_2 <= gamma wbar`

v1 keeps the core budget relation through `eq:gamma_bound` and `eq:budget`, but
does not keep the full explanatory subsection. This is a real compression. The
argument is still present but less pedagogical.

### 4. Force-safety proposition wording

v0 had:

- proposition label `thm:force`
- stronger wording: "Force Safety as Positive Invariance"
- realized force statement with an explicit margin `Delta`

v1 replaced it with:

- proposition label `prop:force_invariance`
- "Design-level force invariance"
- a more conditional statement tied to validated residual bounds, tightened
  rows, and common-P storage

This is a correction in evidence level. The v0 version was stronger and more
complete rhetorically, but it depended on an explicit terminal set and quantified
nonlinear margin that are not yet fully computed. The v1 version keeps the
invariance concept without overstating hardware/nonlinear certification.

### 5. Offline H2/H-infinity numerical subsection

v0 had a separate subsection:

- `Offline H_infinity/H_2 Design, Computed`, label `sec:hinfnum`

v1 moved those numerical results into the main H2/H-infinity design section and
the conclusion. The content is mostly restored, with updated numbers:

- H2 spectral radius: 0.8182
- MPC/LQR gain match: 9.78e-10
- H-infinity minimal gamma: 1.0081
- deploy gamma: 1.5122
- common-P LMI gamma: 1.480
- common-P max spectral radius: 0.9936

This is not currently missing, but it is less visible than the v0 standalone
subsection.

### 6. Some old mechanism-test numbers were replaced

v0 emphasized:

- offset-free steady-state tip error falling from 3.94 mm to 0.03 mm
- force governor: 3.5 N at a 3 N cap versus 5.4 N push-through
- RLS stiffness estimate 485.8 N/m, 2.8 percent error
- soft null-space RCM gain-insensitivity around 3.57 mm

v1 replaces these with the current total-command/decision-coupled implementation
results:

- nominal RCM max 0.354 mm
- force peak 1.18 N in the nominal benchmark
- deep-reference force limitation 1.46 N versus 7.69 N disabled
- RLS estimate 476.3 N/m, 4.7 percent error
- decision-coupled versus soft RCM: 0.362 mm versus 3.522 mm

This is mostly a correction to match current code, not an accidental removal.

### 7. References removed from bibliography

v0 bibliography had 18 entries. v1 has 11. Removed keys:

- `larby2022` -- relevant to H-infinity impedance/passivity
- `pannocchia2003` -- relevant to offset-free MPC disturbance models
- `cao2004antiwindup` -- relevant to saturated offset-free output tracking
- `mayne2000` -- relevant to constrained MPC stability/terminal sets
- `mayne2005tube` -- relevant to tube MPC/tightening
- `rawlings2017` -- relevant to MPC textbook support
- `osqp` -- relevant to QP solver attribution

Given that v1 restored H-infinity, invariance, disturbance-model, and QP solver
claims, at least `larby2022`, `pannocchia2003` or `rawlings2017`, `mayne2000`,
and `osqp` should probably be restored unless page budget is critical.

## Content added in v1 that was not in v0

- PID-plus-force baseline.
- Eight randomized repeated trials.
- Monte Carlo figure.
- Multi-condition sweep figure.
- Total feedforward-plus-MPC actuator constraint.
- Exact-ZOH input correction.
- Explicit high-delay fallback/supervisor.
- Common-P LMI script and certificate in v1 simulation.
- More conservative language about clinical safety and nonlinear invariance.

## Bottom line

The removals were not all random. Some were legitimate corrections because v0
claimed terminal-set/recursive-feasibility/nonlinear safety more strongly than
the implementation currently proves. However, v1 did over-compress several
important explanatory parts from v0:

1. standalone Kalman disturbance augmentation;
2. gamma-to-force explanation;
3. standalone offline H2/H-infinity numerical subsection;
4. supporting references for H-infinity, offset-free MPC, MPC stability, tube
   MPC, and OSQP.

## Repair completed

The recommended repair has been applied to `body.tex`:

- restored a compact standalone Kalman disturbance augmentation subsection;
- restored the constrained-realization explanation for the \(H_2/H_\infty\)
  design;
- restored a separate offline \(H_2/H_\infty\) numerical subsection;
- restored the key references for \(H_\infty\) impedance, offset-free MPC,
  constrained/tube MPC, anti-windup under saturation, and OSQP;
- kept the corrected v1 evidence level: the common-P result is a linear
  post-feedforward certificate, while nonlinear clinical safety remains future
  validation.

After repair, the IEEE and arXiv PDFs both build to 9 pages.
