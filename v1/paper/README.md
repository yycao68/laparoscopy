# Laparoscopy v1 paper

Current title: **A Predictive Interaction Dynamics Framework for Safe Robot-Assisted Laparoscopy with Remote Center-of-Motion Constraints**

The v1 manuscript is claim-aligned with `../simulation/`.

Build:

```bash
latexmk -pdf laparoscopy_ieee.tex
latexmk -pdf laparoscopy_arxiv.tex
```

The title uses **safe** to describe the framework objective, while the text
keeps the reported 3 N value as an engineering simulation threshold rather than
a clinical tissue-damage limit.

The safety, invariance, oscillator, and \(H_\infty\) material is retained in
v1, but stated at the correct evidence level:

- the online benchmark uses the constrained joint/task QP;
- the oscillator is implemented and stress-tested, although the current
  slow-respiration case shows negligible measured benefit;
- `../simulation/lap_hinf_design.py` computes the H2/H-infinity state-feedback
  design and unconstrained MPC/LQR gain match;
- `../simulation/lap_commonP_sdp.py` provides a common-P LMI certificate for
  the linear post-feedforward residual plant;
- nonlinear clinical safety/invariance is not claimed without hardware/ex vivo
  validation and quantified nonlinear residual margins.
