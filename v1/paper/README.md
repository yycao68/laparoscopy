# Laparoscopy v1 paper

Current title: **Predictive Interaction Dynamics for Robot-Assisted Laparoscopy with Remote Center-of-Motion and Tissue-Force Constraints**

The v1 manuscript is implementation-aligned with `../simulation/`. Stored
JSON, figures, and PDFs predate the latest controller corrections, so the
maintained body excludes the archived quantitative block by default until a
complete regeneration is available.

Build:

```bash
latexmk -pdf laparoscopy_ieee.tex
latexmk -pdf laparoscopy_arxiv.tex
```

The force, oscillator, and \(H_\infty\) material is stated at the implemented
evidence level:

- the online benchmark uses the constrained joint/task QP;
- the oscillator free response is connected to the deployed joint/task QP and
  covered by focused regression tests; detuning metrics await regeneration;
- `../simulation/lap_hinf_design.py` computes the H2/H-infinity state-feedback
  design and unconstrained MPC/LQR gain match;
- `../simulation/lap_commonP_sdp.py` provides a common-P weighted state/control
  certificate for a separate six-state linear tip model;
- tissue-force invariance, recursive feasibility, and clinical safety are not
  claimed without a contact-force output, terminal set, quantified nonlinear
  margins, and hardware or ex vivo validation.

Do not define `\includeStalePreCorrectionResults` for a submission build. That
switch exists only to preserve the pre-correction tables and figures as an
auditable development record.

`body_short_verified.tex`, `../laparoscopy.md`, and `../laparoscopy_zh.md` are
archived working drafts and are not included by either maintained wrapper.
