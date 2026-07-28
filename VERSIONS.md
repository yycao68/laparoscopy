# Laparoscopy paper versions

## v0

`v0/` is an immutable snapshot of the original paper, simulation source,
generated metrics, figures, and PDFs before the June 25, 2026 technical audit.

Build:

```bash
cd v0/paper
latexmk -pdf laparoscopy_ieee.tex
```

## v1

`v1/` contains the corrected implementation and claim-aligned paper:

- exact-ZOH input terms;
- one joint/task QP with decision-coupled RCM rows;
- adaptive Hunt--Crossley penetration rows in OSQP;
- deep-reference feasibility backstop;
- measured solver timing and constraint ablations;
- local, explicitly limited contact-channel robustness analysis.

Run and build:

```bash
cd v1/simulation
python3 lap_benchmark.py
python3 lap_verify.py
python3 lap_experiments.py

cd ../paper
latexmk -pdf laparoscopy_ieee.tex
```
