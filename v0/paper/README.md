# IEEE Submission — RCM-Constrained Impedance MPC for Force-Safe Robotic Laparoscopy

IEEE two-column rewrite of `../laparoscopy.md` (2026-06-12), following
the same two-wrapper pattern as `dexterous_hand/paper/`.

## Files

- `body.tex` — the full paper content (abstract → bibliography), **shared** by both
  wrappers; edit the paper here.
- `laparoscopy_ieee.tex` — journal-submission wrapper, **anonymized** for
  double-anonymous review, page numbers suppressed for PaperPlaza margin compliance.
  Build: `latexmk -pdf laparoscopy_ieee.tex`
- `laparoscopy_arxiv.tex` — arXiv preprint wrapper with author information
  (Cao; affiliations TODO). Build: `latexmk -pdf laparoscopy_arxiv.tex`
- `figures/` — `lap_benchmark.png`, `lap_verify.png`, `lap_experiments.png`, copied
  from `../simulation/`.

## Key editorial decisions vs. the markdown draft

1. **Projected tables removed.** The markdown's Tables 6.3 (B1–B5 full-design
   benchmark), 6.4 (ablation), and 6.5 (horizon stress sweep) are explicitly labeled
   *projected/targets* there; tables of projected numbers are not submittable. The
   IEEE version reports only the **measured** results (Table I palpation benchmark,
   T1–T4 verification, robustness Tables II–IV, offline H∞/H₂ numbers) and describes
   the full B1–B5 benchmark + hard-RCM rows as the planned validation protocol
   (Discussion). Restore the projected tables only when measured.
2. **Theorem numbering is fresh**: Lemma 1 (horizon growth), Theorem 1 (horizon
   criterion; md's "Theorem 2"), Theorem 2 (force safety; md's "Theorem 3"). Results
   of the base paper are cited as results of [1], not numbered here.
3. **Abstract** rewritten to measured claims, 196 words (the md's "projected 73%
   peak-force cut" claim is omitted).
4. **Reference [19]** of the md ("Iordanou & Anderson, Adaptive control with dead
   zone, Automatica 1991") could not be verified and was replaced by Peterson &
   Narendra, "Bounded error adaptive control," IEEE TAC 1982 (the classic dead-zone
   reference), bib key `iordanou1991` kept.
5. Double-anonymous: base paper [1] (arXiv:2606.08281, Cao & Tang) cited in third
   person throughout; author block "Anonymous Author(s)" in `laparoscopy_ieee.tex`.

## Before submission (TODO)

- Choose the venue (T-MRB / RA-L / T-RO) and check its page limit (currently 9 pp).
- Add affiliations in `laparoscopy_arxiv.tex` before arXiv upload.
