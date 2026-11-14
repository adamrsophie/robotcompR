## R CMD check results

0 errors | 0 warnings | 1 note

* This is a new submission.

## Test environments

* GitHub Actions: macOS (R release), Windows (R release), Ubuntu (R release
  and R devel), with `--as-cran`.
* GitHub Actions: Ubuntu (R devel) with the CRAN incoming checks
  (`_R_CHECK_CRAN_INCOMING_=true`, remote URL and DOI checks) and the PDF
  manual built.
* GitHub Actions: Ubuntu (R release) with only hard dependencies installed
  (`_R_CHECK_DEPENDS_ONLY_=true`).

## Notes for the reviewer

* The incoming check lists possibly misspelled words in the Description.
  They are correct: Sola, Deray, Atchuthan, Denavit, Hartenberg,
  Featherstone, Gilbert, Keerthi, Kuffner, LaValle, Karaman, Frazzoli and
  Kavraki are author names, Kalman is a proper name and "et al." is Latin; GJK, EPA, RRT, PRM, EKF and UKF are acronyms
  spelled out where they first appear; Jacobians, manipulability,
  quaternions, revolute, polytope and roadmaps are standard robotics terms.
* The slowest example takes under half a second; none are wrapped in
  `\dontrun{}`.
* `robot_ik()` draws its random restarts from a fixed default `seed` so
  results are reproducible. It saves and restores `.Random.seed`, so the
  user's random number stream is never changed; a test checks this.
* The package writes files only when the user passes a path, and tests use
  `tempfile()`.
