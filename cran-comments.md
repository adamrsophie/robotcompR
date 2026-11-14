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

* Words such as Denavit, Hartenberg, Featherstone, Keerthi, Kuffner,
  LaValle, Karaman, Frazzoli and Kavraki in the Description are author
  names and method names, not misspellings.
* All examples run in well under five seconds each; none are wrapped in
  `\dontrun{}`.
* The package writes files only when the user passes a path, and tests use
  `tempfile()`.
