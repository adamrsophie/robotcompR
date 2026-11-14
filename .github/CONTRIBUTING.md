# Contributing to robotcompR

Thank you for your interest in robotcompR.

* Report bugs and request features in the
  [issue tracker](https://github.com/adamrsophie/robotcompR/issues). A minimal
  reproducible example (mesh size, problem definition, the call that fails)
  helps a lot.
* Numerical changes need a validation test: a manufactured solution with an
  observed convergence order, or an exact reproduction of a polynomial
  solution. Keep routine tests small (meshes up to 32 by 32) so that
  `R CMD check` stays fast.
* Work on a branch and open a pull request against `main`. The R-CMD-check
  workflow must pass on Linux, macOS and Windows.
* Documentation uses roxygen2 with markdown. Run `devtools::document()`
  before committing changes to roxygen comments.
