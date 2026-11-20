# Contributing to robotcompR

Thank you for your interest in robotcompR.

- Report bugs and request features in the [issue
  tracker](https://github.com/adamrsophie/robotcompR/issues). A minimal
  reproducible example (the robot or geometry, the configuration and the
  call that fails) helps a lot.
- Numerical changes need a validation test with an independent expected
  value: an analytic solution, a dense reference computation, or a
  property such as invariance under a rigid motion or a joint
  permutation. Do not test a result against the formula the
  implementation itself uses. Keep routine tests small so that
  `R CMD check` stays fast.
- Joint arrays follow one rule everywhere: unnamed values are in
  [`robot_joint_names()`](https://adamrsophie.github.io/robotcompR/reference/robot_introspection.md)
  order; named values must carry exactly the joint names and are matched
  by name. For example `robot_fk(robot, c(joint2 = 0.1, joint1 = 0.5))`
  equals `robot_fk(robot, c(0.5, 0.1))`. New functions that take joint
  arrays should validate them with the same internal checks.
- Work on a branch and open a pull request against `main`. The
  R-CMD-check workflow must pass on Linux, macOS and Windows and on the
  minimum supported R version.
- Documentation uses roxygen2 with markdown. Run `devtools::document()`
  before committing changes to roxygen comments.
