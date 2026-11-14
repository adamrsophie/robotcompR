# Manipulability and singularity analysis

Computes SVD-based diagnostics of the world-frame Jacobian (or a block
of it). Translational and rotational blocks have consistent units; the
`"full"` 6-row Jacobian mixes meters and radians, so measures computed
from it depend on the length unit and should be interpreted only with an
explicit scaling convention. The Yoshikawa measure is reported as \\w =
\prod_i \sigma_i\\ over the \\\min(m, n)\\ singular values of the
selected \\m \times n\\ block. When \\m \le n\\ this equals
\\\sqrt{\det(J J^\top)}\\; for a tall block (more task rows than joints,
such as the 3 translational rows of a planar arm) \\\det(J J^\top)\\ is
always zero, and the product of singular values measures the volume of
the reachable velocity set within the column space instead. Select the
meaningful rows with `rows` (for a planar arm, `rows = 1:2`) to get the
classical value. The condition number is \\\sigma\_{max} /
\sigma\_{min}\\ (`Inf` when the rank is deficient). The rank is the
number of singular values above `tol * sigma_max`.

## Usage

``` r
robot_manipulability(
  robot,
  q,
  frame = NULL,
  measure = c("translational", "rotational", "full"),
  rows = NULL,
  tol = 1e-08
)
```

## Arguments

- robot:

  A `robot_model`.

- q:

  Configuration.

- frame:

  Frame name (`NULL` for the end effector).

- measure:

  `"translational"`, `"rotational"`, or `"full"`.

- rows:

  Optional integer vector of Jacobian rows to keep instead of `measure`.

- tol:

  Relative tolerance for rank and singularity decisions.

## Value

A `robot_manipulability` list with `singular_values`, `rank`,
`yoshikawa`, `condition`, `inverse_condition`, `singular` (logical),
`ellipsoid` (a list with `axes`, the left singular vectors, and `radii`,
the singular values) and `rows`.

## Details

Rows that are identically zero for the robot (for example out-of-plane
rows of a planar arm) can be removed with `rows`.

## Examples

``` r
robot <- robot_example("planar2")
robot_manipulability(robot, c(0.3, 0), rows = 1:2)
#> <robot_manipulability> custom rows (vx, vy)
#> singular values: 2.2361, 1.0641e-16
#> rank 1, Yoshikawa 2.37933e-16, condition Inf, SINGULAR
robot_manipulability(robot, c(0.3, 1), rows = 1:2)$yoshikawa
#> [1] 0.841471
```
