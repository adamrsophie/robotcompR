# Closed-form inverse kinematics of a planar two-link arm

For link lengths \\l_1, l_2\\ and target \\(x, y)\\, \\c_2 = (x^2 +
y^2 - l_1^2 - l_2^2) / (2 l_1 l_2)\\. When \\\|c_2\| \le 1\\ the
solutions are \\q_2 = \pm\arccos c_2\\ and \\q_1 = \mathrm{atan2}(y,
x) - \mathrm{atan2}(l_2 \sin q_2, l_1 + l_2 \cos q_2)\\. At the
workspace boundary (\\\|c_2\| = 1\\) the two solutions coincide and one
row is returned. Unreachable targets return a matrix with zero rows. At
the origin with \\l_1 = l_2\\ every \\q_1\\ is a solution; the function
returns \\q_1 = 0\\.

## Usage

``` r
ik_planar2(x, y, l1, l2, tol = 1e-12)
```

## Arguments

- x, y:

  Target position.

- l1, l2:

  Link lengths.

- tol:

  Tolerance for the reachability test.

## Value

A matrix with columns `q1`, `q2` and rows `q2_positive` and
`q2_negative`.

## Examples

``` r
ik_planar2(-1, 1, 1, 1)
#>                    q1        q2
#> q2_positive  1.570796  1.570796
#> q2_negative -3.141593 -1.570796
ik_planar2(3, 0, 1, 1)
#>      q1 q2
```
