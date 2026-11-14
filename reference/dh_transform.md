# Denavit-Hartenberg transform of one link

Standard convention: \\T_i = R_z(\theta_i) T_z(d_i) T_x(a_i)
R_x(\alpha_i)\\. Modified (Craig) convention: \\T_i = R_x(\alpha\_{i-1})
T_x(a\_{i-1}) R_z(\theta_i) T_z(d_i)\\, where the arguments `a` and
`alpha` hold \\a\_{i-1}\\ and \\\alpha\_{i-1}\\.

## Usage

``` r
dh_transform(a, alpha, d, theta, convention = c("standard", "modified"))
```

## Arguments

- a, alpha, d, theta:

  DH parameters (meters and radians).

- convention:

  `"standard"` or `"modified"`.

## Value

A `robot_pose`.

## Examples

``` r
dh_transform(a = 1, alpha = 0, d = 0, theta = pi / 2)
#> <robot_pose> SE(3)
#>      [,1] [,2] [,3] [,4]
#> [1,]    0   -1    0    0
#> [2,]    1    0    0    1
#> [3,]    0    0    1    0
#> [4,]    0    0    0    1
```
