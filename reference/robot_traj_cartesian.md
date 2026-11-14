# Task-space (Cartesian) trajectories

Interpolates between two poses with a quintic time scaling \\s(t)\\: the
position moves along the straight line and the orientation along the
shortest great circle
([`quaternion_slerp()`](https://adamrsophie.github.io/robotcompR/reference/quaternion_slerp.md)).
Use
[`robot_ik()`](https://adamrsophie.github.io/robotcompR/reference/robot_ik.md)
at each sample to obtain joint configurations.

## Usage

``` r
robot_traj_cartesian(from, to, duration, n = 51)
```

## Arguments

- from, to:

  Start and end `robot_pose` objects (SE(3)).

- duration:

  Duration in seconds.

- n:

  Number of samples.

## Value

A list with `time`, `s` and `poses` (a list of `robot_pose`).

## Examples

``` r
ct <- robot_traj_cartesian(robot_pose(), robot_pose(rotz(1), c(1, 0, 0)),
                           duration = 1, n = 3)
pose_translation(ct$poses[[2]])
#> [1] 0.5 0.0 0.0
```
