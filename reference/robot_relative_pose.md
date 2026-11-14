# Relative pose between two frames

Given the poses of frames `a` and `b` in a common frame `w`, returns
\\{}^{a}T\_{b} = ({}^{w}T\_{a})^{-1}\\{}^{w}T\_{b}\\.

## Usage

``` r
robot_relative_pose(from, to)
```

## Arguments

- from:

  Pose of frame `a`.

- to:

  Pose of frame `b`.

## Value

A `robot_pose`.

## Examples

``` r
A <- robot_pose(translation = c(1, 0, 0))
B <- robot_pose(translation = c(1, 1, 0))
pose_translation(robot_relative_pose(A, B))
#> [1] 0 1 0
```
