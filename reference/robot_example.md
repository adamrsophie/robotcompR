# Example robot models

Returns validated example models. All are serial manipulators built from
standard Denavit-Hartenberg parameters, with link frames at the distal
end of each link.

## Usage

``` r
robot_example(name = c("planar2", "planar1", "planar3", "puma560", "branched"))
```

## Arguments

- name:

  Model name.

## Value

A `robot_model`.

## Details

- `"planar1"`: one revolute link of length 1 m, a uniform 1 kg rod.

- `"planar2"`: two revolute links of length 1 m, uniform 1 kg rods,
  limits \\\[-\pi, \pi\]\\. End effector \\(\cos q_1 + \cos(q_1 + q_2),
  \sin q_1 + \sin(q_1 + q_2))\\.

- `"planar3"`: three revolute links of lengths 1, 0.8 and 0.6 m with
  masses 1, 0.8 and 0.6 kg.

- `"puma560"`: kinematic model of the PUMA 560 arm using the standard DH
  parameters and joint limits published with the Robotics Toolbox
  (Corke, 2017, *Robotics, Vision and Control*, 2nd ed., Springer). No
  inertial parameters are included.

- `"branched"`: a five-link tree with two fingers on a revolute wrist,
  including a fixed joint, used to demonstrate branched kinematics.

## Examples

``` r
robot_example("planar2")
#> <robot_model> planar2: 3 links, 2 joints, 2 degrees of freedom
#> end effector: link2
#>    name     type parent child q_index     lower    upper
#>  joint1 revolute   base link1       1 -3.141593 3.141593
#>  joint2 revolute  link1 link2       2 -3.141593 3.141593
robot_example("puma560")
#> <robot_model> puma560: 7 links, 6 joints, 6 degrees of freedom
#> end effector: link6
#>    name     type parent child q_index      lower     upper
#>  joint1 revolute   base link1       1 -2.7925268 2.7925268
#>  joint2 revolute  link1 link2       2 -0.7853982 3.9269908
#>  joint3 revolute  link2 link3       3 -3.9269908 0.7853982
#>  joint4 revolute  link3 link4       4 -1.9198622 2.9670597
#>  joint5 revolute  link4 link5       5 -1.7453293 1.7453293
#>  joint6 revolute  link5 link6       6 -4.6425758 4.6425758
```
