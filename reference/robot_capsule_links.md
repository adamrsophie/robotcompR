# Collision geometry for planar DH arms

Adds a capsule along every link of a serial DH model whose links lie
along their x axes (as in the planar examples), from the parent frame
origin to the link frame origin.

## Usage

``` r
robot_capsule_links(robot, radius)
```

## Arguments

- robot:

  A `robot_model` built by
  [`robot_dh()`](https://adamrsophie.github.io/robotcompR/reference/robot_dh.md)
  or
  [`robot_example()`](https://adamrsophie.github.io/robotcompR/reference/robot_example.md).

- radius:

  Capsule radius.

## Value

A `robot_model`.

## Examples

``` r
robot <- robot_capsule_links(robot_example("planar2"), 0.05)
```
