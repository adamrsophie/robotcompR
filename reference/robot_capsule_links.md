# Capsule collision geometry for DH arms

Adds a capsule along every DH link, with its core segment joining the
origins of consecutive DH frames, so offsets along both \\a\\ and \\d\\
are covered (for example the PUMA 560 forearm). For the standard
convention the segment is fixed in the child link frame and is attached
there; for the modified convention it is fixed in the parent frame and
is attached to the parent link (the base for the first joint). Segments
of zero length are skipped. Prismatic joints change the segment length
with the joint value, so they get no capsule and a warning names them:
attach geometry to those links yourself.

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
