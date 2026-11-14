# Attach collision geometry to a link

Returns a copy of `robot` with `geometry` added to the collision
geometry of `link`. The geometry origin is relative to the link frame.

## Usage

``` r
robot_attach_geometry(robot, link, geometry)
```

## Arguments

- robot:

  A `robot_model`.

- link:

  Link name.

- geometry:

  A `robot_geometry`.

## Value

A `robot_model`.

## Examples

``` r
robot <- robot_example("planar2")
robot <- robot_attach_geometry(robot, "link1",
  geom_capsule(0.05, 1, origin = transl(-0.5, 0, 0) * troty(pi / 2)))
robot_links(robot)
#>    name parent mass n_collision
#> 1  base   <NA>    0           0
#> 2 link1   base    1           1
#> 3 link2  link1    1           0
```
