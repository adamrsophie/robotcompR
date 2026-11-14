# Static environments

An environment is a named list of world-frame obstacles. Geometry
origins are interpreted in the world frame.

## Usage

``` r
robot_environment(...)
```

## Arguments

- ...:

  `robot_geometry` objects, optionally named.

## Value

A `robot_environment`.

## Examples

``` r
env <- robot_environment(wall = geom_box(c(0.2, 2, 1),
                                         origin = transl(1.5, 0, 0)))
env
#> <robot_environment> 1 obstacle
#>   wall: <robot_geometry> box: size (0.2, 2, 1), at (1.5, 0, 0)
```
