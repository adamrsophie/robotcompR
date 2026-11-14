# Inertia tensors of simple solids

Rotational inertia about the center of mass for uniform solids, in the
body axes stated: a rod of length `length` along `axis`, a solid cuboid
with edge lengths `size`, a solid cylinder of radius `radius` and height
`length` along z, and a solid sphere.

## Usage

``` r
inertia_rod(mass, length, axis = c("x", "y", "z"))

inertia_box(mass, size)

inertia_cylinder(mass, radius, length)

inertia_sphere(mass, radius)
```

## Arguments

- mass:

  Mass in kilograms.

- length:

  Length in meters.

- axis:

  For rods, `"x"`, `"y"` or `"z"`.

- size:

  Edge lengths c(x, y, z).

- radius:

  Radius in meters.

## Value

A 3 by 3 inertia matrix.

## Examples

``` r
inertia_rod(1, 0.5)
#>      [,1]       [,2]       [,3]
#> [1,]    0 0.00000000 0.00000000
#> [2,]    0 0.02083333 0.00000000
#> [3,]    0 0.00000000 0.02083333
inertia_box(2, c(0.1, 0.2, 0.3))
#>            [,1]       [,2]        [,3]
#> [1,] 0.02166667 0.00000000 0.000000000
#> [2,] 0.00000000 0.01666667 0.000000000
#> [3,] 0.00000000 0.00000000 0.008333333
```
