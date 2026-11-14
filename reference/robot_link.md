# Robot links

A link is a rigid body with an attached frame. Inertial parameters are
optional: `mass` in kilograms, `com` the center of mass in the link
frame (meters), and `inertia` the 3 by 3 rotational inertia about the
center of mass, expressed in axes parallel to the link frame (kg m^2).
Inertia matrices must be symmetric positive semidefinite and satisfy the
triangle inequality of principal moments.

## Usage

``` r
robot_link(name, mass = 0, com = c(0, 0, 0), inertia = matrix(0, 3, 3))
```

## Arguments

- name:

  Link name.

- mass:

  Mass (nonnegative).

- com:

  Center of mass in the link frame.

- inertia:

  Inertia about the center of mass.

## Value

A `robot_link` object.

## Examples

``` r
robot_link("upper_arm", mass = 2, com = c(0.25, 0, 0),
           inertia = diag(c(0.001, 0.04, 0.04)))
#> <robot_link> upper_arm: mass 2 kg, com (0.25, 0, 0)
```
