# Write a robot model as URDF

Exports links (inertial parameters and geometry), joints and limits.
URDF has no joint offset or post-motion transform, so the export keeps
the kinematics exact by folding an offset into the joint origin and by
splitting a joint with a post transform into the moving joint, a
massless intermediate link named `<child>_motion`, and a fixed joint
named `<joint>_post` (a numeric suffix is added if a name is taken). The
extra joints can change the breadth-first order of the movable joints;
when the reimported configuration order differs, a warning gives both
orders. Names are XML-escaped. Unspecified velocity and effort limits
are written as 0, which
[`robot_read_urdf()`](https://adamrsophie.github.io/robotcompR/reference/robot_read_urdf.md)
reads back as unspecified. Forward kinematics of every original frame is
preserved; the exported model has extra frames. Convex-hull geometry has
no URDF equivalent and is omitted with a warning.

## Usage

``` r
robot_write_urdf(robot, file = NULL)
```

## Arguments

- robot:

  A `robot_model`.

- file:

  Output path, or `NULL` to return the XML string.

## Value

The URDF text, invisibly when written to a file.

## Examples

``` r
robot <- robot_example("planar2")
txt <- robot_write_urdf(robot)
back <- robot_read_urdf(txt)
pose_translation(robot_fk(back, c(pi / 2, pi / 2), "link2"))
#> [1] -1  1  0
```
