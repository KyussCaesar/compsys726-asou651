# COMPSYS 726 Assignment

#### Antony Southworth

# Getting Started

This project has been written in the [Rust](https://www.rust-lang.org/en-US/)
programming language. You can follow the link to for installation instructions.
Keeping all of the defaults should be fine; i.e use the latest "stable"
toolchain.

## Breifly, What is Rust?

From the latest issue of [This Week in
Rust](https://this-week-in-rust.org/blog/2018/05/22/this-week-in-rust-235/):

> Rust is a systems language pursuing the trifecta: safety, concurrency and
> speed.

Rust is one of my favourite languages, because it lets you write code that
competes with C/C++ for speed, but is memory-safe *without a garbage collector*.
Rust makes use of compile-time checks to remove the three most common bugs in
C/C++ code:

* Use-after-free.
* Double-free.
* Dereferencing a NULL pointer.

As a nifty bonus; these rules also mean that code containing race-conditions
*won't compile*. Rust calls this "Fearless Concurrency".

It also has a really nice build tool and package manager, `cargo`, which can
fetch, build and publish packages, which in Rust are called "crates".


# Workspace Structure and Contents

This project is set up as both a `catkin` workspace as well as a `cargo`
workspace. Rust crates can be found in the respective folders in the project
root, and the `catkin` packages can be found in the `src/` subdirectory, just
like any other `catkin` package.


## Rust Crates

This workspace contains the following crates:

### `common` (library crate)

Exports a number of utilities that are shared across the other crates.


### `obstacle-detection` (binary crate)

Contains the `obstacle-detection` node, which listens on `/map` and prints
information about the detected obstacles to the console.


### `pathfinding` (binary crate)

Contains the `pathfinding` node, which listens on `/map` and `/ropose` and emits
`geometry_msgs/Twist` on `/cmd_vel`. This node is responsible for pathfinding
and ensuring that the whole warehouse is explored. Or at least, it would have
been, if my code had worked.


## `catkin` Packages

This workspace contains a single `catkin` package: `ropose`, which contains a
single node of the same name. This node simply listens on `/tf` and publishes
the robot pose w.r.t the map frame on topic `/ropose`.


# Building

Before building anything, make sure that you have sourced
`/opt/ros/kinetic/setup.bash` and `devel/setup.bash`.


## Dependencies

Requires ROS Kinetic (duh) and the latest version of stable Rust (also duh).


### Rust Crates

Use the following command to build the Rust crates.

```
cargo build --release
```

`cargo` is the Rust package manager and build tool, and should be installed
along with the main language installation from above. If the command doesn't
work, ensure that `~/.cargo/bin` is on your PATH. The Rust installer should do
this for you, but you may need to restart your terminal first.


### `ropose` (`catkin` package)

Build this like any other `catkin` package. Make sure that you are in the root
of the project, and run `catkin_make`.



# Running

The following command can be used to start `ropose`:

```
rosrun ropose ropose
```

Symlinks to the Rust nodes (`obstacle-detection` and `pathfinding`) are located
in the `nodes/` subdirectory and can be started like so.

```
./nodes/<node-name>
```

# Documentation

Documentation for each crate (and it's dependencies) can be found online:

[`obstacle-detection`](https://asou651.bitbucket.io/compsys726/doc/obstacle_detection/index.html)

[`pathfinding`](https://asou651.bitbucket.io/compsys726/doc/pathfinding/index.html)

[`common`](https://asou651.bitbucket.io/compsys726/doc/common/index.html)

