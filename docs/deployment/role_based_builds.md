# Role-Based Builds

## Purpose

Robot Savo uses one ROS 2 workspace on multiple computers, but each computer builds only the packages assigned to its deployment role. The role build scripts are the supported build entry points for the Core Pi and Edge Pi.

The authoritative package lists are defined in:

deploy/core/env_core.sh
deploy/edge/env_edge.sh

Do not maintain independent package lists in documentation, CI scripts, or local helper scripts. Changes to role membership must be made in the corresponding environment file and validated through the normal role build.

Supported targets

| Role | Expected hostnames | Build script | Package count |
| --- | --- | --- | --- |
| Core | core, savo-core | deploy/core/build_core.sh | 14 |
| Edge | edge, savo-edge | deploy/edge/build_edge.sh | 10 |
| Observer | Operator workstation | deploy/observer/build_observer.sh or package-up-to build | Separate |

The hostname checks are intentional safeguards. They reduce the chance of installing hardware-facing packages on the wrong computer.

Prerequisites

Before building either role:

Use Ubuntu 24.04 with ROS 2 Jazzy installed.
Clone the repository to the intended SAVO_ROOT location.
Confirm at least 8 GiB of free disk space.
Install the role dependencies.
Confirm the workspace contains every package required by the role.
Stop any manual ROS sessions that source a different workspace overlay.

Install dependencies with:

cd ~/Savo_Pi
bash deploy/common/install_role_deps.sh --role core

or:

cd ~/Savo_Pi
bash deploy/common/install_role_deps.sh --role edge

The dependency installer:

requires Ubuntu 24.04;
expects ARM64 unless SAVO_ALLOW_PC_INSTALL=true is intentionally set for a development computer;
installs common deployment tools and role-specific system packages;
initializes or updates rosdep as needed;
runs rosdep install only for the source paths used by the selected role;
prepares Edge runtime socket directories when the Edge role is selected;
writes deployment logs under ${SAVO_ROOT}/log/deploy unless overridden.
Core build

Run on the Core Pi:

cd ~/Savo_Pi
bash deploy/core/build_core.sh --clean --test

The Core script builds the package set from SAVO_CORE_BUILD_PACKAGES:

savo_msgs
savo_description
savo_bringup
savo_perception
savo_power
savo_supervisor
savo_base
savo_control
savo_lidar
savo_localization
savo_locations
savo_mapping
savo_nav
savo_head
Core options

| Option | Behavior |
| --- | --- |
| --clean | Removes build/, install/, and log/ before building |
| --test or --tests | Runs colcon test and prints the complete test-result summary |
| --allow-missing | Skips missing packages; permitted only for staged development, never for release qualification |

A production or release build must not use --allow-missing.

Edge build

Run on the Edge Pi:

cd ~/Savo_Pi
bash deploy/edge/build_edge.sh --clean --test

The Edge script builds the package set from SAVO_EDGE_BUILD_PACKAGES:

savo_msgs
savo_description
savo_bringup
savo_bridge
savo_perception
savo_power
savo_realsense
savo_speech
savo_ui
savo_vo

The Edge script fails if any required package is absent. It does not provide an --allow-missing mode.

Build behavior

Both role scripts:

source the configured ROS 2 distribution;
verify the expected target hostname;
inspect the workspace with colcon list;
build only the packages assigned to the role;
use --symlink-install for the normal development/target workflow;
source the completed role install before tests;
return a non-zero status when a required command, directory, package, build, or test fails.

The build scripts do not launch the robot, enable systemd services, apply network changes, or authorize motion.

Pass criteria

A role build passes only when:

the hostname check passes;
every required role package is present;
colcon build exits successfully;
all requested tests complete;
colcon test-result --verbose reports zero errors and zero failures;
no package was silently omitted;
the build did not require removing a safety gate or changing a production default.

Retain the terminal output with the source revision, host identity, date, and role.

Failure handling

If a build fails:

do not launch the role from the partial install;
record the first build or test failure, not only the final colcon summary;
correct dependency or source issues in the repository;
rerun with --clean --test;
do not use --allow-missing to qualify an incomplete Core build;
do not copy build artifacts from another computer as a substitute for a target build.
Staged release builds

For an already deployed target, use the staged update workflow rather than overwriting the active install in place:

cd ~/Savo_Pi
bash deploy/common/update_role.sh --role core --pull

or:

cd ~/Savo_Pi
bash deploy/common/update_role.sh --role edge --pull

The staged updater:

verifies the Git checkout and available disk space;
refuses a dirty tree unless explicitly allowed for a build-only operation;
uses fast-forward-only Git updates;
verifies the full package set before stopping the active service;
builds and tests in savo_ws/.releases/;
activates the new install only after all tests pass;
retains the previous install as install.previous.release-timestamp;
restarts a previously active role service unless --no-restart is supplied.

See Production startup, Systemd services, and Recovery operations before using staged deployment on a robot.

Evidence record

For every target build, record:

Date and timezone:
Host and role:
Operating system:
ROS distribution:
Repository commit:
Build command:
Dependency-install command:
Packages selected:
Build result:
Test summary:
Operator:
Notes or deviations:
