# IARC :)

- `ros/` colcon packages
    - `qualisys/` qtm bridge
- `scripts/` workspace assembly and stuff
- `.github/` CI

## setup

If you want to run the Gazebo simulation you need to install `nix`!

## Stage 1 intelligent-flight PoC

From a Linux desktop / WSL2 shell with the PX4 fork built:

```sh
scripts/make_workspace.sh /tmp/mission10_ws
cd /tmp/mission10_ws
colcon build --symlink-install
. install/setup.sh
ros2 launch bringup stage1_intelligent.launch.py px4_dir:=/path/to/PX4-Autopilot
```

In a second terminal with the same workspace sourced:

```sh
ros2 run px4_offboard mission_gate
```

Press ENTER in the gate terminal. The expected PoC sequence is:
PX4 SITL + gz x500 mono cam, gz truth bridged as EV, AUTO.TAKEOFF to 6 m,
OFFBOARD hover, 10 orbit revolutions, then AUTO.RTL. The startup delays in the
launch file are temporary PoC shims; replace them with topic readiness gates
after the first end-to-end flight.
