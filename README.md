# roplat_rerun

`roplat_rerun` connects an RsBullet robot to a Rerun visualization. It loads the robot's URDF visuals, then records simulated link poses, joint positions, joint velocities, and motor torques as the simulation advances.

Use it when you already have an `RsBulletRobot<R>` and want to inspect that robot through the shared `robot_behavior` builder interfaces. For a file recorder or a different pose source, use [rerun_urdf](https://github.com/Robot-Exp-Platform/rerun_urdf) directly; it accepts a recording stream and per-link poses without requiring a simulator.

## How the pieces fit

`RerunHost` owns the recording connection and model search paths. Its `robot_builder::<R>(name)` reads the URDF named by `R: RobotDescription` and creates a `RerunRobot<R>`. Calling `visualization.attach_from(&mut simulated_robot)` moves that visualizer into a callback on the robot's simulation queue.

```text
URDF + meshes ────────────────> Rerun geometry
                                      ↑
RsBullet::step() -> queued state reads -> link poses + scalar logs
                -> Bullet physics step
```

The simulator remains responsible for timing and state; the visualizer consumes it. Calling `attach_from` once is enough, but nothing updates until you call `sim.step()`. In the current implementation, callbacks read state before that call's physics integration. The adapter is a simulation observer, not a controller, a hardware driver, or its own roplat `Rhythm`.

## Install and prerequisites

The published crate is `roplat_rerun 0.2.0`, built around RsBullet `0.4.0`, `robot_behavior 0.6`, `rerun_urdf 0.1.1`, and Rerun `0.26`. Its current dependencies include RsBullet unconditionally. Even a visualization-only application therefore needs Rust nightly and the Bullet native build prerequisites:

- **Windows:** MSVC C++ build tools, Windows SDK, and CMake on `PATH`.
- **Linux:** C/C++ tools, CMake, OpenGL/GLU, and X11 development libraries. On Ubuntu: `build-essential cmake libgl1-mesa-dev libglu1-mesa-dev libx11-dev libxi-dev`.
- **macOS:** Xcode Command Line Tools and CMake; Cocoa and OpenGL are linked by the Bullet build.

There are no crate-level features to enable for this adapter. It does not require `rsbullet/roplat` for ordinary visualization. A source checkout still uses pinned Git dependencies; the registry installation below is the easier starting point.

`RerunHost::new` starts or connects to a Rerun Viewer through an executable on `PATH`. Install a compatible `0.26.x` Viewer before running this example, for example:

```sh
cargo install rerun-cli --version 0.26.2 --locked
```

## First visualization: one simulated joint

This example uses Bullet DIRECT mode and opens a **Rerun Viewer**, not a Bullet GUI. It writes no hardware commands. For compilation only, use `cargo +nightly check` instead of `run`.

```sh
cargo new simulation-viewer
cd simulation-viewer
```

Replace `Cargo.toml` with:

```toml
[package]
name = "simulation-viewer"
version = "0.1.0"
edition = "2024"

[dependencies]
roplat_rerun = "0.2.0"
rsbullet = "0.4.0"
robot_behavior = "0.6.1"
anyhow = "1"
```

Save this model as `demo.urdf` beside `Cargo.toml`. Both the simulation and the renderer load the same file; its visuals are primitive boxes, with no mesh downloads:

```xml
<?xml version="1.0"?>
<robot name="one_joint_demo">
  <link name="base">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual><geometry><box size="0.2 0.2 0.1"/></geometry></visual>
  </link>
  <link name="arm">
    <inertial>
      <origin xyz="0.2 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.007" iyz="0" izz="0.007"/>
    </inertial>
    <visual>
      <origin xyz="0.2 0 0"/>
      <geometry><box size="0.4 0.05 0.05"/></geometry>
    </visual>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <origin xyz="0 0 0.1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="5" velocity="2"/>
  </joint>
</robot>
```

Replace `src/main.rs` with:

```rust
use std::time::Duration;

use robot_behavior::{
    AddRobot, AddSearchPath, AttachFrom, EntityBuilder, PhysicsEngine, RobotDescription,
};
use roplat_rerun::RerunHost;
use rsbullet::{Mode, RsBullet};

struct DemoRobot;

impl RobotDescription for DemoRobot {
    const URDF: Option<&'static str> = Some("demo.urdf");
}

fn main() -> anyhow::Result<()> {
    let model_dir = std::env::current_dir()?;
    let mut sim = RsBullet::new(Mode::Direct)?;
    sim.add_search_path(&model_dir)?;
    sim.set_step_time(Duration::from_secs_f64(1.0 / 240.0))?;
    sim.set_gravity([0.0, 0.0, 0.0])?;
    let mut robot = sim.robot_builder::<DemoRobot>("demo").base_fixed(true).load()?;

    // This constructor starts (or connects to) a Rerun Viewer.
    let mut viewer = RerunHost::new("simulation_viewer")?;
    viewer.add_search_path(&model_dir)?;
    viewer.robot_builder::<DemoRobot>("demo").load()?.attach_from(&mut robot)?;

    for frame in 0..240 {
        // Prescribe a pose for a visualization demo, rather than drive a motor.
        let angle = (frame as f64 * 0.03).sin();
        sim.client.reset_joint_state(robot.body_id, 0, angle, Some(0.0))?;
        sim.step()?;
        std::thread::sleep(Duration::from_millis(10));
    }

    sim.shutdown();
    Ok(())
}
```

Run from the project directory:

```sh
# Linux/macOS
BULLET_SKIP_ASSET_EXPORT=1 cargo +nightly run
```

```powershell
# Windows PowerShell
$env:BULLET_SKIP_ASSET_EXPORT = "1"
cargo +nightly run
```

The viewer receives geometry under `world/robots/demo` and 240 pose samples as the arm swings. Select the `realtime` timeline to inspect the poses: despite its name, this adapter writes an integer frame counter, not elapsed time. The sleep makes the demonstration easier to watch; it does not synchronize wall-clock time with the physics timestep. Resetting the joint prescribes a pose for visualization and is not a motor-control example.

## Models, coordinates, and recorded data

`RobotDescription::URDF` must be `Some(path)`; the builder currently panics for `None`. `RerunHost::add_search_path` searches the added directories for that relative URDF path and also supplies the matching directory as the mesh root. Give the simulator access to the same model, use the same base pose, and keep mesh filenames resolvable relative to that root. ROS `package://` paths are not resolved automatically.

Link poses come from Bullet's world-frame state and are logged under `world/robots/<name>/<link>`. `rerun_urdf` labels `world` as right-handed with Z up. The example consistently uses meters and radians; the adapter copies numbers without converting units. Revolute joint positions/velocities and prismatic joint positions/velocities must be interpreted according to that joint's type and model scale.

| Entity suffix | Value source |
| --- | --- |
| `joint/<i>` | Position of the i-th selected revolute/prismatic joint |
| `joint_vel/<i>` | Velocity of that joint |
| `torque/<i>` | Bullet's reported applied motor torque value |
| `<link>` | Base pose or Bullet `world_link_frame` transform |
| `cartesian_vel` | Base/link velocity samples, currently sharing one entity path |

The numeric joint index is the position in `RsBulletRobot::joint_indices`, not a URDF joint name. Pose logging sets the `realtime` frame counter after the scalar logs are emitted, so scalar and pose samples are not guaranteed to share an exact frame timestamp. Treat this as an inspection adapter, not a synchronized measurement recorder.

## Current boundaries

The loader's initial registration is designed for a root-first serial chain. The attachment also maps URDF child links in declaration order to Bullet link indices. Verify that correspondence for your model, especially with fixed links, branching, or Bullet load flags that alter the model structure. Do not assume arbitrary URDFs map correctly by name.

`base_fixed` and `scaling` exist on the shared builder interface, but the Rerun builder currently does not apply those settings. Use a matching unscaled URDF for this version. The adapter also contains diagnostic printing and unchecked logging results, so avoid placing it on a deadline-critical control path.

For recording without opening a viewer, the public `RerunHost::new` API currently has no file-output option or recording-stream setter. Start with the complete `.rrd` example in [rerun_urdf](https://github.com/Robot-Exp-Platform/rerun_urdf) and supply the poses you need. Keep the host alive while the simulation runs; the attached visualizer uses a weak recording-stream handle.

## Next steps and license

- [Public API](https://docs.rs/roplat_rerun/0.2.0/roplat_rerun/).
- [Host and model lookup](src/rerun_renderer.rs), [attachment and logged values](src/rerun_robot.rs).
- [RsBullet](https://github.com/Robot-Exp-Platform/rsbullet) for simulation, state queries, and queued control.
- [rerun_urdf](https://github.com/Robot-Exp-Platform/rerun_urdf) for direct recordings from a custom pose source.

Distributed under the [Apache-2.0 license](LICENSE). Model and mesh assets retain their own licenses.
