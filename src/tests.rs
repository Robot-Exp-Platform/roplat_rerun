//! Joint positions through the model reproduce Bullet's own link frames.

use nalgebra as na;
use rerun_urdf::roplat_description::RobotModel;
use rsbullet::{Mode, PhysicsClient, UrdfOptions};

use crate::rerun_robot::BulletJoints;

/// The base's centre of mass is offset and rotated from its link frame; the
/// tree branches at the base; a fixed tool hangs off the first branch.
const TREE: &str = r#"<?xml version="1.0"?>
<robot name="tree">
  <link name="base">
    <inertial><origin xyz="0.05 0.02 0.03" rpy="0.1 0 0.2"/><mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
  <link name="a"><inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="b"><inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="c"><inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="tool"><inertial><mass value="0.1"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="a"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="1" effort="1"/></joint>
  <joint name="j2" type="revolute"><parent link="a"/><child link="b"/>
    <origin xyz="0.2 0 0" rpy="0 0.3 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" velocity="1" effort="1"/></joint>
  <joint name="j3" type="prismatic"><parent link="base"/><child link="c"/>
    <origin xyz="0 0.1 0"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
  <joint name="mount" type="fixed"><parent link="b"/><child link="tool"/><origin xyz="0.1 0 0"/></joint>
</robot>
"#;

fn close(a: &na::Isometry3<f64>, b: &na::Isometry3<f64>) -> bool {
    (a.translation.vector - b.translation.vector).norm() < 1e-6
        && a.rotation.angle_to(&b.rotation) < 1e-6
}

#[test]
fn model_frames_match_bullet_link_frames() {
    let dir = std::env::temp_dir().join(format!("roplat_rerun_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("tree.urdf");
    std::fs::write(&path, TREE).unwrap();

    let model = RobotModel::from_urdf_str(TREE).unwrap();
    let mut client = PhysicsClient::connect(Mode::Direct).unwrap();
    let options = UrdfOptions {
        base: Some(na::Isometry3::new(
            na::Vector3::new(0.3, -0.2, 0.5),
            na::Vector3::new(0.0, 0.0, 0.7),
        )),
        use_fixed_base: true,
        ..Default::default()
    };
    let body = client.load_urdf(&path, Some(options)).unwrap();

    let joints = BulletJoints::query(&mut client, body, &model).unwrap();
    assert_eq!(joints.names, ["j1", "j2", "j3"]);
    for (&index, q) in joints.indices.iter().zip([0.4, -0.6, 0.25]) {
        client.reset_joint_state(body, index, q, Some(0.0)).unwrap();
    }

    let (base, positions) = joints.read(&mut client, body).unwrap();
    let frames = model.link_frames(|joint| {
        let i = joints.names.iter().position(|n| *n == joint.name).unwrap();
        positions[i]
    });
    for i in 0..client.get_num_joints(body) {
        let info = client.get_joint_info(body, i).unwrap();
        let link = model
            .links
            .iter()
            .position(|l| l.name == info.link_name)
            .unwrap();
        let bullet = client
            .get_link_state(body, i, true, false)
            .unwrap()
            .world_link_frame;
        assert!(
            close(&(base * frames[link]), &bullet),
            "link {}",
            info.link_name
        );
    }
    std::fs::remove_dir_all(dir).unwrap();
}
