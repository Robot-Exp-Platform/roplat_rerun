use std::{marker::PhantomData, path::PathBuf};

use anyhow::Result;
use nalgebra as na;
use rerun as rr;
use rerun_urdf::{RerunUrdfLoader, roplat_description::RobotModel};
use roplat_behavior::RobotDescription;
use roplat_world::{AttachFrom, EntityBuilder};
use rsbullet::{BulletResult, PhysicsClient, RsBulletRobot};

pub struct RerunRobot<R> {
    _marker: PhantomData<R>,
    loader: RerunUrdfLoader,
    pub(crate) rerun: rr::RecordingStream,
}

impl<R: Send> AttachFrom<RsBulletRobot<R>> for RerunRobot<R> {
    fn attach_from(self, from: &mut RsBulletRobot<R>) -> Result<()> {
        let body_id = from.body_id;
        let joint_indices = from.joint_indices.clone();
        let end_effector_link = from.end_effector_link;
        let root_prefix = self.loader.root_prefix.clone();
        let mut frame = 0;
        let mut bullet_joints: Option<BulletJoints> = None;
        from.enqueue(move |client, _| {
            self.rerun.set_time_sequence("realtime", frame);
            let joint_states = client.get_joint_states(body_id, &joint_indices)?;
            for (i, state) in joint_states.iter().enumerate() {
                let _ = self.rerun.log(
                    format!("{root_prefix}/joint/{i}"),
                    &rr::Scalars::new([state.position]),
                );
                let _ = self.rerun.log(
                    format!("{root_prefix}/joint_vel/{i}"),
                    &rr::Scalars::new([state.velocity]),
                );
                let _ = self.rerun.log(
                    format!("{root_prefix}/torque/{i}"),
                    &rr::Scalars::new([state.motor_torque]),
                );
            }

            if bullet_joints.is_none() {
                bullet_joints = Some(BulletJoints::query(client, body_id, &self.loader.model)?);
            }
            let joints = bullet_joints.as_ref().unwrap();
            let (base, positions) = joints.read(client, body_id)?;
            self.loader.log_joint_positions(
                &self.rerun,
                frame,
                base,
                joints.names.iter().map(String::as_str).zip(positions),
                "realtime",
            );

            let velocity = if end_effector_link >= 0 {
                client
                    .get_link_state(body_id, end_effector_link, true, true)?
                    .world_velocity
            } else {
                Some(client.get_base_velocity(body_id)?)
            };
            if let Some(velocity) = velocity {
                let _ = self.rerun.log(
                    format!("{root_prefix}/cartesian_vel"),
                    &rr::Scalars::new(velocity),
                );
            }
            frame += 1;
            Ok(false)
        })?;
        Ok(())
    }
}
/// The Bullet joints that drive movable joints of the model, matched by name,
/// and the base's local inertial frame.
pub(crate) struct BulletJoints {
    pub(crate) indices: Vec<i32>,
    pub(crate) names: Vec<String>,
    base_inertial: na::Isometry3<f64>,
}

impl BulletJoints {
    pub(crate) fn query(
        client: &mut PhysicsClient,
        body_id: i32,
        model: &RobotModel,
    ) -> BulletResult<Self> {
        let (mut indices, mut names) = (Vec::new(), Vec::new());
        for i in 0..client.get_num_joints(body_id) {
            let info = client.get_joint_info(body_id, i)?;
            if model
                .joint(&info.joint_name)
                .is_some_and(|j| j.is_movable())
            {
                indices.push(i);
                names.push(info.joint_name);
            }
        }
        let base = client.get_dynamics_info(body_id, -1)?;
        let [x, y, z] = base.local_inertia_position;
        let [qx, qy, qz, qw] = base.local_inertia_orientation;
        let base_inertial = na::Isometry3::from_parts(
            na::Translation3::new(x, y, z),
            na::UnitQuaternion::from_quaternion(na::Quaternion::new(qw, qx, qy, qz)),
        );
        Ok(Self { indices, names, base_inertial })
    }

    /// The root link's world frame and the joint positions, in `names` order.
    /// Bullet reports the base's centre of mass, so the root link frame is
    /// that pose times the inverse local inertial frame.
    pub(crate) fn read(
        &self,
        client: &mut PhysicsClient,
        body_id: i32,
    ) -> BulletResult<(na::Isometry3<f64>, Vec<f64>)> {
        let base =
            client.get_base_position_and_orientation(body_id)? * self.base_inertial.inverse();
        let states = client.get_joint_states(body_id, &self.indices)?;
        Ok((base, states.iter().map(|s| s.position).collect()))
    }
}

pub struct RerunRobotBuilder<'a, R> {
    pub(crate) _marker: PhantomData<R>,
    pub(crate) rerun: &'a mut rr::RecordingStream,
    pub(crate) name: String,
    pub(crate) load_file: PathBuf,
    pub(crate) mesh_path: Option<PathBuf>,
    pub(crate) base: Option<nalgebra::Isometry3<f64>>,
    pub(crate) base_fixed: bool,
    pub(crate) scaling: Option<f64>,
}

impl<'a, R: RobotDescription> RerunRobotBuilder<'a, R> {
    pub fn new(rerun: &'a mut rr::RecordingStream) -> Self {
        Self {
            _marker: PhantomData,
            rerun,
            name: String::new(),
            load_file: R::URDF
                .expect("robot description must provide a URDF path")
                .into(),
            mesh_path: None,
            base: None,
            base_fixed: false,
            scaling: None,
        }
    }
    pub fn mesh_path(mut self, mesh_path: &'static str) -> Self {
        self.mesh_path = Some(mesh_path.into());
        self
    }
}

impl<'a, R> EntityBuilder<'a> for RerunRobotBuilder<'a, R> {
    type Entity = RerunRobot<R>;
    fn name(mut self, name: String) -> Self {
        self.name = name;
        self
    }

    fn base(mut self, base: impl Into<na::Isometry3<f64>>) -> Self {
        self.base = Some(base.into());
        self
    }
    fn base_fixed(mut self, base_fixed: bool) -> Self {
        self.base_fixed = base_fixed;
        self
    }
    fn scaling(mut self, scaling: f64) -> Self {
        self.scaling = Some(scaling);
        self
    }
    fn load(self) -> Result<RerunRobot<R>> {
        let loader = RerunUrdfLoader::from_urdf(
            self.load_file,
            self.mesh_path,
            format!("world/robots/{}", self.name),
        )?;
        loader.register_statics(&self.rerun.clone_weak(), self.base.unwrap_or_default())?;
        Ok(RerunRobot { _marker: PhantomData, loader, rerun: self.rerun.clone_weak() })
    }
}
