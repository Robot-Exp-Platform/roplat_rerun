use std::{collections::HashMap, marker::PhantomData, path::PathBuf};

use anyhow::Result;
use nalgebra as na;
use rerun as rr;
use rerun_urdf::RerunUrdfLoader;
use robot_behavior::{AttachFrom, EntityBuilder, RobotFile};
use rsbullet::RsBulletRobot;

pub struct RerunRobot<R> {
    _marker: PhantomData<R>,
    loader: RerunUrdfLoader,
    pub(crate) rerun: rr::RecordingStream,
}

impl<R: Send> AttachFrom<RsBulletRobot<R>> for RerunRobot<R> {
    fn attach_from(self, from: &mut RsBulletRobot<R>) -> Result<()> {
        let body_id = from.body_id;
        let joint_indices = from.joint_indices.clone();
        let root_prefix = self.loader.root_prefix.clone();
        let mut frame = 0;
        from.enqueue(move |client, _| {
            let joint_states = client.get_joint_states(body_id, &joint_indices)?;

            for (i, state) in joint_states.iter().enumerate() {
                self.rerun
                    .log(
                        format!("{root_prefix}/joint/{i}"),
                        &rr::Scalars::new([state.position]),
                    )
                    .unwrap();
                self.rerun
                    .log(
                        format!("{root_prefix}/joint_vel/{i}"),
                        &rr::Scalars::new([state.velocity]),
                    )
                    .unwrap();
                self.rerun
                    .log(
                        format!("{root_prefix}/torque/{i}"),
                        &rr::Scalars::new([state.motor_torque]),
                    )
                    .unwrap();
            }

            let mut poses: HashMap<String, na::Isometry3<f64>> = HashMap::new();
            let ordered_links = self.loader.ordered_links_urdf().unwrap(); // ★ 确定性顺序
            for (i, link_name) in ordered_links.iter().enumerate() {
                if i == 0 {
                    let base_world = client.get_base_position_and_orientation(body_id)?;
                    poses.insert(link_name.clone(), base_world);
                    if let Ok(vel6) = client.get_base_velocity(body_id) {
                        self.rerun
                            .log(
                                format!("{root_prefix}/cartesian_vel"),
                                &rr::Scalars::new(vel6),
                            )
                            .unwrap();
                    }
                    continue;
                }

                let link_index = (i - 1) as i32;
                // 容错：不要用 joint_indices.len() 来提前 break——有的 URDF 有固定关节/虚拟关节等
                let state = client.get_link_state(body_id, link_index, true, true)?;
                println!(
                    "Link {} world pose: {:?}",
                    link_name, state.world_link_frame
                );
                poses.insert(link_name.clone(), state.world_link_frame);

                if let Some(vel) = state.world_velocity {
                    self.rerun
                        .log(
                            format!("{root_prefix}/cartesian_vel"),
                            &rr::Scalars::new(vel),
                        )
                        .unwrap();
                }
            }

            self.loader
                .log_frame(&self.rerun, frame, &poses, "realtime");
            frame += 1;
            Ok(false)
        })?;
        Ok(())
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

impl<'a, R: RobotFile> RerunRobotBuilder<'a, R> {
    pub fn new(rerun: &'a mut rr::RecordingStream) -> Self {
        Self {
            _marker: PhantomData,
            rerun,
            name: String::new(),
            load_file: R::URDF.into(),
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
