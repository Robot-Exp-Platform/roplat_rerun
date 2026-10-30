//! The planning scene and sensed points in Rerun: what RViz's planning-scene
//! and point-cloud displays show.

use rerun as rr;
use roplat_scene::{BodyKind, PlanningScene};

/// Draws every link and object of `scene` at `q` under `root`, as the
/// collision checks see them: links grey under `root/links`, objects in the
/// world blue under `root/objects`, attached objects orange under
/// `root/attached`. Everything under `root` is cleared first, so objects
/// removed from the scene since the last call disappear.
pub fn log_scene<const N: usize>(
    rec: &rr::RecordingStream,
    root: &str,
    scene: &PlanningScene<N>,
    q: &[f64; N],
) -> rr::RecordingStreamResult<()> {
    rec.log(root, &rr::Clear::recursive())?;
    for mesh in scene.meshes(q) {
        let (group, color) = match mesh.kind {
            BodyKind::Link => (
                "links",
                rr::Rgba32::from_unmultiplied_rgba(170, 170, 170, 255),
            ),
            BodyKind::Object => (
                "objects",
                rr::Rgba32::from_unmultiplied_rgba(80, 140, 230, 200),
            ),
            BodyKind::Attached => (
                "attached",
                rr::Rgba32::from_unmultiplied_rgba(240, 150, 40, 255),
            ),
        };
        let positions = mesh.vertices.iter().map(|v| v.map(|c| c as f32));
        rec.log(
            format!("{root}/{group}/{}", mesh.name),
            &rr::Mesh3D::new(positions)
                .with_triangle_indices(mesh.triangles.iter().copied())
                .with_albedo_factor(color),
        )?;
    }
    Ok(())
}

/// Draws `points` (in the world) at `path` with `radius` (m), such as a
/// depth reading before it joins the scene or an object's segmented points.
pub fn log_points(
    rec: &rr::RecordingStream,
    path: &str,
    points: &[[f64; 3]],
    radius: f32,
    color: [u8; 3],
) -> rr::RecordingStreamResult<()> {
    let [r, g, b] = color;
    rec.log(
        path,
        &rr::Points3D::new(points.iter().map(|p| p.map(|c| c as f32)))
            .with_radii([radius])
            .with_colors([rr::Color::from_rgb(r, g, b)]),
    )
}

#[cfg(test)]
mod tests {
    use nalgebra::Isometry3;
    use rerun_urdf::roplat_description::{Geometry, RobotModel};

    use super::*;

    const ARM: &str = r#"<robot name="toy">
      <link name="base"><collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
      <joint name="turn" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.2"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
      <link name="arm"><collision><origin xyz="0.25 0 0"/><geometry><box size="0.5 0.05 0.05"/></geometry></collision></link>
    </robot>"#;

    #[test]
    fn a_scene_and_points_are_logged() {
        let model = RobotModel::from_urdf_str(ARM).unwrap();
        let mut scene: PlanningScene<1> =
            PlanningScene::new(model, ["turn"], |name: &str| std::path::PathBuf::from(name))
                .unwrap();
        scene
            .add_object(
                "ball",
                &Geometry::Sphere { radius: 0.05 },
                Isometry3::translation(0.6, 0.0, 0.2),
            )
            .unwrap();
        scene.attach("ball", "arm", &[], &[0.0]).unwrap();
        scene
            .add_object(
                "table",
                &Geometry::Box { size: [1.0, 1.0, 0.02] },
                Isometry3::translation(0.5, 0.0, 0.0),
            )
            .unwrap();
        let (rec, storage) = rr::RecordingStreamBuilder::new("roplat_rerun_scene")
            .memory()
            .unwrap();
        log_scene(&rec, "scene", &scene, &[0.3]).unwrap();
        log_points(
            &rec,
            "scene/seen",
            &[[0.5, 0.1, 0.05], [0.5, 0.12, 0.05]],
            0.005,
            [0, 200, 0],
        )
        .unwrap();
        let _ = rec.flush_blocking();
        // The clear, two links, one object, one attached object, the points,
        // and the recording's own setup messages.
        assert!(storage.take().len() >= 6);
    }
}
