use rapier3d::math::Pose;

/// `glam::Affine3A` -> rapier `Pose` (translation + rotation, scale discarded).
pub fn affine_to_pose(t: glam::Affine3A) -> Pose {
    let (_, rotation, translation) = t.to_scale_rotation_translation();
    Pose { rotation, translation }
}

/// `glam::Mat4` -> rapier `Pose` (translation + rotation, scale discarded).
/// Used when reading scene node transforms, which are `Mat4`.
pub fn mat4_to_pose(m: glam::Mat4) -> Pose {
    let (_, rotation, translation) = m.to_scale_rotation_translation();
    Pose { rotation, translation }
}

/// Rapier `Pose` -> `glam::Affine3A` (unit scale).
pub fn pose_to_affine(p: &Pose) -> glam::Affine3A {
    glam::Affine3A::from_rotation_translation(p.rotation, p.translation)
}
