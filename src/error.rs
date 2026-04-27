use std::fmt;

/// Errors returned from fallible plugin operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RapierError {
    /// `ColliderBuilder::convex_hull` rejected the point cloud (degenerate input,
    /// fewer than 4 non-coplanar points, etc).
    InvalidConvexHull,
    /// `ColliderBuilder::trimesh` rejected the mesh (empty, degenerate triangles,
    /// or index out of range).
    InvalidTriMesh,
}

impl fmt::Display for RapierError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RapierError::InvalidConvexHull => {
                f.write_str("convex hull construction failed (degenerate or coplanar points)")
            }
            RapierError::InvalidTriMesh => {
                f.write_str("trimesh construction failed (empty or malformed mesh)")
            }
        }
    }
}

impl std::error::Error for RapierError {}
