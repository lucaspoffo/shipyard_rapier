#[cfg(feature = "dim2")]
pub extern crate rapier2d as rapier;
#[cfg(feature = "dim3")]
pub extern crate rapier3d as rapier;

pub mod physics;

#[cfg(feature = "render")]
pub mod render;
