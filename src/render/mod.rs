use crate::physics::{ColliderHandleComponent, RapierConfiguration};
use macroquad::prelude::*;
use rapier::dynamics::RigidBodySet;
use rapier::geometry::{Collider, ColliderSet, ShapeType};
use rapier::pipeline::PhysicsPipeline;
use shipyard::{Get, IntoIter, IntoWithId, UniqueView, View};
use std::collections::HashMap;

/// The desired render color of a Rapier collider.
pub struct RapierRenderColor(pub f32, pub f32, pub f32);

const PALLETE: [Color; 3] = [
    Color::new(
        0x98 as f32 / 255.0,
        0xC1 as f32 / 255.0,
        0xD9 as f32 / 255.0,
        1.0,
    ),
    Color::new(
        0x05 as f32 / 255.0,
        0x3C as f32 / 255.0,
        0x5E as f32 / 255.0,
        1.0,
    ),
    Color::new(
        0x1F as f32 / 255.0,
        0x7A as f32 / 255.0,
        0x8C as f32 / 255.0,
        1.0,
    ),
];

#[allow(dead_code)]
const WIRE_COLOR: Color = Color::new(
    0x0e as f32 / 255.0,
    0x2c as f32 / 255.0,
    0x33 as f32 / 255.0,
    1.0,
);

const GROUND_COLOR: Color = Color::new(
    0xF3 as f32 / 255.0,
    0xD9 as f32 / 255.0,
    0xB1 as f32 / 255.0,
    1.0,
);

/// Render the physics time and the total frame time in the screen.
pub fn render_physics_stats(pipeline: UniqueView<PhysicsPipeline>) {
    let physics_time = format!("Physics time: {:.2}", pipeline.counters.step_time());
    let frame_time = format!("Frame time: {:.2}", get_frame_time() * 1000.);
    let fps = format!("FPS: {}", get_fps());
    draw_text(&physics_time, 10.0, 10.0, 30.0, BLACK);
    draw_text(&frame_time, 10.0, 30.0, 30.0, BLACK);
    draw_text(&fps, 10.0, 50.0, 30.0, BLACK);
}

/// System responsible for rendering the colliders with the macroquad rendering crate.
pub fn render_colliders(
    configuration: UniqueView<RapierConfiguration>,
    bodies: UniqueView<RigidBodySet>,
    colliders: UniqueView<ColliderSet>,
    colliders_handles: View<ColliderHandleComponent>,
    debug_colors: View<RapierRenderColor>,
) {
    let mut icolor = 0;
    let mut body_colors = HashMap::new();

    let gl = unsafe { get_internal_gl().quad_gl };

    for (entity, collider) in colliders_handles.iter().with_id() {
        if let Some(collider) = colliders.get(collider.handle()) {
            if let Some(body) = bodies.get(collider.parent()) {
                let default_color = if body.is_static() {
                    GROUND_COLOR
                } else {
                    *body_colors.entry(collider.parent()).or_insert_with(|| {
                        icolor += 1;
                        PALLETE[icolor % PALLETE.len()]
                    })
                };

                let debug_color = debug_colors.get(entity).ok();

                let color = debug_color
                    .map(|c| Color::new(c.0, c.1, c.2, 1.0))
                    .unwrap_or(default_color);

                render_colider(collider, color, configuration.scale, gl);
            }
        }
    }
}

#[cfg(feature = "dim2")]
fn render_colider(collider: &Collider, color: Color, scale: f32, gl: &mut QuadGl) {
    let pos = collider.position();
    let shape = collider.shape();

    let translation =
        glam::Vec3::new(pos.translation.vector.x, -pos.translation.vector.y, 0.0) * scale;
    match shape.shape_type() {
        ShapeType::Cuboid => {
            let c = shape.as_cuboid().unwrap();
            gl.push_model_matrix(glam::Mat4::from_translation(translation));
            gl.push_model_matrix(glam::Mat4::from_rotation_z(-pos.rotation.angle()));
            gl.push_model_matrix(glam::Mat4::from_scale(Vec3::one() * scale));

            draw_rectangle(
                -c.half_extents.x,
                -c.half_extents.y,
                c.half_extents.x * 2.0,
                c.half_extents.y * 2.0,
                color,
            );
            gl.pop_model_matrix();
            gl.pop_model_matrix();
            gl.pop_model_matrix();
        }
        ShapeType::Ball => {
            let b = shape.as_ball().unwrap();
            gl.push_model_matrix(glam::Mat4::from_translation(translation));
            gl.push_model_matrix(glam::Mat4::from_rotation_z(-pos.rotation.angle()));
            gl.push_model_matrix(glam::Mat4::from_scale(Vec3::one() * scale));

            draw_circle(0.0, 0.0, b.radius, color);

            gl.pop_model_matrix();
            gl.pop_model_matrix();
            gl.pop_model_matrix();
        }
        _ => {}
    };
}

#[cfg(feature = "dim3")]
fn render_colider(collider: &Collider, color: Color, scale: f32, gl: &mut QuadGl) {
    let pos = collider.position();
    let shape = collider.shape();
    let translation = glam::Vec3::new(
        pos.translation.vector.x,
        pos.translation.vector.y,
        pos.translation.vector.z,
    ) * scale;
    let quat = glam::Quat::from_xyzw(
        pos.rotation.i,
        pos.rotation.j,
        pos.rotation.k,
        pos.rotation.w,
    );

    gl.push_model_matrix(glam::Mat4::from_scale_rotation_translation(
        Vec3::one() * scale,
        quat,
        translation,
    ));

    match shape.shape_type() {
        ShapeType::Cuboid => {
            let c = shape.as_cuboid().unwrap();
            let size = Vec3::new(c.half_extents.x, c.half_extents.y, c.half_extents.z) * 2.0;

            draw_cube(Vec3::zero(), size, None, color);
            draw_cube_wires(Vec3::zero(), size, WIRE_COLOR);
        }
        ShapeType::Ball => {
            let b = shape.as_ball().unwrap();
            let radius = b.radius * scale;

            draw_sphere(Vec3::zero(), radius, None, color);
        }
        ShapeType::TriMesh => {
            let t = shape.as_trimesh().unwrap();
            let tris: Vec<([f32; 3], [f32; 2], [f32; 4])> = t
                .vertices()
                .iter()
                .enumerate()
                .map(|(i, v)| {
                    let uv: [f32; 2] = match i % 3 {
                        0 => [1., 0.],
                        1 => [0., 1.],
                        _ => [0., 0.],
                    };

                    ([v.x, v.y, v.z], uv, color.into())
                })
                .collect();

            gl.draw_mode(DrawMode::Lines);
            let indices = t
                .flat_indices()
                .iter()
                .map(|x| *x as u16)
                .collect::<Vec<u16>>();
            gl.geometry(&tris[..], &indices);
        }
        _ => {}
    }
    gl.pop_model_matrix();
}
