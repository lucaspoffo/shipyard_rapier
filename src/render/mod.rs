use crate::physics::{ColliderHandleComponent, RapierConfiguration};
use macroquad::prelude::*;
use rapier::dynamics::RigidBodySet;
use rapier::geometry::{ColliderSet, ShapeType};
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

pub fn render_physics_stats(pipeline: UniqueView<PhysicsPipeline>) {
    let text = format!("Physics time: {:.2}", pipeline.counters.step_time());
    draw_text(&text, 10.0, 10.0, 30.0, BLACK);
}

/// System responsible for attaching a PbrBundle to each entity having a collider.
pub fn render_colliders(
    configuration: UniqueView<RapierConfiguration>,
    bodies: UniqueView<RigidBodySet>,
    colliders: UniqueView<ColliderSet>,
    colliders_handles: View<ColliderHandleComponent>,
    debug_colors: View<RapierRenderColor>,
) {
    let mut icolor = 0;
    let mut body_colors = HashMap::new();
    let scale = Vec3::one() * configuration.scale;

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

                let shape = collider.shape();

                let debug_color = debug_colors.get(entity).ok();

                let color = debug_color
                    .map(|c| Color::new(c.0, c.1, c.2, 1.0))
                    .unwrap_or(default_color);

                let pos = collider.position();
                let translation =
                    glam::Vec3::new(pos.translation.vector.x, -pos.translation.vector.y, 0.0)
                        * configuration.scale;

                #[cfg(feature = "dim2")]
                match shape.shape_type() {
                    ShapeType::Cuboid => {
                        let c = shape.as_cuboid().unwrap();
                        gl.push_model_matrix(glam::Mat4::from_translation(translation));
                        gl.push_model_matrix(glam::Mat4::from_rotation_z(-pos.rotation.angle()));
                        gl.push_model_matrix(glam::Mat4::from_scale(scale));

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
                        gl.push_model_matrix(glam::Mat4::from_scale(scale));

                        draw_circle(0.0, 0.0, b.radius, color);

                        gl.pop_model_matrix();
                        gl.pop_model_matrix();
                        gl.pop_model_matrix();
                    }
                    _ => continue,
                };
                #[cfg(feature = "dim3")]
                match shape.shape_type() {
                    ShapeType::Cuboid => {
                        let c = shape.as_cuboid().unwrap();

                        let size =
                            Vec3::new(c.half_extents.x, c.half_extents.y, c.half_extents.z) * 2.0;

                        let translation = glam::Vec3::new(
                            pos.translation.vector.x,
                            pos.translation.vector.y,
                            pos.translation.vector.z,
                        ) * configuration.scale;

                        let quat = glam::Quat::from_xyzw(
                            pos.rotation.i,
                            pos.rotation.j,
                            pos.rotation.k,
                            pos.rotation.w,
                        );

                        gl.push_model_matrix(glam::Mat4::from_scale_rotation_translation(
                            scale,
                            quat,
                            translation,
                        ));

                        draw_cube(Vec3::zero(), size, None, color);
                        draw_cube_wires(Vec3::zero(), size, WIRE_COLOR);

                        gl.pop_model_matrix();
                    }
                    _ => continue,
                }
            }
        }
    }
}
