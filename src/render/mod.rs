use crate::physics::{ColliderHandleComponent, RapierConfiguration};
use macroquad::prelude::*;
use rapier::dynamics::RigidBodySet;
use rapier::geometry::{ColliderSet, ShapeType};
use shipyard::{Get, IntoIter, IntoWithId, UniqueView, View};
use std::collections::HashMap;

/// The desired render color of a Rapier collider.
pub struct RapierRenderColor(pub f32, pub f32, pub f32);

/// System responsible for attaching a PbrBundle to each entity having a collider.
pub fn render_colliders(
    configuration: UniqueView<RapierConfiguration>,
    bodies: UniqueView<RigidBodySet>,
    colliders: UniqueView<ColliderSet>,
    colliders_handles: View<ColliderHandleComponent>,
    debug_colors: View<RapierRenderColor>,
) {
    let ground_color = Color::new(
        0xF3 as f32 / 255.0,
        0xD9 as f32 / 255.0,
        0xB1 as f32 / 255.0,
        1.0,
    );

    let palette = [
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

    let mut icolor = 0;
    let mut body_colors = HashMap::new();

    let gl = unsafe { get_internal_gl().quad_gl };

    for (entity, collider) in &mut colliders_handles.iter().with_id() {
        if let Some(collider) = colliders.get(collider.handle()) {
            if let Some(body) = bodies.get(collider.parent()) {
                let default_color = if body.is_static() {
                    ground_color
                } else {
                    *body_colors.entry(collider.parent()).or_insert_with(|| {
                        icolor += 1;
                        palette[icolor % palette.len()]
                    })
                };

                let shape = collider.shape();

                let debug_color = debug_colors.get(entity).ok();

                let color = debug_color
                    .map(|c| Color::new(c.0, c.1, c.2, 1.0))
                    .unwrap_or(default_color);

                let pos = collider.position();

                match shape.shape_type() {
                    // #[cfg(feature = "dim3")]
                    // ShapeType::Cuboid => Mesh::from(shape::Cube { size: 2.0 }),
                    #[cfg(feature = "dim2")]
                    ShapeType::Cuboid => {
                        let c = shape.as_cuboid().unwrap();
                        let translation = glam::Vec3::new(
                            pos.translation.vector.x,
                            -pos.translation.vector.y,
                            0.0,
                        );
                        gl.push_model_matrix(glam::Mat4::from_translation(translation));
                        gl.push_model_matrix(glam::Mat4::from_rotation_z(pos.rotation.angle()));

                        draw_rectangle(
                            -c.half_extents.x,
                            -c.half_extents.y,
                            c.half_extents.x * 2.0,
                            c.half_extents.y * 2.0,
                            color,
                        );
                        gl.pop_model_matrix();
                        gl.pop_model_matrix();
                        // Mesh::from(shape::Quad { size: Vec2::new(2.0, 2.0), flip: false })
                    }
                    /*
                     ShapeType::Ball => Mesh::from(shape::Icosphere {
                         subdivisions: 2,
                         radius: 1.0,
                     }),
                     */
                     /*
                     #[cfg(feature = "dim2")]
                     ShapeType::TriMesh => {
                         let mut mesh =
                             Mesh::new(bevy::render::pipeline::PrimitiveTopology::TriangleList);
                         let trimesh = shape.as_trimesh().unwrap();
                         mesh.set_attribute(
                             Mesh::ATTRIBUTE_POSITION,
                             VertexAttributeValues::from(
                                 trimesh
                                     .vertices()
                                     .iter()
                                     .map(|vertice| [vertice.x, vertice.y])
                                     .collect::<Vec<_>>(),
                             ),
                         );
                         mesh.set_indices(Some(Indices::U32(
                             trimesh
                                 .indices()
                                 .iter()
                                 .flat_map(|triangle| triangle.iter())
                                 .cloned()
                                 .collect(),
                         )));
                         mesh
                     }
                     */
                    /* #[cfg(feature = "dim3")]
                     ShapeType::TriMesh => {
                         let mut mesh =
                             Mesh::new(bevy::render::pipeline::PrimitiveTopology::TriangleList);
                         let trimesh = shape.as_trimesh().unwrap();
                         mesh.set_attribute(
                             Mesh::ATTRIBUTE_POSITION,
                             VertexAttributeValues::from(
                                 trimesh
                                     .vertices()
                                     .iter()
                                     .map(|vertex| [vertex.x, vertex.y, vertex.z])
                                     .collect::<Vec<_>>(),
                             ),
                         );
                         // Compute vertex normals by averaging the normals
                         // of every triangle they appear in.
                         // NOTE: This is a bit shonky, but good enough for visualisation.
                         let verts = trimesh.vertices();
                         let mut normals: Vec<Vec3> = vec![Vec3::zero(); trimesh.vertices().len()];
                         for triangle in trimesh.indices().iter() {
                             let ab = verts[triangle[1] as usize] - verts[triangle[0] as usize];
                             let ac = verts[triangle[2] as usize] - verts[triangle[0] as usize];
                             let normal = ab.cross(&ac);
                             // Contribute this normal to each vertex in the triangle.
                             for i in 0..3 {
                                 normals[triangle[i] as usize] +=
                                     Vec3::new(normal.x, normal.y, normal.z);
                             }
                         }
                         let normals: Vec<[f32; 3]> = normals
                             .iter()
                             .map(|normal| {
                                 let normal = normal.normalize();
                                 [normal.x, normal.y, normal.z]
                             })
                             .collect();
                         mesh.set_attribute(
                             Mesh::ATTRIBUTE_NORMAL,
                             VertexAttributeValues::from(normals),
                         );
                         // There's nothing particularly meaningful we can do
                         // for this one without knowing anything about the overall topology.
                         mesh.set_attribute(
                             Mesh::ATTRIBUTE_UV_0,
                             VertexAttributeValues::from(
                                 trimesh
                                     .vertices()
                                     .iter()
                                     .map(|_vertex| [0.0, 0.0])
                                     .collect::<Vec<_>>(),
                             ),
                         );
                         mesh.set_indices(Some(Indices::U32(
                             trimesh
                                 .indices()
                                 .iter()
                                 .flat_map(|triangle| triangle.iter())
                                 .cloned()
                                 .collect(),
                         )));
                         mesh
                     }*/
                    _ => continue,
                };

                /*let scale = match shape.shape_type() {
                    #[cfg(feature = "dim2")]
                    ShapeType::Cuboid => {
                        let c = shape.as_cuboid().unwrap();
                        Vec3::new(c.half_extents.x, c.half_extents.y, 1.0)
                    }
                    // #[cfg(feature = "dim3")]
                    // ShapeType::Cuboid => {
                    //     let c = shape.as_cuboid().unwrap();
                    //     Vec3::from_slice_unaligned(c.half_extents.as_slice())
                    // }
                    ShapeType::Ball => {
                        let b = shape.as_ball().unwrap();
                        Vec3::new(b.radius, b.radius, b.radius)
                    }
                    ShapeType::TriMesh => Vec3::one(),
                    _ => unimplemented!(),
                } * configuration.scale;

                let mut transform = Transform::from_scale(scale);
                crate::physics::sync_transform(
                    collider.position_wrt_parent(),
                    configuration.scale,
                    &mut transform,
                );

                let ground_pbr = PbrBundle {
                    mesh: meshes.add(mesh),
                    material: materials.add(color.into()),
                    transform,
                    ..Default::default()
                };

                commands.insert(entity, ground_pbr);
                */
            }
        }
    }
}
