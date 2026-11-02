use gl::types::*;
use std::mem::size_of;
use glam::{Vec3, Mat4, EulerRot};

pub struct Mesh {
    pub vertices: Vec<f32>,
    pub indices: Vec<u32>,
}

#[derive(Default, Clone, Copy)]
pub struct Transform {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub pit: f32,
    pub yaw: f32,
    pub rol: f32,
}

impl Transform {
    pub fn new(x: f32, y: f32, z: f32, pit: f32, yaw: f32, rol: f32) -> Self {
        Transform {
            x, y, z, pit, yaw, rol,
        }
    }

    pub fn to_matrix(&self) -> Mat4 {
        Mat4::from_translation(Vec3::new(self.x, self.y, self.z))
            * Mat4::from_euler(EulerRot::YXZ, self.yaw, self.pit, self.rol)
    }
}

pub struct Motive {
    pub speed: f32,
    pub move_time: f32,
    pub move_cool: f32,
    pub delta_yaw: f32,
    pub yaw_time: f32,
}

impl Motive {
    pub fn new() -> Self {
        Motive {
            speed: 0.0,
            move_time: 1.0,
            move_cool: 1.0,
            delta_yaw: 0.0,
            yaw_time: 0.0,
        }
    }
}

pub struct Model {
    vao: GLuint,
    vbo: GLuint,
    ebo: GLuint,
    index_count: GLsizei,
}

impl Model {
    pub fn new(mesh: Mesh) -> Self {
        let (mut vao, mut vbo, mut ebo) = (0, 0, 0);
        unsafe {
            gl::GenVertexArrays(1, &mut vao);
            gl::GenBuffers(1, &mut vbo);
            gl::GenBuffers(1, &mut ebo);

            gl::BindVertexArray(vao);

            gl::BindBuffer(gl::ARRAY_BUFFER, vbo);
            gl::BufferData(
                gl::ARRAY_BUFFER,
                (mesh.vertices.len() * size_of::<f32>()) as GLsizeiptr,
                mesh.vertices.as_ptr() as *const _,
                gl::STATIC_DRAW,
            );

            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, ebo);
            gl::BufferData(
                gl::ELEMENT_ARRAY_BUFFER,
                (mesh.indices.len() * size_of::<u32>()) as GLsizeiptr,
                mesh.indices.as_ptr() as *const _,
                gl::STATIC_DRAW,
            );

            gl::VertexAttribPointer(
                0, 3, gl::FLOAT, gl::FALSE,
                (3 * size_of::<f32>()) as GLsizei,
                std::ptr::null(),
            );
            gl::EnableVertexAttribArray(0);

            gl::BindVertexArray(0);
        }
        Self { vao, vbo, ebo, index_count: mesh.indices.len() as GLsizei }
    }

    pub fn draw(&self) {
        unsafe {
            gl::BindVertexArray(self.vao);
            gl::DrawElements(gl::TRIANGLES, self.index_count, gl::UNSIGNED_INT, std::ptr::null());
            gl::BindVertexArray(0);
        }
    }
}

impl Drop for Model {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteVertexArrays(1, &self.vao);
            gl::DeleteBuffers(1, &self.vbo);
            gl::DeleteBuffers(1, &self.ebo);
        }
    }
}
