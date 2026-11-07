use gl::types::*;
use std::f32::consts::PI;
use glam::{Vec3, Vec4, Mat4};
use rand::RngExt;

use crate::things::{Transform, Motive, Mesh, Model};
use crate::limbs::{Leg, LegAttach, Wing, WingAttach};

const BORDER_TOP: f32 = -60.0;
const BORDER_BOTTOM: f32 = 45.0;
const BORDER_LEFT: f32 = -45.0;
const BORDER_RIGHT: f32 = 45.0;

pub struct CellarSpider {
    body_model: Model,
    legseg_model: Model,
    pub transform: Transform,
    pub motive: Motive,
    legs: [Leg; 8],

    last_group_step_time: f32,
    anim_time: f32,
}

impl CellarSpider {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        let transform = Transform::new(x, y, z, 0.0, 0.0, 0.0);
        let motive = Motive::new();

        let body_matrix = transform.to_matrix();

        let attatchments: [LegAttach; 8] = Self::create_legattach();
        let leg_scale: [f32; 8] = Self::leg_scale();

        let legs: [Leg; 8] = std::array::from_fn(|i| {
            let att = &attatchments[i];
            let scale = leg_scale[i];
            let mut leg = Leg::default();

            let outward_dir = Vec3::new(-att.base_yaw.sin(), 0.0, -att.base_yaw.cos());

            leg.coxa_base_yaw = att.base_yaw;
            leg.knee_max_yaw = 60.0;

            leg.coxa.transform.x = att.offset.x;
            leg.coxa.transform.y = att.offset.y;
            leg.coxa.transform.z = att.offset.z;
            leg.coxa.transform.yaw = att.base_yaw;
            leg.coxa.transform.pit = 20.0_f32.to_radians();
            leg.coxa.length = 2.0 * scale;

            leg.femur.transform.x = 0.0;
            leg.femur.transform.y = -leg.coxa.length;
            leg.femur.transform.z = 0.0;
            leg.femur.transform.pit = 40.0_f32.to_radians();
            leg.femur.length = 2.0 * scale;

            leg.tibia.transform.y = -leg.femur.length;
            leg.tibia.transform.pit = (-70.0_f32).to_radians();
            leg.tibia.length = 3.0 * scale;

            leg.tarsus.transform.y = -leg.tibia.length;
            leg.tarsus.length = 0.5 * scale;

            let leg_reach = leg.coxa.length + leg.femur.length
                + leg.tibia.length + leg.tarsus.length;
            let horiz_dist = leg_reach * 0.6;
            let drop_dist = leg_reach * 0.4;

            leg.rest_position = att.offset
                + outward_dir * horiz_dist
                + Vec3::new(0.0, -drop_dist, 0.0);

            let world_rest = body_matrix.transform_point3(leg.rest_position);
            leg.current_target = world_rest;
            leg.step_time = 1.0;
            leg.step_offset = if i % 2 == 0 { 0.0 } else { 0.15 };
            leg.step_start = world_rest;
            leg.step_end = world_rest;

            leg.coxa_base_pitch = (-80.0_f32).to_radians();

            leg
        });

        CellarSpider {
            transform,
            motive,
            body_model: Model::new(Self::create_body_mesh()),
            legseg_model: Model::new(Self::create_legseg_mesh()),
            legs,
            last_group_step_time: 0.0,
            anim_time: 0.0,
        }
    }

    pub fn draw(&self, model_loc: GLint) {
        let body_mat: Mat4 = self.transform.to_matrix();
        unsafe { gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, body_mat.as_ref().as_ptr()) };
        self.body_model.draw();

        for leg in &self.legs {
            let coxa_mat = body_mat * leg.coxa.transform.to_matrix();
            let femur_mat = coxa_mat * leg.femur.transform.to_matrix();
            let tibia_mat = femur_mat * leg.tibia.transform.to_matrix();
            let tarsus_mat = tibia_mat * leg.tarsus.transform.to_matrix();

           struct SegDraw {
                mat: *const Mat4,
                length: f32,
                width: f32,
            }

            let segs: [SegDraw; 4] = [
                SegDraw {mat: &coxa_mat, length: leg.coxa.length, width: 1.6},
                SegDraw {mat: &femur_mat, length: leg.femur.length, width: 1.2},
                SegDraw {mat: &tibia_mat, length: leg.tibia.length, width: 0.8},
                SegDraw {mat: &tarsus_mat, length: leg.tarsus.length, width: 0.4},
            ];

            for seg in &segs {
                unsafe {
                    let final_mat = *seg.mat * Mat4::from_scale(Vec3::new(seg.width, seg.length, seg.width));
                    gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, final_mat.as_ref().as_ptr());
                }
                self.legseg_model.draw();
            }
        }
    }

    pub fn update(&mut self, delta_time: f32) {
        const STEP_TRIGGER_DIST: f32 = 2.0;
        const STEP_HEIGHT: f32 = 5.0;

        let stride = STEP_TRIGGER_DIST + 0.1 * self.motive.speed;
        let step_speed = (self.motive.speed / (2.0 * stride)).max(5.0);
        let group_stagger = 1.0 / self.motive.speed;

        self.anim_time += delta_time;

        let model = self.transform.to_matrix();
        let inv_body = model.inverse();

        for (i, leg) in self.legs.iter_mut().enumerate() {
            let group_a = i % 2 == 0;

            let ideal_world_position: Vec3 = (model
                * Vec4::from((leg.rest_position, 1.0))).truncate();

            if leg.step_time >= 1.0 {
                leg.step_cool -= delta_time;
                let drift = (ideal_world_position - leg.current_target).length();

                let stagger_ok = group_a 
                    || (self.anim_time - self.last_group_step_time > group_stagger);

                if drift > STEP_TRIGGER_DIST && leg.step_cool <= 0.0 && stagger_ok {
                    leg.step_start = leg.current_target;
                    leg.step_end = ideal_world_position;
                    leg.step_time = 0.0;
                    leg.step_cool = 0.25;

                    if group_a {
                        self.last_group_step_time = self.anim_time;
                    }
                }
            }
            else {
                leg.step_time = (leg.step_time + delta_time * step_speed).min(1.0);

                let flat = leg.step_start.lerp(leg.step_end, leg.step_time);
                let lift = (leg.step_time * PI).sin() * STEP_HEIGHT;

                leg.current_target = flat + Vec3::new(0.0, lift, 0.0);
            }

            let target = leg.current_target;
            leg.solve_leg_ik(target, inv_body, delta_time);
        }
    }

    pub fn behave(&mut self, delta_time: f32) {
        let mut rng = rand::rng();

        if self.motive.move_cool <= 0.0 {
            self.motive.move_time -= delta_time;
            if self.motive.move_time <= 0.0 {
                self.motive.move_cool = rng.random_range(1.0..5.0);
            }
        }
        else {
            self.motive.speed = 0.0;
            self.motive.move_cool -= delta_time;
            if self.motive.move_cool <= 0.01 {
                self.motive.move_time = rng.random_range(3.0..10.0);
                self.motive.speed = rng.random_range(3.0..5.0);
                self.motive.yaw_time = 0.5;
                self.motive.delta_yaw = rng.random_range(-5.0..5.0);
            }
        }

        let face_x = -self.transform.yaw.sin() * self.transform.pit.cos();
        let face_z = -self.transform.yaw.cos() * self.transform.pit.cos();

        let facing_out = 
            (self.transform.x > BORDER_RIGHT && face_x > 0.0) ||
            (self.transform.x < BORDER_LEFT && face_x < 0.0) ||
            (self.transform.z > BORDER_BOTTOM && face_z > 0.0) ||
            (self.transform.z < BORDER_TOP && face_z < 0.0);

        if facing_out {
            self.transform.yaw += delta_time;
        }
        else if self.motive.yaw_time > 0.0 {
            self.transform.yaw += self.motive.delta_yaw * delta_time;
            self.motive.yaw_time -= delta_time;
        }
        else {
            self.transform.x += face_x * self.motive.speed * delta_time;
            self.transform.z += face_z * self.motive.speed * delta_time;
        }
    } 

    fn create_legattach() -> [LegAttach; 8] {
        [
            LegAttach { offset: Vec3::new(0.05, -0.1, 0.03), base_yaw: (-140.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.05, -0.1, 0.0), base_yaw: (-110.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.05, -0.1, -0.02), base_yaw: (-80.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.05, -0.1, -0.075), base_yaw: (-45.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.05, -0.1, 0.03), base_yaw: (140.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.05, -0.1, 0.0), base_yaw: (110.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.05, -0.1, -0.02), base_yaw: (80.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.05, -0.1, -0.075), base_yaw: (45.0_f32).to_radians() },
        ]
    }

    fn leg_scale() -> [f32; 8] {
        [
            0.8, 1.0, 1.1, 1.1,
            0.8, 1.0, 1.1, 1.1,
        ]
    }

    fn create_body_mesh() -> Mesh {
        let vertices = vec![
            -0.049091, -0.053018, -0.300000,  // front square
            0.049091, -0.053018, -0.300000,
            -0.049091, -0.182618, -0.300000,
            0.049091, -0.182618, -0.300000,
            -0.049091, -0.053018,  0.092727,  // back square
            0.049091, -0.053018,  0.092727,
            -0.049091, -0.182618,  0.092727,
            0.049091, -0.182618,  0.092727,
            -0.098182, -0.053018, -0.201818,  // left square
            -0.098182, -0.053018, -0.005455,
            -0.098182, -0.182618, -0.201818,
            -0.098182, -0.182618, -0.005455,
            0.098182, -0.053018, -0.201818,  // right square
            0.098182, -0.053018, -0.005455,
            0.098182, -0.182618, -0.201818,
            0.098182, -0.182618, -0.005455,
            -0.049091,  0.011782, -0.201818,  // top square
            0.049091,  0.011782, -0.201818,
            -0.049091,  0.011782, -0.005455,
            0.049091,  0.011782, -0.005455,
            -0.049091, -0.247418, -0.201818,  // bottom square
            0.049091, -0.247418, -0.201818,
            -0.049091, -0.247418, -0.005455,
            0.049091, -0.247418, -0.005455,

            -0.065455,  0.065455, -0.054545,  // front square
            0.065455,  0.065455, -0.054545,
            -0.065455, -0.065455, -0.054545,
            0.065455, -0.065455, -0.054545,
            -0.032727,  0.032727,  0.600000,  // back square
            0.032727,  0.032727,  0.600000,
            -0.032727, -0.032727,  0.600000,
            0.032727, -0.032727,  0.600000,
            -0.130909,  0.065455,  0.109091,  // left square
            -0.104727,  0.052364,  0.436364,
            -0.130909, -0.065455,  0.109091,
            -0.130909, -0.065455,  0.436364,
            0.130909,  0.065455,  0.109091,  // right square
            0.104727,  0.052364,  0.436364,
            0.130909, -0.065455,  0.109091,
            0.130909, -0.065455,  0.436364,
            -0.065455,  0.130909,  0.109091,  // top square
            0.065455,  0.130909,  0.109091,
            -0.052364,  0.104727,  0.436364,
            0.052364,  0.104727,  0.436364,
            -0.065455, -0.130909,  0.109091,  // bottom square
            0.065455, -0.130909,  0.109091,
            -0.065455, -0.130909,  0.436364,
            0.065455, -0.130909,  0.436364,
        ];

        let indices = vec![
            0, 1, 2, // squares
            2, 3, 1,
            4, 5, 6,
            6, 7, 5,
            8, 9, 10,
            10, 11, 9,
            12, 13, 14,
            14, 15, 13,
            16, 17, 18,
            18, 19, 17,
            20, 21, 22,
            22, 23, 21,
            0, 2, 8, // connecting squares
            2, 8, 10,
            1, 3, 12,
            3, 12, 14,
            9, 11, 6,
            4, 6, 9,
            13, 15, 5,
            15, 5, 7,
            16, 17, 0,
            0, 1, 17,
            18, 19, 4,
            19, 4, 5,
            16, 18, 8,
            18, 8, 9,
            17, 19, 12,
            19, 12, 13,
            20, 21, 2,
            2, 3, 21,
            22, 23, 6,
            6, 7, 23,
            20, 22, 10,
            10, 11, 22,
            21, 23, 14,
            14, 15, 23,
            0, 8, 16, // triangles front
            1, 12, 17,
            2, 10, 20,
            3, 14, 21,
            4, 9, 18, // triangles back
            5, 13, 19,
            6, 11, 22,
            7, 15, 23,

            24, 25, 26, // squares
            26, 27, 25,
            28, 29, 30,
            30, 31, 29,
            32, 33, 34,
            34, 35, 33,
            36, 37, 38,
            38, 39, 37,
            40, 41, 42,
            42, 43, 41,
            44, 45, 46,
            46, 47, 45,
            24, 26, 32, // connecting squares
            26, 32, 34,
            25, 27, 36,
            27, 36, 38,
            33, 35, 30,
            28, 30, 33,
            37, 39, 29,
            39, 29, 31,
            40, 41, 24,
            24, 25, 41,
            42, 43, 28,
            43, 28, 29,
            40, 42, 32,
            42, 32, 33,
            41, 43, 36,
            43, 36, 37,
            44, 45, 26,
            26, 27, 45,
            46, 47, 30,
            30, 31, 47,
            44, 46, 34,
            34, 35, 46,
            45, 47, 38,
            38, 39, 47,
            24, 32, 40, // triangles front
            25, 36, 41,
            26, 34, 44,
            27, 38, 45,
            28, 33, 42, // triangles back
            29, 37, 43,
            30, 35, 46,
            31, 39, 47
        ];

        Mesh { vertices, indices }
    }

    fn create_legseg_mesh() -> Mesh {
        let vertices = vec![
            -0.0025, -0.25, -0.005, // front square
            0.0025, -0.25, -0.005,
            -0.0025, -0.75, -0.005,
            0.0025, -0.75, -0.005,
            -0.0025, -0.25, 0.005, // back square
            0.0025, -0.25, 0.005,
            -0.0025, -0.75, 0.005,
            0.0025, -0.75, 0.005,
            -0.005, -0.25, -0.0025, // left square
            -0.005, -0.25, 0.0025,
            -0.005, -0.75, -0.0025,
            -0.005, -0.75, 0.0025,
            0.005, -0.25, -0.0025, // right square
            0.005, -0.25, 0.0025,
            0.005, -0.75, -0.0025,
            0.005, -0.75, 0.0025,
            -0.0025, 0.0, -0.0025, // top square
            0.0025, 0.0, -0.0025,
            -0.0025, 0.0, 0.0025,
            0.0025, 0.0, 0.0025,
            -0.0025, -1.0, -0.0025, // bottom square
            0.0025, -1.0, -0.0025,
            -0.0025, -1.0, 0.0025,
            0.0025, -1.0, 0.0025
        ];

        let indices = vec![
            0, 1, 2, // squares
            2, 3, 1,
            4, 5, 6,
            6, 7, 5,
            8, 9, 10,
            10, 11, 9,
            12, 13, 14,
            14, 15, 13,
            16, 17, 18,
            18, 19, 17,
            20, 21, 22,
            22, 23, 21,
            0, 2, 8, // connecting squares
            2, 8, 10,
            1, 3, 12,
            3, 12, 14,
            9, 11, 6,
            4, 6, 9,
            13, 15, 5,
            15, 5, 7,
            16, 17, 0,
            0, 1, 17,
            18, 19, 4,
            19, 4, 5,
            16, 18, 8,
            18, 8, 9,
            17, 19, 12,
            19, 12, 13,
            20, 21, 2,
            2, 3, 21,
            22, 23, 6,
            6, 7, 23,
            20, 22, 10,
            10, 11, 22,
            21, 23, 14,
            14, 15, 23,
            0, 8, 16, // triangles front
            1, 12, 17,
            2, 10, 20,
            3, 14, 21,
            4, 9, 18, // triangles back
            5, 13, 19,
            6, 11, 22,
            7, 15, 23
        ];

        Mesh { vertices, indices }
    }
}

pub struct HouseSpider {
    body_model: Model,
    legseg_model: Model,
    pub transform: Transform,
    pub motive: Motive,
    legs: [Leg; 8],

    last_group_step_time: f32,
    anim_time: f32,
}

impl HouseSpider {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        let transform = Transform::new(x, y, z, 0.0, 0.0, 0.0);
        let motive = Motive::new();

        let body_matrix = transform.to_matrix();

        let attatchments: [LegAttach; 8] = Self::create_legattach();
        let leg_scale: [f32; 8] = Self::leg_scale();

        let legs: [Leg; 8] = std::array::from_fn(|i| {
            let att = &attatchments[i];
            let scale = leg_scale[i];
            let mut leg = Leg::default();

            let outward_dir = Vec3::new(-att.base_yaw.sin(), 0.0, -att.base_yaw.cos());

            leg.coxa_base_yaw = att.base_yaw;
            leg.knee_max_yaw = 60.0;

            leg.coxa.transform.x = att.offset.x;
            leg.coxa.transform.y = att.offset.y;
            leg.coxa.transform.z = att.offset.z;
            leg.coxa.transform.yaw = att.base_yaw;
            leg.coxa.transform.pit = 20.0_f32.to_radians();
            leg.coxa.length = 1.5 * scale;

            leg.femur.transform.x = 0.0;
            leg.femur.transform.y = -leg.coxa.length;
            leg.femur.transform.z = 0.0;
            leg.femur.transform.pit = 40.0_f32.to_radians();
            leg.femur.length = 1.5 * scale;

            leg.tibia.transform.y = -leg.femur.length;
            leg.tibia.transform.pit = (-70.0_f32).to_radians();
            leg.tibia.length = 2.5 * scale;

            leg.tarsus.transform.y = -leg.tibia.length;
            leg.tarsus.length = 0.5 * scale;

            let leg_reach = leg.coxa.length + leg.femur.length
                + leg.tibia.length + leg.tarsus.length;
            let horiz_dist = leg_reach * 1.0;
            let drop_dist = leg_reach * 0.4;

            leg.rest_position = att.offset
                + outward_dir * horiz_dist
                + Vec3::new(0.0, -drop_dist, 0.0);

            let world_rest = body_matrix.transform_point3(leg.rest_position);
            leg.current_target = world_rest;
            leg.step_time = 1.0;
            leg.step_offset = if i % 2 == 0 { 0.0 } else { 0.15 };
            leg.step_start = world_rest;
            leg.step_end = world_rest;

            leg.coxa_base_pitch = (-30.0_f32).to_radians();

            leg
        });

        HouseSpider {
            transform,
            motive,
            body_model: Model::new(Self::create_body_mesh()),
            legseg_model: Model::new(Self::create_legseg_mesh()),
            legs,
            last_group_step_time: 0.0,
            anim_time: 0.0,
        }
    }

    pub fn draw(&self, model_loc: GLint) {
        let body_mat: Mat4 = self.transform.to_matrix();
        unsafe { gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, body_mat.as_ref().as_ptr()) };
        self.body_model.draw();

        for leg in &self.legs {
            let coxa_mat = body_mat * leg.coxa.transform.to_matrix();
            let femur_mat = coxa_mat * leg.femur.transform.to_matrix();
            let tibia_mat = femur_mat * leg.tibia.transform.to_matrix();
            let tarsus_mat = tibia_mat * leg.tarsus.transform.to_matrix();

            struct SegDraw {
                mat: *const Mat4,
                length: f32,
                width: f32,
            }

            let segs: [SegDraw; 4] = [
                SegDraw {mat: &coxa_mat, length: leg.coxa.length, width: 1.6},
                SegDraw {mat: &femur_mat, length: leg.femur.length, width: 1.2},
                SegDraw {mat: &tibia_mat, length: leg.tibia.length, width: 0.8},
                SegDraw {mat: &tarsus_mat, length: leg.tarsus.length, width: 0.4},
            ];

            for seg in &segs {
                unsafe {
                    let final_mat = *seg.mat * Mat4::from_scale(Vec3::new(seg.width, seg.length, seg.width));
                    gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, final_mat.as_ref().as_ptr());
                }
                self.legseg_model.draw();
            }
        }
    }

    pub fn update(&mut self, delta_time: f32) {
        const STEP_TRIGGER_DIST: f32 = 2.0;
        const STEP_HEIGHT: f32 = 1.5;

        let stride = STEP_TRIGGER_DIST + 0.1 * self.motive.speed;
        let step_speed = (self.motive.speed / (2.0 * stride)).max(100.0);
        let group_stagger = 0.1 / (self.motive.speed * 1.5);

        self.anim_time += delta_time;

        let model = self.transform.to_matrix();
        let inv_body = model.inverse();

        for (i, leg) in self.legs.iter_mut().enumerate() {
            let group_a = i % 2 == 0;

            let ideal_world_position: Vec3 = (model
                * Vec4::from((leg.rest_position, 1.0))).truncate();

            if leg.step_time >= 1.0 {
                leg.step_cool -= delta_time;
                let drift = (ideal_world_position - leg.current_target).length();

                let stagger_ok = group_a 
                    || (self.anim_time - self.last_group_step_time > group_stagger);

                if drift > STEP_TRIGGER_DIST && leg.step_cool <= 0.0 && stagger_ok {
                    leg.step_start = leg.current_target;
                    leg.step_end = ideal_world_position;
                    leg.step_time = 0.0;
                    leg.step_cool = 0.25;

                    if group_a {
                        self.last_group_step_time = self.anim_time;
                    }
                }
            }
            else {
                leg.step_time = (leg.step_time + delta_time * step_speed).min(1.0);

                let flat = leg.step_start.lerp(leg.step_end, leg.step_time);
                let lift = (leg.step_time * PI).sin() * STEP_HEIGHT;

                leg.current_target = flat + Vec3::new(0.0, lift, 0.0);
            }

            let target = leg.current_target;
            leg.solve_leg_ik(target, inv_body, delta_time);
        }
    }

    pub fn behave(&mut self, delta_time: f32) {
        let mut rng = rand::rng();

        if self.motive.move_cool <= 0.0 {
            self.motive.move_time -= delta_time;
            if self.motive.move_time <= 0.0 {
                self.motive.move_cool = rng.random_range(2.0..10.0);
            }
        }
        else {
            self.motive.speed = 0.0;
            self.motive.move_cool -= delta_time;
            if self.motive.move_cool <= 0.01 {
                self.motive.move_time = rng.random_range(3.0..10.0);
                self.motive.speed = rng.random_range(-5.0..10.0); 
                self.motive.yaw_time = 0.1;
                self.motive.delta_yaw = rng.random_range(-5.0..5.0);
            }
        }

        let face_x = -self.transform.yaw.sin() * self.transform.pit.cos();
        let face_z = -self.transform.yaw.cos() * self.transform.pit.cos();

        let facing_out = 
            (self.transform.x > BORDER_RIGHT && face_x > 0.0) ||
            (self.transform.x < BORDER_LEFT && face_x < 0.0) ||
            (self.transform.z > BORDER_BOTTOM && face_z > 0.0) ||
            (self.transform.z < BORDER_TOP && face_z < 0.0);

        if facing_out {
            self.transform.yaw += delta_time;
        }
        else if self.motive.yaw_time > 0.0 {
            self.transform.yaw += self.motive.delta_yaw * delta_time;
            self.motive.yaw_time -= delta_time;
        }
        else {
            self.transform.x += face_x * self.motive.speed * delta_time;
            self.transform.z += face_z * self.motive.speed * delta_time;
        }
    } 

    fn create_legattach() -> [LegAttach; 8] {
        [
            LegAttach { offset: Vec3::new(0.05, -0.05, 0.03), base_yaw: (-150.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.05, -0.05, 0.0), base_yaw: (-120.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.05, -0.05, -0.02), base_yaw: (-60.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.05, -0.05, -0.075), base_yaw: (-30.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.05, -0.05, 0.03), base_yaw: (150.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.05, -0.05, 0.0), base_yaw: (120.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.05, -0.05, -0.02), base_yaw: (60.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.05, -0.05, -0.075), base_yaw: (30.0_f32).to_radians() },
        ]
    }

    fn leg_scale() -> [f32; 8] {
        [
            0.8, 1.0, 1.0, 1.2,
            0.8, 1.0, 1.0, 1.2,
        ]
    }

    fn create_body_mesh() -> Mesh {
        let vertices = vec![
            -0.109091, -0.117818, -0.750000,  // front square
            0.109091, -0.117818, -0.750000,
            -0.109091, -0.405818, -0.750000,
            0.109091, -0.405818, -0.750000,
            -0.109091, -0.117818,  0.122727,  // back square
            0.109091, -0.117818,  0.122727,
            -0.109091, -0.405818,  0.122727,
            0.109091, -0.405818,  0.122727,
            -0.218182, -0.117818, -0.531818,  // left square
            -0.218182, -0.117818, -0.095455,
            -0.218182, -0.405818, -0.531818,
            -0.218182, -0.405818, -0.095455,
            0.218182, -0.117818, -0.531818,  // right square
            0.218182, -0.117818, -0.095455,
            0.218182, -0.405818, -0.531818,
            0.218182, -0.405818, -0.095455,
            -0.109091,  0.026182, -0.531818,  // top square
            0.109091,  0.026182, -0.531818,
            -0.109091,  0.026182, -0.095455,
            0.109091,  0.026182, -0.095455,
            -0.109091, -0.549818, -0.531818,  // bottom square
            0.109091, -0.549818, -0.531818,
            -0.109091, -0.549818, -0.095455,
            0.109091, -0.549818, -0.095455,

            -0.145455,  0.145455, -0.204545,  // front square
            0.145455,  0.145455, -0.204545,
            -0.145455, -0.145455, -0.204545,
            0.145455, -0.145455, -0.204545,
            -0.072727,  0.072727,  1.250000,  // back square
            0.072727,  0.072727,  1.250000,
            -0.072727, -0.072727,  1.250000,
            0.072727, -0.072727,  1.250000,
            -0.290909,  0.145455,  0.159091,  // left square
            -0.232727,  0.116364,  0.886364,
            -0.290909, -0.145455,  0.159091,
            -0.290909, -0.145455,  0.886364,
            0.290909,  0.145455,  0.159091,  // right square
            0.232727,  0.116364,  0.886364,
            0.290909, -0.145455,  0.159091,
            0.290909, -0.145455,  0.886364,
            -0.145455,  0.290909,  0.159091,  // top square
            0.145455,  0.290909,  0.159091,
            -0.116364,  0.232727,  0.886364,
            0.116364,  0.232727,  0.886364,
            -0.145455, -0.290909,  0.159091,  // bottom square
            0.145455, -0.290909,  0.159091,
            -0.145455, -0.290909,  0.886364,
            0.145455, -0.290909,  0.886364,
        ];

        let indices = vec![
            0, 1, 2, // squares
            2, 3, 1,
            4, 5, 6,
            6, 7, 5,
            8, 9, 10,
            10, 11, 9,
            12, 13, 14,
            14, 15, 13,
            16, 17, 18,
            18, 19, 17,
            20, 21, 22,
            22, 23, 21,
            0, 2, 8, // connecting squares
            2, 8, 10,
            1, 3, 12,
            3, 12, 14,
            9, 11, 6,
            4, 6, 9,
            13, 15, 5,
            15, 5, 7,
            16, 17, 0,
            0, 1, 17,
            18, 19, 4,
            19, 4, 5,
            16, 18, 8,
            18, 8, 9,
            17, 19, 12,
            19, 12, 13,
            20, 21, 2,
            2, 3, 21,
            22, 23, 6,
            6, 7, 23,
            20, 22, 10,
            10, 11, 22,
            21, 23, 14,
            14, 15, 23,
            0, 8, 16, // triangles front
            1, 12, 17,
            2, 10, 20,
            3, 14, 21,
            4, 9, 18, // triangles back
            5, 13, 19,
            6, 11, 22,
            7, 15, 23,

            24, 25, 26, // squares
            26, 27, 25,
            28, 29, 30,
            30, 31, 29,
            32, 33, 34,
            34, 35, 33,
            36, 37, 38,
            38, 39, 37,
            40, 41, 42,
            42, 43, 41,
            44, 45, 46,
            46, 47, 45,
            24, 26, 32, // connecting squares
            26, 32, 34,
            25, 27, 36,
            27, 36, 38,
            33, 35, 30,
            28, 30, 33,
            37, 39, 29,
            39, 29, 31,
            40, 41, 24,
            24, 25, 41,
            42, 43, 28,
            43, 28, 29,
            40, 42, 32,
            42, 32, 33,
            41, 43, 36,
            43, 36, 37,
            44, 45, 26,
            26, 27, 45,
            46, 47, 30,
            30, 31, 47,
            44, 46, 34,
            34, 35, 46,
            45, 47, 38,
            38, 39, 47,
            24, 32, 40, // triangles front
            25, 36, 41,
            26, 34, 44,
            27, 38, 45,
            28, 33, 42, // triangles back
            29, 37, 43,
            30, 35, 46,
            31, 39, 47
        ];

        Mesh { vertices, indices }
    }

    fn create_legseg_mesh() -> Mesh {
        let vertices = vec![
            -0.025, -0.25, -0.1, // front square
            0.025, -0.25, -0.1,
            -0.025, -0.75, -0.1,
            0.025, -0.75, -0.1,
            -0.025, -0.25, 0.1, // back square
            0.025, -0.25, 0.1,
            -0.025, -0.75, 0.1,
            0.025, -0.75, 0.1,
            -0.05, -0.25, -0.05, // left square
            -0.05, -0.25, 0.05,
            -0.05, -0.75, -0.05,
            -0.05, -0.75, 0.05,
            0.05, -0.25, -0.05, // right square
            0.05, -0.25, 0.05,
            0.05, -0.75, -0.05,
            0.05, -0.75, 0.05,
            -0.025, 0.0, -0.05, // top square
            0.025, 0.0, -0.05,
            -0.025, 0.0, 0.05,
            0.025, 0.0, 0.05,
            -0.025, -1.0, -0.05, // bottom square
            0.025, -1.0, -0.05,
            -0.025, -1.0, 0.05,
            0.025, -1.0, 0.05
        ];

        let indices = vec![
            0, 1, 2, // squares
            2, 3, 1,
            4, 5, 6,
            6, 7, 5,
            8, 9, 10,
            10, 11, 9,
            12, 13, 14,
            14, 15, 13,
            16, 17, 18,
            18, 19, 17,
            20, 21, 22,
            22, 23, 21,
            0, 2, 8, // connecting squares
            2, 8, 10,
            1, 3, 12,
            3, 12, 14,
            9, 11, 6,
            4, 6, 9,
            13, 15, 5,
            15, 5, 7,
            16, 17, 0,
            0, 1, 17,
            18, 19, 4,
            19, 4, 5,
            16, 18, 8,
            18, 8, 9,
            17, 19, 12,
            19, 12, 13,
            20, 21, 2,
            2, 3, 21,
            22, 23, 6,
            6, 7, 23,
            20, 22, 10,
            10, 11, 22,
            21, 23, 14,
            14, 15, 23,
            0, 8, 16, // triangles front
            1, 12, 17,
            2, 10, 20,
            3, 14, 21,
            4, 9, 18, // triangles back
            5, 13, 19,
            6, 11, 22,
            7, 15, 23
        ];

        Mesh { vertices, indices }
    }
}

pub struct FalseWidow {
    body_model: Model,
    legseg_model: Model,
    pub transform: Transform,
    pub motive: Motive,
    legs: [Leg; 8],

    last_group_step_time: f32,
    anim_time: f32,
}

impl FalseWidow {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        let transform = Transform::new(x, y, z, 0.0, 0.0, 0.0);
        let motive = Motive::new();

        let body_matrix = transform.to_matrix();

        let attatchments: [LegAttach; 8] = Self::create_legattach();
        let leg_scale: [f32; 8] = Self::leg_scale();

        let legs: [Leg; 8] = std::array::from_fn(|i| {
            let att = &attatchments[i];
            let scale = leg_scale[i];
            let mut leg = Leg::default();

            let outward_dir = Vec3::new(-att.base_yaw.sin(), 0.0, -att.base_yaw.cos());

            leg.coxa_base_yaw = att.base_yaw;
            leg.knee_max_yaw = 60.0;

            leg.coxa.transform.x = att.offset.x;
            leg.coxa.transform.y = att.offset.y;
            leg.coxa.transform.z = att.offset.z;
            leg.coxa.transform.yaw = att.base_yaw;
            leg.coxa.transform.pit = 20.0_f32.to_radians();
            leg.coxa.length = 1.0 * scale;

            leg.femur.transform.x = 0.0;
            leg.femur.transform.y = -leg.coxa.length;
            leg.femur.transform.z = 0.0;
            leg.femur.transform.pit = 40.0_f32.to_radians();
            leg.femur.length = 1.0 * scale;

            leg.tibia.transform.y = -leg.femur.length;
            leg.tibia.transform.pit = (-70.0_f32).to_radians();
            leg.tibia.length = 1.75 * scale;

            leg.tarsus.transform.y = -leg.tibia.length;
            leg.tarsus.length = 0.25 * scale;

            let leg_reach = leg.coxa.length + leg.femur.length
                + leg.tibia.length + leg.tarsus.length;
            let horiz_dist = leg_reach * 1.0;
            let drop_dist = leg_reach * 0.4;

            leg.rest_position = att.offset
                + outward_dir * horiz_dist
                + Vec3::new(0.0, -drop_dist, 0.0);

            let world_rest = body_matrix.transform_point3(leg.rest_position);
            leg.current_target = world_rest;
            leg.step_time = 1.0;
            leg.step_offset = if i % 2 == 0 { 0.0 } else { 0.15 };
            leg.step_start = world_rest;
            leg.step_end = world_rest;

            leg.coxa_base_pitch = (-30.0_f32).to_radians();

            leg
        });

        FalseWidow {
            transform,
            motive,
            body_model: Model::new(Self::create_body_mesh()),
            legseg_model: Model::new(Self::create_legseg_mesh()),
            legs,
            last_group_step_time: 0.0,
            anim_time: 0.0,
        }
    }

    pub fn draw(&self, model_loc: GLint) {
        let body_mat: Mat4 = self.transform.to_matrix();
        unsafe { gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, body_mat.as_ref().as_ptr()) };
        self.body_model.draw();

        for leg in &self.legs {
            let coxa_mat = body_mat * leg.coxa.transform.to_matrix();
            let femur_mat = coxa_mat * leg.femur.transform.to_matrix();
            let tibia_mat = femur_mat * leg.tibia.transform.to_matrix();
            let tarsus_mat = tibia_mat * leg.tarsus.transform.to_matrix();

            struct SegDraw {
                mat: *const Mat4,
                length: f32,
                width: f32,
            }

            let segs: [SegDraw; 4] = [
                SegDraw {mat: &coxa_mat, length: leg.coxa.length, width: 1.6},
                SegDraw {mat: &femur_mat, length: leg.femur.length, width: 1.2},
                SegDraw {mat: &tibia_mat, length: leg.tibia.length, width: 0.8},
                SegDraw {mat: &tarsus_mat, length: leg.tarsus.length, width: 0.4},
            ];

            for seg in &segs {
                unsafe {
                    let final_mat = *seg.mat * Mat4::from_scale(Vec3::new(seg.width, seg.length, seg.width));
                    gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, final_mat.as_ref().as_ptr());
                }
                self.legseg_model.draw();
            }
        }
    }

    pub fn update(&mut self, delta_time: f32) {
        const STEP_TRIGGER_DIST: f32 = 2.0;
        const STEP_HEIGHT: f32 = 1.5;

        let stride = STEP_TRIGGER_DIST + 0.1 * self.motive.speed;
        let step_speed = (self.motive.speed / (2.0 * stride)).max(100.0);
        let group_stagger = 0.1 / (self.motive.speed * 1.5);

        self.anim_time += delta_time;

        let model = self.transform.to_matrix();
        let inv_body = model.inverse();

        for (i, leg) in self.legs.iter_mut().enumerate() {
            let group_a = i % 2 == 0;

            let ideal_world_position: Vec3 = (model
                * Vec4::from((leg.rest_position, 1.0))).truncate();

            if leg.step_time >= 1.0 {
                leg.step_cool -= delta_time;
                let drift = (ideal_world_position - leg.current_target).length();

                let stagger_ok = group_a 
                    || (self.anim_time - self.last_group_step_time > group_stagger);

                if drift > STEP_TRIGGER_DIST && leg.step_cool <= 0.0 && stagger_ok {
                    leg.step_start = leg.current_target;
                    leg.step_end = ideal_world_position;
                    leg.step_time = 0.0;
                    leg.step_cool = 0.25;

                    if group_a {
                        self.last_group_step_time = self.anim_time;
                    }
                }
            }
            else {
                leg.step_time = (leg.step_time + delta_time * step_speed).min(1.0);

                let flat = leg.step_start.lerp(leg.step_end, leg.step_time);
                let lift = (leg.step_time * PI).sin() * STEP_HEIGHT;

                leg.current_target = flat + Vec3::new(0.0, lift, 0.0);
            }

            let target = leg.current_target;
            leg.solve_leg_ik(target, inv_body, delta_time);
        }
    }

    pub fn behave(&mut self, delta_time: f32) {
        let mut rng = rand::rng();

        if self.motive.move_cool <= 0.0 {
            self.motive.move_time -= delta_time;
            if self.motive.move_time <= 0.0 {
                self.motive.move_cool = rng.random_range(2.0..10.0);
            }
        }
        else {
            self.motive.speed = 0.0;
            self.motive.move_cool -= delta_time;
            if self.motive.move_cool <= 0.01 {
                self.motive.move_time = rng.random_range(3.0..10.0);
                self.motive.speed = rng.random_range(-5.0..10.0); 
                self.motive.yaw_time = 0.1;
                self.motive.delta_yaw = rng.random_range(-5.0..5.0);
            }
        }

        let face_x = -self.transform.yaw.sin() * self.transform.pit.cos();
        let face_z = -self.transform.yaw.cos() * self.transform.pit.cos();

        let facing_out = 
            (self.transform.x > BORDER_RIGHT && face_x > 0.0) ||
            (self.transform.x < BORDER_LEFT && face_x < 0.0) ||
            (self.transform.z > BORDER_BOTTOM && face_z > 0.0) ||
            (self.transform.z < BORDER_TOP && face_z < 0.0);

        if facing_out {
            self.transform.yaw += delta_time;
        }
        else if self.motive.yaw_time > 0.0 {
            self.transform.yaw += self.motive.delta_yaw * delta_time;
            self.motive.yaw_time -= delta_time;
        }
        else {
            self.transform.x += face_x * self.motive.speed * delta_time;
            self.transform.z += face_z * self.motive.speed * delta_time;
        }
    } 

    fn create_legattach() -> [LegAttach; 8] {
        [
            LegAttach { offset: Vec3::new(0.05, -0.05, 0.03), base_yaw: (-150.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.05, -0.05, 0.0), base_yaw: (-120.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.05, -0.05, -0.02), base_yaw: (-60.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.05, -0.05, -0.075), base_yaw: (-30.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.05, -0.05, 0.03), base_yaw: (150.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.05, -0.05, 0.0), base_yaw: (120.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.05, -0.05, -0.02), base_yaw: (60.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.05, -0.05, -0.075), base_yaw: (30.0_f32).to_radians() },
        ]
    }

    fn leg_scale() -> [f32; 8] {
        [
            0.8, 1.0, 1.0, 1.2,
            0.8, 1.0, 1.0, 1.2,
        ]
    }

    fn create_body_mesh() -> Mesh {
        let vertices = vec![
            -0.109091, -0.117818, -0.750000,  // front square
            0.109091, -0.117818, -0.750000,
            -0.109091, -0.405818, -0.750000,
            0.109091, -0.405818, -0.750000,
            -0.109091, -0.117818,  0.122727,  // back square
            0.109091, -0.117818,  0.122727,
            -0.109091, -0.405818,  0.122727,
            0.109091, -0.405818,  0.122727,
            -0.218182, -0.117818, -0.531818,  // left square
            -0.218182, -0.117818, -0.095455,
            -0.218182, -0.405818, -0.531818,
            -0.218182, -0.405818, -0.095455,
            0.218182, -0.117818, -0.531818,  // right square
            0.218182, -0.117818, -0.095455,
            0.218182, -0.405818, -0.531818,
            0.218182, -0.405818, -0.095455,
            -0.109091,  0.026182, -0.531818,  // top square
            0.109091,  0.026182, -0.531818,
            -0.109091,  0.026182, -0.095455,
            0.109091,  0.026182, -0.095455,
            -0.109091, -0.549818, -0.531818,  // bottom square
            0.109091, -0.549818, -0.531818,
            -0.109091, -0.549818, -0.095455,
            0.109091, -0.549818, -0.095455,

            -0.145455,  0.290910, -0.204545,  // front square
            0.145455,  0.290910, -0.204545,
            -0.145455, -0.290910, -0.204545,
            0.145455, -0.290910, -0.204545,
            -0.072727,  0.145454,  1.250000,  // back square
            0.072727,  0.145454,  1.250000,
            -0.072727, -0.145454,  1.250000,
            0.072727, -0.145454,  1.250000,
            -0.290909,  0.290910,  0.159091,  // left square
            -0.232727,  0.232728,  0.886364,
            -0.290909, -0.290910,  0.159091,
            -0.290909, -0.290910,  0.886364,
            0.290909,  0.290910,  0.159091,  // right square
            0.232727,  0.232728,  0.886364,
            0.290909, -0.290910,  0.159091,
            0.290909, -0.290910,  0.886364,
            -0.145455,  0.581818,  0.159091,  // top square
            0.145455,  0.581818,  0.159091,
            -0.116364,  0.465454,  0.886364,
            0.116364,  0.465454,  0.886364,
            -0.145455, -0.581818,  0.159091,  // bottom square
            0.145455, -0.581818,  0.159091,
            -0.145455, -0.581818,  0.886364,
            0.145455, -0.581818,  0.886364,
        ];

        let indices = vec![
            0, 1, 2, // squares
            2, 3, 1,
            4, 5, 6,
            6, 7, 5,
            8, 9, 10,
            10, 11, 9,
            12, 13, 14,
            14, 15, 13,
            16, 17, 18,
            18, 19, 17,
            20, 21, 22,
            22, 23, 21,
            0, 2, 8, // connecting squares
            2, 8, 10,
            1, 3, 12,
            3, 12, 14,
            9, 11, 6,
            4, 6, 9,
            13, 15, 5,
            15, 5, 7,
            16, 17, 0,
            0, 1, 17,
            18, 19, 4,
            19, 4, 5,
            16, 18, 8,
            18, 8, 9,
            17, 19, 12,
            19, 12, 13,
            20, 21, 2,
            2, 3, 21,
            22, 23, 6,
            6, 7, 23,
            20, 22, 10,
            10, 11, 22,
            21, 23, 14,
            14, 15, 23,
            0, 8, 16, // triangles front
            1, 12, 17,
            2, 10, 20,
            3, 14, 21,
            4, 9, 18, // triangles back
            5, 13, 19,
            6, 11, 22,
            7, 15, 23,

            24, 25, 26, // squares
            26, 27, 25,
            28, 29, 30,
            30, 31, 29,
            32, 33, 34,
            34, 35, 33,
            36, 37, 38,
            38, 39, 37,
            40, 41, 42,
            42, 43, 41,
            44, 45, 46,
            46, 47, 45,
            24, 26, 32, // connecting squares
            26, 32, 34,
            25, 27, 36,
            27, 36, 38,
            33, 35, 30,
            28, 30, 33,
            37, 39, 29,
            39, 29, 31,
            40, 41, 24,
            24, 25, 41,
            42, 43, 28,
            43, 28, 29,
            40, 42, 32,
            42, 32, 33,
            41, 43, 36,
            43, 36, 37,
            44, 45, 26,
            26, 27, 45,
            46, 47, 30,
            30, 31, 47,
            44, 46, 34,
            34, 35, 46,
            45, 47, 38,
            38, 39, 47,
            24, 32, 40, // triangles front
            25, 36, 41,
            26, 34, 44,
            27, 38, 45,
            28, 33, 42, // triangles back
            29, 37, 43,
            30, 35, 46,
            31, 39, 47
        ];

        Mesh { vertices, indices }
    }

    fn create_legseg_mesh() -> Mesh {
        let vertices = vec![
            -0.05, -0.25, -0.1, // front square
            0.05, -0.25, -0.1,
            -0.05, -0.75, -0.1,
            0.05, -0.75, -0.1,
            -0.05, -0.25, 0.1, // back square
            0.05, -0.25, 0.1,
            -0.05, -0.75, 0.1,
            0.05, -0.75, 0.1,
            -0.1, -0.25, -0.05, // left square
            -0.1, -0.25, 0.05,
            -0.1, -0.75, -0.05,
            -0.1, -0.75, 0.05,
            0.1, -0.25, -0.05, // right square
            0.1, -0.25, 0.05,
            0.1, -0.75, -0.05,
            0.1, -0.75, 0.05,
            -0.05, 0.0, -0.05, // top square
            0.05, 0.0, -0.05,
            -0.05, 0.0, 0.05,
            0.05, 0.0, 0.05,
            -0.05, -1.0, -0.05, // bottom square
            0.05, -1.0, -0.05,
            -0.05, -1.0, 0.05,
            0.05, -1.0, 0.05
        ];

        let indices = vec![
            0, 1, 2, // squares
            2, 3, 1,
            4, 5, 6,
            6, 7, 5,
            8, 9, 10,
            10, 11, 9,
            12, 13, 14,
            14, 15, 13,
            16, 17, 18,
            18, 19, 17,
            20, 21, 22,
            22, 23, 21,
            0, 2, 8, // connecting squares
            2, 8, 10,
            1, 3, 12,
            3, 12, 14,
            9, 11, 6,
            4, 6, 9,
            13, 15, 5,
            15, 5, 7,
            16, 17, 0,
            0, 1, 17,
            18, 19, 4,
            19, 4, 5,
            16, 18, 8,
            18, 8, 9,
            17, 19, 12,
            19, 12, 13,
            20, 21, 2,
            2, 3, 21,
            22, 23, 6,
            6, 7, 23,
            20, 22, 10,
            10, 11, 22,
            21, 23, 14,
            14, 15, 23,
            0, 8, 16, // triangles front
            1, 12, 17,
            2, 10, 20,
            3, 14, 21,
            4, 9, 18, // triangles back
            5, 13, 19,
            6, 11, 22,
            7, 15, 23
        ];

        Mesh { vertices, indices }
    }
}


pub struct HouseCentipede {
    body_model: Model,
    legseg_model: Model,
    pub transform: Transform,
    pub motive: Motive,
    legs: [Leg; 30],

    last_group_step_time: f32,
    anim_time: f32,
}

impl HouseCentipede {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        let transform = Transform::new(x, y, z, 0.0, 0.0, 0.0);
        let motive = Motive::new();

        let body_matrix = transform.to_matrix();

        let attatchments: [LegAttach; 30] = Self::create_legattach();
        let leg_scale: [f32; 30] = Self::leg_scale();

        let legs: [Leg; 30] = std::array::from_fn(|i| {
            let att = &attatchments[i];
            let scale = leg_scale[i];
            let mut leg = Leg::default();

            let outward_dir = Vec3::new(att.offset.x, 0.0, att.offset.z).normalize();

            leg.coxa_base_yaw = att.base_yaw;
            leg.knee_max_yaw = 5.0;

            leg.coxa.transform.x = att.offset.x;
            leg.coxa.transform.y = att.offset.y;
            leg.coxa.transform.z = att.offset.z;
            leg.coxa.transform.yaw = att.base_yaw;
            leg.coxa.transform.pit = 20.0_f32.to_radians();
            leg.coxa.length = 2.0 * scale;

            leg.femur.transform.x = 0.0;
            leg.femur.transform.y = -leg.coxa.length;
            leg.femur.transform.z = 0.0;
            leg.femur.transform.pit = 40.0_f32.to_radians();
            leg.femur.length = 2.0 * scale;

            leg.tibia.transform.y = -leg.femur.length;
            leg.tibia.transform.pit = (-70.0_f32).to_radians();
            leg.tibia.length = 3.0 * scale;

            leg.tarsus.transform.y = -leg.tibia.length;
            leg.tarsus.length = 0.5 * scale;

            let leg_reach = leg.coxa.length + leg.femur.length
                + leg.tibia.length + leg.tarsus.length;
            let horiz_dist = leg_reach * 0.3;
            let drop_dist = leg_reach * 0.1;

            leg.rest_position = att.offset
                + outward_dir * horiz_dist
                + Vec3::new(0.0, -drop_dist, 0.0);

            let world_rest = body_matrix.transform_point3(leg.rest_position);
            leg.current_target = world_rest;
            leg.step_time = 1.0;
            leg.step_offset = if i % 2 == 0 { 0.0 } else { 0.15 };
            leg.step_start = world_rest;
            leg.step_end = world_rest;

            leg.coxa_base_pitch = (-45.0_f32).to_radians();

            leg
        });

        HouseCentipede {
            transform,
            motive,
            body_model: Model::new(Self::create_body_mesh()),
            legseg_model: Model::new(Self::create_legseg_mesh()),
            legs,
            last_group_step_time: 0.0,
            anim_time: 0.0,
        }
    }

    pub fn draw(&self, model_loc: GLint) {
        let body_mat: Mat4 = self.transform.to_matrix();
        unsafe { gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, body_mat.as_ref().as_ptr()) };
        self.body_model.draw();

        for leg in &self.legs {
            let coxa_mat = body_mat * leg.coxa.transform.to_matrix();
            let femur_mat = coxa_mat * leg.femur.transform.to_matrix();
            let tibia_mat = femur_mat * leg.tibia.transform.to_matrix();
            let tarsus_mat = tibia_mat * leg.tarsus.transform.to_matrix();

            struct SegDraw {
                mat: *const Mat4,
                length: f32,
            }

            let segs: [SegDraw; 4] = [
                SegDraw {mat: &coxa_mat, length: leg.coxa.length},
                SegDraw {mat: &femur_mat, length: leg.femur.length},
                SegDraw {mat: &tibia_mat, length: leg.tibia.length},
                SegDraw {mat: &tarsus_mat, length: leg.tarsus.length},
            ];

            for seg in &segs {
                unsafe {
                    let final_mat = *seg.mat * Mat4::from_scale(Vec3::new(1.0, seg.length, 1.0));
                    gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, final_mat.as_ref().as_ptr());
                }
                self.legseg_model.draw();
            }
        }
    }

    pub fn update(&mut self, delta_time: f32) {
        const STEP_TRIGGER_DIST: f32 = 1.0;
        const STEP_HEIGHT: f32 = 5.0;

        let step_speed = 10.0 * (self.motive.speed + 1.0);
        let group_stagger = 0.1 * (10.0 / self.motive.speed);

        self.anim_time += delta_time;

        let model = self.transform.to_matrix();
        let inv_body = model.inverse();

        for (i, leg) in self.legs.iter_mut().enumerate() {
            let group_a = (i % 15) % 2 == 0;

            let ideal_world_position: Vec3 = (model
                * Vec4::from((leg.rest_position, 1.0))).truncate();
            
            if leg.step_time >= 1.0 {
                leg.step_cool -= delta_time;
                let drift = (ideal_world_position - leg.current_target).length();

                let stagger_ok = group_a 
                    || (self.anim_time - self.last_group_step_time > group_stagger);

                if drift > STEP_TRIGGER_DIST && leg.step_cool <= 0.0 && stagger_ok {
                    leg.step_start = leg.current_target;
                    leg.step_end = ideal_world_position;
                    leg.step_time = 0.0;
                    leg.step_cool = 0.25;

                    if group_a {
                        self.last_group_step_time = self.anim_time;
                    }
                }
            }
            else {
                leg.step_time = (leg.step_time + delta_time * step_speed).min(1.0);

                let flat = leg.step_start.lerp(leg.step_end, leg.step_time);
                let lift = (leg.step_time * PI).sin() * STEP_HEIGHT;

                leg.current_target = flat + Vec3::new(0.0, lift, 0.0);
            }

            let target = leg.current_target;
            leg.solve_leg_ik(target, inv_body, delta_time);
        }
    }

    pub fn behave(&mut self, delta_time: f32) {
        let mut rng = rand::rng();

        if self.motive.move_cool <= 0.0 {
            self.motive.move_time -= delta_time;
            if self.motive.move_time <= 0.0 {
                self.motive.move_cool = rng.random_range(0.0..2.0);
            }
        }
        else {
            self.motive.speed = 0.0;
            self.motive.move_cool -= delta_time;
            if self.motive.move_cool <= 0.01 {
                self.motive.move_time = rng.random_range(3.0..10.0);
                self.motive.speed = rng.random_range(5.0..10.0);
                self.motive.yaw_time = 0.5;
                self.motive.delta_yaw = rng.random_range(-5.0..5.0);
            }
        }

        let face_x = -self.transform.yaw.sin() * self.transform.pit.cos();
        let face_z = -self.transform.yaw.cos() * self.transform.pit.cos();

        let facing_out = 
            (self.transform.x > BORDER_RIGHT && face_x > 0.0) ||
            (self.transform.x < BORDER_LEFT && face_x < 0.0) ||
            (self.transform.z > BORDER_BOTTOM && face_z > 0.0) ||
            (self.transform.z < BORDER_TOP && face_z < 0.0); 

        if facing_out {
            self.transform.yaw += delta_time;
        }
        else if self.motive.yaw_time > 0.0 {
            self.transform.yaw += self.motive.delta_yaw * delta_time;
            self.motive.yaw_time -= delta_time;
        }
        else {
            self.transform.x += face_x * self.motive.speed * delta_time;
            self.transform.z += face_z * self.motive.speed * delta_time;
        }
    } 

    fn create_legattach() -> [LegAttach; 30] {
        [
            LegAttach { offset: Vec3::new(0.25, -0.5,  1.125),   base_yaw: (-175.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5,  0.91965), base_yaw: (-165.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5,  0.7143),  base_yaw: (-150.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5,  0.50895), base_yaw: (-140.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5,  0.30355), base_yaw: (-130.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5,  0.0982),  base_yaw: (-120.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.10715), base_yaw: (-115.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.3125),  base_yaw: (-110.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.51785), base_yaw: (-105.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.7232),  base_yaw: (-100.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.92855), base_yaw: (-95.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -1.13395), base_yaw: (-90.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -1.3393),  base_yaw: (-80.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -1.54465), base_yaw: (-60.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -1.75),    base_yaw: (-50.0_f32).to_radians() },

            LegAttach { offset: Vec3::new(-0.25, -0.5,  1.125),   base_yaw: (175.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5,  0.91965), base_yaw: (165.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5,  0.7143),  base_yaw: (150.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5,  0.50895), base_yaw: (140.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5,  0.30355), base_yaw: (130.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5,  0.0982),  base_yaw: (120.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.10715), base_yaw: (115.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.3125),  base_yaw: (110.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.51785), base_yaw: (105.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.7232),  base_yaw: (100.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.92855), base_yaw: (95.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -1.13395), base_yaw: (90.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -1.3393),  base_yaw: (80.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -1.54465), base_yaw: (60.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -1.75),    base_yaw: (50.0_f32).to_radians() },
        ]
    }

    fn leg_scale() -> [f32; 30] {
        [
            1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 0.95, 0.9, 0.85,
            0.8, 0.75, 0.7, 0.65, 0.6,

            1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 0.95, 0.9, 0.85,
            0.8, 0.75, 0.7, 0.65, 0.6,
        ]
    } 

    fn create_body_mesh() -> Mesh {
        let vertices = vec![
            -0.125, -0.135, -2.0,    // front square
            0.125, -0.135, -2.0,
            -0.125, -0.465, -2.0,
            0.125, -0.465, -2.0,
            -0.125, -0.135,  2.5,    // back square
            0.125, -0.135,  2.5,
            -0.125, -0.465,  2.5,
            0.125, -0.465,  2.5,
            -0.25,  -0.135, -0.875,  // left square
            -0.25,  -0.135,  1.375,
            -0.25,  -0.465, -0.875,
            -0.25,  -0.465,  1.375,
            0.25,  -0.135, -0.875,  // right square
            0.25,  -0.135,  1.375,
            0.25,  -0.465, -0.875,
            0.25,  -0.465,  1.375,
            -0.125,  0.03,  -0.875,  // top square
            0.125,  0.03,  -0.875,
            -0.125,  0.03,   1.375,
            0.125,  0.03,   1.375,
            -0.125, -0.63,  -0.875,  // bottom square
            0.125, -0.63,  -0.875,
            -0.125, -0.63,   1.375,
            0.125, -0.63,   1.375,
        ];

        let indices = vec![
            0, 1, 2, // squares
            2, 3, 1,
            4, 5, 6,
            6, 7, 5,
            8, 9, 10,
            10, 11, 9,
            12, 13, 14,
            14, 15, 13,
            16, 17, 18,
            18, 19, 17,
            20, 21, 22,
            22, 23, 21,
            0, 2, 8, // connecting squares
            2, 8, 10,
            1, 3, 12,
            3, 12, 14,
            9, 11, 6,
            4, 6, 9,
            13, 15, 5,
            15, 5, 7,
            16, 17, 0,
            0, 1, 17,
            18, 19, 4,
            19, 4, 5,
            16, 18, 8,
            18, 8, 9,
            17, 19, 12,
            19, 12, 13,
            20, 21, 2,
            2, 3, 21,
            22, 23, 6,
            6, 7, 23,
            20, 22, 10,
            10, 11, 22,
            21, 23, 14,
            14, 15, 23,
            0, 8, 16, // triangles front
            1, 12, 17,
            2, 10, 20,
            3, 14, 21,
            4, 9, 18, // triangles back
            5, 13, 19,
            6, 11, 22,
            7, 15, 23,
        ];

        Mesh { vertices, indices }
    }

    fn create_legseg_mesh() -> Mesh {
        let vertices = vec![
            -0.0025, -0.25, -0.005, // front square
            0.0025, -0.25, -0.005,
            -0.0025, -0.75, -0.005,
            0.0025, -0.75, -0.005,
            -0.0025, -0.25, 0.005, // back square
            0.0025, -0.25, 0.005,
            -0.0025, -0.75, 0.005,
            0.0025, -0.75, 0.005,
            -0.005, -0.25, -0.0025, // left square
            -0.005, -0.25, 0.0025,
            -0.005, -0.75, -0.0025,
            -0.005, -0.75, 0.0025,
            0.005, -0.25, -0.0025, // right square
            0.005, -0.25, 0.0025,
            0.005, -0.75, -0.0025,
            0.005, -0.75, 0.0025,
            -0.0025, 0.0, -0.0025, // top square
            0.0025, 0.0, -0.0025,
            -0.0025, 0.0, 0.0025,
            0.0025, 0.0, 0.0025,
            -0.0025, -1.0, -0.0025, // bottom square
            0.0025, -1.0, -0.0025,
            -0.0025, -1.0, 0.0025,
            0.0025, -1.0, 0.0025
        ];

        let indices = vec![
            0, 1, 2, // squares
            2, 3, 1,
            4, 5, 6,
            6, 7, 5,
            8, 9, 10,
            10, 11, 9,
            12, 13, 14,
            14, 15, 13,
            16, 17, 18,
            18, 19, 17,
            20, 21, 22,
            22, 23, 21,
            0, 2, 8, // connecting squares
            2, 8, 10,
            1, 3, 12,
            3, 12, 14,
            9, 11, 6,
            4, 6, 9,
            13, 15, 5,
            15, 5, 7,
            16, 17, 0,
            0, 1, 17,
            18, 19, 4,
            19, 4, 5,
            16, 18, 8,
            18, 8, 9,
            17, 19, 12,
            19, 12, 13,
            20, 21, 2,
            2, 3, 21,
            22, 23, 6,
            6, 7, 23,
            20, 22, 10,
            10, 11, 22,
            21, 23, 14,
            14, 15, 23,
            0, 8, 16, // triangles front
            1, 12, 17,
            2, 10, 20,
            3, 14, 21,
            4, 9, 18, // triangles back
            5, 13, 19,
            6, 11, 22,
            7, 15, 23
        ];

        Mesh { vertices, indices }
    }
}

pub struct Harvestman {
    body_model: Model,
    legseg_model: Model,
    pub transform: Transform,
    pub motive: Motive,
    legs: [Leg; 8],

    last_group_step_time: f32,
    anim_time: f32,
}

impl Harvestman {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        let transform = Transform::new(x, y, z, 0.0, 0.0, 0.0);
        let motive = Motive::new();

        let body_matrix = transform.to_matrix();

        let attatchments: [LegAttach; 8] = Self::create_legattach();
        let leg_scale: [f32; 8] = Self::leg_scale();

        let legs: [Leg; 8] = std::array::from_fn(|i| {
            let att = &attatchments[i];
            let scale = leg_scale[i];
            let mut leg = Leg::default();

            let outward_dir = Vec3::new(-att.base_yaw.sin(), 0.0, -att.base_yaw.cos());

            leg.coxa_base_yaw = att.base_yaw;
            leg.knee_max_yaw = 60.0;

            leg.coxa.transform.x = att.offset.x;
            leg.coxa.transform.y = att.offset.y;
            leg.coxa.transform.z = att.offset.z;
            leg.coxa.transform.yaw = att.base_yaw;
            leg.coxa.transform.pit = 20.0_f32.to_radians();
            leg.coxa.length = 1.5 * scale;

            leg.femur.transform.x = 0.0;
            leg.femur.transform.y = -leg.coxa.length;
            leg.femur.transform.z = 0.0;
            leg.femur.transform.pit = 40.0_f32.to_radians();
            leg.femur.length = 1.5 * scale;

            leg.tibia.transform.y = -leg.femur.length;
            leg.tibia.transform.pit = (-70.0_f32).to_radians();
            leg.tibia.length = 3.0 * scale;

            leg.tarsus.transform.y = -leg.tibia.length;
            leg.tarsus.length = 1.0 * scale;

            let leg_reach = leg.coxa.length + leg.femur.length
                + leg.tibia.length + leg.tarsus.length;
            let horiz_dist = leg_reach * 1.0;
            let drop_dist = leg_reach * 0.2;

            leg.rest_position = att.offset
                + outward_dir * horiz_dist
                + Vec3::new(0.0, -drop_dist, 0.0);

            let world_rest = body_matrix.transform_point3(leg.rest_position);
            leg.current_target = world_rest;
            leg.step_time = 1.0;
            leg.step_offset = if i % 2 == 0 { 0.0 } else { 0.15 };
            leg.step_start = world_rest;
            leg.step_end = world_rest;

            leg.coxa_base_pitch = (-10.0_f32).to_radians();

            leg
        });

        Harvestman {
            transform,
            motive,
            body_model: Model::new(Self::create_body_mesh()),
            legseg_model: Model::new(Self::create_legseg_mesh()),
            legs,
            last_group_step_time: 0.0,
            anim_time: 0.0,
        }
    }

    pub fn draw(&self, model_loc: GLint) {
        let body_mat: Mat4 = self.transform.to_matrix();
        unsafe { gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, body_mat.as_ref().as_ptr()) };
        self.body_model.draw();

        for leg in &self.legs {
            let coxa_mat = body_mat * leg.coxa.transform.to_matrix();
            let femur_mat = coxa_mat * leg.femur.transform.to_matrix();
            let tibia_mat = femur_mat * leg.tibia.transform.to_matrix();
            let tarsus_mat = tibia_mat * leg.tarsus.transform.to_matrix();

            struct SegDraw {
                mat: *const Mat4,
                length: f32,
                width: f32,
            }

            let segs: [SegDraw; 4] = [
                SegDraw {mat: &coxa_mat, length: leg.coxa.length, width: 1.6},
                SegDraw {mat: &femur_mat, length: leg.femur.length, width: 1.2},
                SegDraw {mat: &tibia_mat, length: leg.tibia.length, width: 0.8},
                SegDraw {mat: &tarsus_mat, length: leg.tarsus.length, width: 0.4},
            ];

            for seg in &segs {
                unsafe {
                    let final_mat = *seg.mat * Mat4::from_scale(Vec3::new(seg.width, seg.length, seg.width));
                    gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, final_mat.as_ref().as_ptr());
                }
                self.legseg_model.draw();
            }
        }
    }

    pub fn update(&mut self, delta_time: f32) {
        const STEP_TRIGGER_DIST: f32 = 3.0;
        const STEP_HEIGHT: f32 = 10.0;

        let stride = STEP_TRIGGER_DIST + 0.1 * self.motive.speed;
        let step_speed = (self.motive.speed / (2.0 * stride)).max(5.0);
        let group_stagger = 1.0 / self.motive.speed;

        self.anim_time += delta_time;

        let model = self.transform.to_matrix();
        let inv_body = model.inverse();

        for (i, leg) in self.legs.iter_mut().enumerate() {
            let group_a = i % 2 == 0;

            let ideal_world_position: Vec3 = (model
                * Vec4::from((leg.rest_position, 1.0))).truncate();

            if leg.step_time >= 1.0 {
                leg.step_cool -= delta_time;
                let drift = (ideal_world_position - leg.current_target).length();

                let stagger_ok = group_a 
                    || (self.anim_time - self.last_group_step_time > group_stagger);

                if drift > STEP_TRIGGER_DIST && leg.step_cool <= 0.0 && stagger_ok {
                    leg.step_start = leg.current_target;
                    leg.step_end = ideal_world_position;
                    leg.step_time = 0.0;
                    leg.step_cool = 0.25;

                    if group_a {
                        self.last_group_step_time = self.anim_time;
                    }
                }
            }
            else {
                leg.step_time = (leg.step_time + delta_time * step_speed).min(1.0);

                let flat = leg.step_start.lerp(leg.step_end, leg.step_time);
                let lift = (leg.step_time * PI).sin() * STEP_HEIGHT;

                leg.current_target = flat + Vec3::new(0.0, lift, 0.0);
            }

            let target = leg.current_target;
            leg.solve_leg_ik(target, inv_body, delta_time);
        }
    }

    pub fn behave(&mut self, delta_time: f32) {
        let mut rng = rand::rng();

        if self.motive.move_cool <= 0.0 {
            self.motive.move_time -= delta_time;
            if self.motive.move_time <= 0.0 {
                self.motive.move_cool = rng.random_range(5.0..10.0);
            }
        }
        else {
            self.motive.speed = 0.0;
            self.motive.move_cool -= delta_time;
            if self.motive.move_cool <= 0.01 {
                self.motive.move_time = rng.random_range(3.0..10.0);
                self.motive.speed = rng.random_range(3.0..5.0);
                self.motive.yaw_time = 0.5;
                self.motive.delta_yaw = rng.random_range(-5.0..5.0);
            }
        }

        let face_x = -self.transform.yaw.sin() * self.transform.pit.cos();
        let face_z = -self.transform.yaw.cos() * self.transform.pit.cos();

        let facing_out = 
            (self.transform.x > BORDER_RIGHT && face_x > 0.0) ||
            (self.transform.x < BORDER_LEFT && face_x < 0.0) ||
            (self.transform.z > BORDER_BOTTOM && face_z > 0.0) ||
            (self.transform.z < BORDER_TOP && face_z < 0.0);

        if facing_out {
            self.transform.yaw += delta_time;
        }
        else if self.motive.yaw_time > 0.0 {
            self.transform.yaw += self.motive.delta_yaw * delta_time;
            self.motive.yaw_time -= delta_time;
        }
        else {
            self.transform.x += face_x * self.motive.speed * delta_time;
            self.transform.z += face_z * self.motive.speed * delta_time;
        }
    } 

    fn create_legattach() -> [LegAttach; 8] {
        [
            LegAttach { offset: Vec3::new(0.25, -0.5, 0.3), base_yaw: (-120.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, 0.0), base_yaw: (-100.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.2), base_yaw: (-80.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.75), base_yaw: (-60.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, 0.3), base_yaw: (120.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, 0.0), base_yaw: (100.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.2), base_yaw: (80.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.75), base_yaw: (60.0_f32).to_radians() },
        ]
    }

    fn leg_scale() -> [f32; 8] {
        [
            0.8, 1.0, 1.1, 1.0,
            0.8, 1.0, 1.1, 1.0,
        ]
    }

    fn create_body_mesh() -> Mesh {
        let vertices = vec![
            -0.16875, -0.18225, -0.600,  // front square
            0.16875, -0.18225, -0.600,
            -0.16875, -0.62775, -0.600,
            0.16875, -0.62775, -0.600,
            -0.16875, -0.18225,  0.300,  // back square
            0.16875, -0.18225,  0.300,
            -0.16875, -0.62775,  0.300,
            0.16875, -0.62775,  0.300,
            -0.33750, -0.18225, -0.375,  // left square
            -0.33750, -0.18225,  0.075,
            -0.33750, -0.62775, -0.375,
            -0.33750, -0.62775,  0.075,
            0.33750, -0.18225, -0.375,  // right square
            0.33750, -0.18225,  0.075,
            0.33750, -0.62775, -0.375,
            0.33750, -0.62775,  0.075,
            -0.16875,  0.04050, -0.375,  // top square
            0.16875,  0.04050, -0.375,
            -0.16875,  0.04050,  0.075,
            0.16875,  0.04050,  0.075,
            -0.16875, -0.85050, -0.375,  // bottom square
            0.16875, -0.85050, -0.375,
            -0.16875, -0.85050,  0.075,
            0.16875, -0.85050,  0.075,
        ];

        let indices = vec![
            0, 1, 2, // squares
            2, 3, 1,
            4, 5, 6,
            6, 7, 5,
            8, 9, 10,
            10, 11, 9,
            12, 13, 14,
            14, 15, 13,
            16, 17, 18,
            18, 19, 17,
            20, 21, 22,
            22, 23, 21,
            0, 2, 8, // connecting squares
            2, 8, 10,
            1, 3, 12,
            3, 12, 14,
            9, 11, 6,
            4, 6, 9,
            13, 15, 5,
            15, 5, 7,
            16, 17, 0,
            0, 1, 17,
            18, 19, 4,
            19, 4, 5,
            16, 18, 8,
            18, 8, 9,
            17, 19, 12,
            19, 12, 13,
            20, 21, 2,
            2, 3, 21,
            22, 23, 6,
            6, 7, 23,
            20, 22, 10,
            10, 11, 22,
            21, 23, 14,
            14, 15, 23,
            0, 8, 16, // triangles front
            1, 12, 17,
            2, 10, 20,
            3, 14, 21,
            4, 9, 18, // triangles back
            5, 13, 19,
            6, 11, 22,
            7, 15, 23,
        ];

        Mesh { vertices, indices }
    }

    fn create_legseg_mesh() -> Mesh {
        let vertices = vec![
            -0.0025, -0.25, -0.005, // front square
            0.0025, -0.25, -0.005,
            -0.0025, -0.75, -0.005,
            0.0025, -0.75, -0.005,
            -0.0025, -0.25, 0.005, // back square
            0.0025, -0.25, 0.005,
            -0.0025, -0.75, 0.005,
            0.0025, -0.75, 0.005,
            -0.005, -0.25, -0.0025, // left square
            -0.005, -0.25, 0.0025,
            -0.005, -0.75, -0.0025,
            -0.005, -0.75, 0.0025,
            0.005, -0.25, -0.0025, // right square
            0.005, -0.25, 0.0025,
            0.005, -0.75, -0.0025,
            0.005, -0.75, 0.0025,
            -0.0025, 0.0, -0.0025, // top square
            0.0025, 0.0, -0.0025,
            -0.0025, 0.0, 0.0025,
            0.0025, 0.0, 0.0025,
            -0.0025, -1.0, -0.0025, // bottom square
            0.0025, -1.0, -0.0025,
            -0.0025, -1.0, 0.0025,
            0.0025, -1.0, 0.0025
        ];

        let indices = vec![
            0, 1, 2, // squares
            2, 3, 1,
            4, 5, 6,
            6, 7, 5,
            8, 9, 10,
            10, 11, 9,
            12, 13, 14,
            14, 15, 13,
            16, 17, 18,
            18, 19, 17,
            20, 21, 22,
            22, 23, 21,
            0, 2, 8, // connecting squares
            2, 8, 10,
            1, 3, 12,
            3, 12, 14,
            9, 11, 6,
            4, 6, 9,
            13, 15, 5,
            15, 5, 7,
            16, 17, 0,
            0, 1, 17,
            18, 19, 4,
            19, 4, 5,
            16, 18, 8,
            18, 8, 9,
            17, 19, 12,
            19, 12, 13,
            20, 21, 2,
            2, 3, 21,
            22, 23, 6,
            6, 7, 23,
            20, 22, 10,
            10, 11, 22,
            21, 23, 14,
            14, 15, 23,
            0, 8, 16, // triangles front
            1, 12, 17,
            2, 10, 20,
            3, 14, 21,
            4, 9, 18, // triangles back
            5, 13, 19,
            6, 11, 22,
            7, 15, 23
        ];

        Mesh { vertices, indices }
    }
}

pub struct CraneFly {
    body_model: Model,
    legseg_model: Model,
    wing_model: Model,

    pub transform: Transform,
    pub motive: Motive,

    legs: [Leg; 6],
    wings: [Wing; 2],

    last_group_step_time: f32,
    anim_time: f32,
}

impl CraneFly {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        let transform = Transform::new(x, y, z, 0.0, 0.0, 0.0);
        let motive = Motive::new();

        let body_matrix = transform.to_matrix();

        let leg_attachments: [LegAttach; 6] = Self::create_legattach();
        let leg_scale: [f32; 6] = Self::leg_scale();

        let legs: [Leg; 6] = std::array::from_fn(|i| {
            let att = &leg_attachments[i];
            let scale = leg_scale[i];
            let mut leg = Leg::default();

            let outward_dir = Vec3::new(-att.base_yaw.sin(), 0.0, -att.base_yaw.cos());

            leg.coxa_base_yaw = att.base_yaw;
            leg.knee_max_yaw = 60.0;

            leg.coxa.transform.x = att.offset.x;
            leg.coxa.transform.y = att.offset.y;
            leg.coxa.transform.z = att.offset.z;
            leg.coxa.transform.yaw = att.base_yaw;
            leg.coxa.transform.pit = 20.0_f32.to_radians();
            leg.coxa.length = 2.5 * scale;

            leg.femur.transform.x = 0.0;
            leg.femur.transform.y = -leg.coxa.length;
            leg.femur.transform.z = 0.0;
            leg.femur.transform.pit = 40.0_f32.to_radians();
            leg.femur.length = 2.5 * scale;

            leg.tibia.transform.y = -leg.femur.length;
            leg.tibia.transform.pit = (-70.0_f32).to_radians();
            leg.tibia.length = 4.0 * scale;

            leg.tarsus.transform.y = -leg.tibia.length;
            leg.tarsus.length = 1.0 * scale;

            let leg_reach = leg.coxa.length + leg.femur.length
                + leg.tibia.length + leg.tarsus.length;
            let horiz_dist = leg_reach * 1.0;
            let drop_dist = leg_reach * 0.4;

            leg.rest_position = att.offset
                + outward_dir * horiz_dist
                + Vec3::new(0.0, -drop_dist, 0.0);

            let world_rest = body_matrix.transform_point3(leg.rest_position);
            leg.current_target = world_rest;
            leg.step_time = 1.0;
            leg.step_offset = if i % 2 == 0 { 0.0 } else { 0.15 };
            leg.step_start = world_rest;
            leg.step_end = world_rest;

            leg.coxa_base_pitch = (0.0_f32).to_radians();

            leg
        });

        let wing_attachments: [WingAttach; 2] = Self::create_wingattach();

        let wings: [Wing; 2] = std::array::from_fn(|i| {
            let att = &wing_attachments[i];
            let mut wing = Wing::default();

            let outward_dir = Vec3::new(-att.base_yaw.sin(), 0.0, -att.base_yaw.cos());

            let horiz_dist = 0.5;
            let drop_dist = 0.5;

            wing.rest_position = att.offset
                + outward_dir * horiz_dist
                + Vec3::new(0.0, -drop_dist, 0.0);

            wing
        });

        CraneFly {
            transform,
            motive,
            body_model: Model::new(Self::create_body_mesh()),
            legseg_model: Model::new(Self::create_legseg_mesh()),
            wing_model: Model::new(Self::create_wing_mesh()),
            legs,
            wings,
            last_group_step_time: 0.0,
            anim_time: 0.0,
        }
    }

    pub fn draw(&self, model_loc: GLint) {
        let body_mat: Mat4 = self.transform.to_matrix();
        unsafe { gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, body_mat.as_ref().as_ptr()) };
        self.body_model.draw();

        for leg in &self.legs {
            let coxa_mat = body_mat * leg.coxa.transform.to_matrix();
            let femur_mat = coxa_mat * leg.femur.transform.to_matrix();
            let tibia_mat = femur_mat * leg.tibia.transform.to_matrix();
            let tarsus_mat = tibia_mat * leg.tarsus.transform.to_matrix();

           struct SegDraw {
                mat: *const Mat4,
                length: f32,
                width: f32,
            }

            let segs: [SegDraw; 4] = [
                SegDraw {mat: &coxa_mat, length: leg.coxa.length, width: 1.6},
                SegDraw {mat: &femur_mat, length: leg.femur.length, width: 1.2},
                SegDraw {mat: &tibia_mat, length: leg.tibia.length, width: 0.8},
                SegDraw {mat: &tarsus_mat, length: leg.tarsus.length, width: 0.4},
            ];

            for seg in &segs {
                unsafe {
                    let final_mat = *seg.mat * Mat4::from_scale(Vec3::new(seg.width, seg.length, seg.width));
                    gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, final_mat.as_ref().as_ptr());
                }
                self.legseg_model.draw();
            }
        }

        for wing in &self.wings {
            let wing_mat = body_mat * wing.transform.to_matrix();

            struct WingDraw {
                mat: *const Mat4,
            }

            let wingy: WingDraw = WingDraw {mat: &wing_mat};

            unsafe {
                let final_mat = *wingy.mat * Mat4::from_scale(Vec3::new(1.0, 1.0, 1.0));
                gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, final_mat.as_ref().as_ptr());
            }
            self.wing_model.draw();
        }
    }

    pub fn update(&mut self, delta_time: f32) {
        const STEP_TRIGGER_DIST: f32 = 3.0;
        const STEP_HEIGHT: f32 = 5.0;

        let stride = STEP_TRIGGER_DIST + 0.1 * self.motive.speed;
        let step_speed = (self.motive.speed / (2.0 * stride)).max(5.0);
        let group_stagger = 1.0 / self.motive.speed;

        self.anim_time += delta_time;

        let model = self.transform.to_matrix();
        let inv_body = model.inverse();

        for (i, leg) in self.legs.iter_mut().enumerate() {
            let group_a = i % 2 == 0;

            let ideal_world_position: Vec3 = (model
                * Vec4::from((leg.rest_position, 1.0))).truncate();

            if leg.step_time >= 1.0 {
                leg.step_cool -= delta_time;
                let drift = (ideal_world_position - leg.current_target).length();

                let stagger_ok = group_a 
                    || (self.anim_time - self.last_group_step_time > group_stagger);

                if drift > STEP_TRIGGER_DIST && leg.step_cool <= 0.0 && stagger_ok {
                    leg.step_start = leg.current_target;
                    leg.step_end = ideal_world_position;
                    leg.step_time = 0.0;
                    leg.step_cool = 0.25;

                    if group_a {
                        self.last_group_step_time = self.anim_time;
                    }
                }
            }
            else {
                leg.step_time = (leg.step_time + delta_time * step_speed).min(1.0);

                let flat = leg.step_start.lerp(leg.step_end, leg.step_time);
                let lift = (leg.step_time * PI).sin() * STEP_HEIGHT;

                leg.current_target = flat + Vec3::new(0.0, lift, 0.0);
            }

            let target = leg.current_target;
            leg.solve_leg_ik(target, inv_body, delta_time);
        }

        for (i, wing) in self.wings.iter_mut().enumerate() {
            wing.flap(inv_body, i as f32, delta_time);
        }
    }

    pub fn behave(&mut self, delta_time: f32) {
        let mut rng = rand::rng();

        if self.motive.move_cool <= 0.0 {
            self.motive.move_time -= delta_time;
            if self.motive.move_time <= 0.0 {
                self.motive.move_cool = rng.random_range(1.0..5.0);
            }
        }
        else {
            self.motive.speed = 0.0;
            self.motive.move_cool -= delta_time;
            if self.motive.move_cool <= 0.01 {
                self.motive.move_time = rng.random_range(3.0..10.0);
                self.motive.speed = rng.random_range(5.0..10.0);
                self.motive.yaw_time = 0.5;
                self.motive.delta_yaw = rng.random_range(-5.0..5.0);
            }
        }

        let face_x = -self.transform.yaw.sin() * self.transform.pit.cos();
        let face_z = -self.transform.yaw.cos() * self.transform.pit.cos();

        let facing_out = 
            (self.transform.x > BORDER_RIGHT && face_x > 0.0) ||
            (self.transform.x < BORDER_LEFT && face_x < 0.0) ||
            (self.transform.z > BORDER_BOTTOM && face_z > 0.0) ||
            (self.transform.z < BORDER_TOP && face_z < 0.0);

        if facing_out {
            self.transform.yaw += delta_time;
        }
        else if self.motive.yaw_time > 0.0 {
            self.transform.yaw += self.motive.delta_yaw * delta_time;
            self.motive.yaw_time -= delta_time;
        }
        else {
            self.transform.x += face_x * self.motive.speed * delta_time;
            self.transform.z += face_z * self.motive.speed * delta_time;
        }
    } 

    fn create_legattach() -> [LegAttach; 6] {
        [
            LegAttach { offset: Vec3::new(0.25, -0.5, 0.3), base_yaw: (-135.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, 0.0), base_yaw: (-70.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.2), base_yaw: (-15.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, 0.3), base_yaw: (135.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, 0.0), base_yaw: (70.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.2), base_yaw: (15.0_f32).to_radians() },
        ]
    }

    fn leg_scale() -> [f32; 6] {
        [
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0,
        ]
    }

    fn create_wingattach() -> [WingAttach; 2] {
        [
            WingAttach { offset: Vec3::new(0.25, 0.0, 2.0), base_yaw: (-45.0_f32).to_radians() },
            WingAttach { offset: Vec3::new(-0.25, 0.0, 2.0), base_yaw: (45.0_f32).to_radians() },
        ]
    }

    fn create_body_mesh() -> Mesh {
        let vertices = vec![
            -0.09375,  -0.10125, -1.25,   // front square
            0.09375,  -0.10125, -1.25,
            -0.09375,  -0.34875, -1.25,
            0.09375,  -0.34875, -1.25,
            -0.09375,  -0.10125,  0.25,   // back square
            0.09375,  -0.10125,  0.25,
            -0.09375,  -0.34875,  0.25,
            0.09375,  -0.34875,  0.25,
            -0.1875,   -0.10125, -0.875,  // left square
            -0.1875,   -0.10125, -0.125,
            -0.1875,   -0.34875, -0.875,
            -0.1875,   -0.34875, -0.125,
            0.1875,   -0.10125, -0.875,  // right square
            0.1875,   -0.10125, -0.125,
            0.1875,   -0.34875, -0.875,
            0.1875,   -0.34875, -0.125,
            -0.09375,   0.0225,  -0.875,  // top square
            0.09375,   0.0225,  -0.875,
            -0.09375,   0.0225,  -0.125,
            0.09375,   0.0225,  -0.125,
            -0.09375,  -0.4725,  -0.875,  // bottom square
            0.09375,  -0.4725,  -0.875,
            -0.09375,  -0.4725,  -0.125,
            0.09375,  -0.4725,  -0.125,

            -0.125,    0.125,   -0.3125,  // front square
            0.125,    0.125,   -0.3125,
            -0.125,   -0.125,   -0.3125,
            0.125,   -0.125,   -0.3125,
            -0.0625,   0.0625,   5.1875, // back square
            0.0625,   0.0625,   5.1875,
            -0.0625,  -0.0625,   5.1875,
            0.0625,  -0.0625,   5.1875,
            -0.25,     0.125,    0.3125,  // left square
            -0.2,      0.1,      1.5625,
            -0.25,    -0.125,    0.3125,
            -0.25,    -0.125,    1.5625,
            0.25,     0.125,    0.3125,  // right square
            0.2,      0.1,      1.5625,
            0.25,    -0.125,    0.3125,
            0.25,    -0.125,    1.5625,
            -0.125,    0.25,     0.3125,  // top square
            0.125,    0.25,     0.3125,
            -0.1,      0.2,      1.5625,
            0.1,      0.2,      1.5625,
            -0.125,   -0.25,     0.3125,  // bottom square
            0.125,   -0.25,     0.3125,
            -0.125,   -0.25,     1.5625,
            0.125,   -0.25,     1.5625,
        ];

        let indices = vec![
            0, 1, 2, // squares
            2, 3, 1,
            4, 5, 6,
            6, 7, 5,
            8, 9, 10,
            10, 11, 9,
            12, 13, 14,
            14, 15, 13,
            16, 17, 18,
            18, 19, 17,
            20, 21, 22,
            22, 23, 21,
            0, 2, 8, // connecting squares
            2, 8, 10,
            1, 3, 12,
            3, 12, 14,
            9, 11, 6,
            4, 6, 9,
            13, 15, 5,
            15, 5, 7,
            16, 17, 0,
            0, 1, 17,
            18, 19, 4,
            19, 4, 5,
            16, 18, 8,
            18, 8, 9,
            17, 19, 12,
            19, 12, 13,
            20, 21, 2,
            2, 3, 21,
            22, 23, 6,
            6, 7, 23,
            20, 22, 10,
            10, 11, 22,
            21, 23, 14,
            14, 15, 23,
            0, 8, 16, // triangles front
            1, 12, 17,
            2, 10, 20,
            3, 14, 21,
            4, 9, 18, // triangles back
            5, 13, 19,
            6, 11, 22,
            7, 15, 23,

            24, 25, 26, // squares
            26, 27, 25,
            28, 29, 30,
            30, 31, 29,
            32, 33, 34,
            34, 35, 33,
            36, 37, 38,
            38, 39, 37,
            40, 41, 42,
            42, 43, 41,
            44, 45, 46,
            46, 47, 45,
            24, 26, 32, // connecting squares
            26, 32, 34,
            25, 27, 36,
            27, 36, 38,
            33, 35, 30,
            28, 30, 33,
            37, 39, 29,
            39, 29, 31,
            40, 41, 24,
            24, 25, 41,
            42, 43, 28,
            43, 28, 29,
            40, 42, 32,
            42, 32, 33,
            41, 43, 36,
            43, 36, 37,
            44, 45, 26,
            26, 27, 45,
            46, 47, 30,
            30, 31, 47,
            44, 46, 34,
            34, 35, 46,
            45, 47, 38,
            38, 39, 47,
            24, 32, 40, // triangles front
            25, 36, 41,
            26, 34, 44,
            27, 38, 45,
            28, 33, 42, // triangles back
            29, 37, 43,
            30, 35, 46,
            31, 39, 47
        ];

        Mesh { vertices, indices }
    }

    fn create_legseg_mesh() -> Mesh {
        let vertices = vec![
            -0.0025, -0.25, -0.005, // front square
            0.0025, -0.25, -0.005,
            -0.0025, -0.75, -0.005,
            0.0025, -0.75, -0.005,
            -0.0025, -0.25, 0.005, // back square
            0.0025, -0.25, 0.005,
            -0.0025, -0.75, 0.005,
            0.0025, -0.75, 0.005,
            -0.005, -0.25, -0.0025, // left square
            -0.005, -0.25, 0.0025,
            -0.005, -0.75, -0.0025,
            -0.005, -0.75, 0.0025,
            0.005, -0.25, -0.0025, // right square
            0.005, -0.25, 0.0025,
            0.005, -0.75, -0.0025,
            0.005, -0.75, 0.0025,
            -0.0025, 0.0, -0.0025, // top square
            0.0025, 0.0, -0.0025,
            -0.0025, 0.0, 0.0025,
            0.0025, 0.0, 0.0025,
            -0.0025, -1.0, -0.0025, // bottom square
            0.0025, -1.0, -0.0025,
            -0.0025, -1.0, 0.0025,
            0.0025, -1.0, 0.0025
        ];

        let indices = vec![
            0, 1, 2, // squares
            2, 3, 1,
            4, 5, 6,
            6, 7, 5,
            8, 9, 10,
            10, 11, 9,
            12, 13, 14,
            14, 15, 13,
            16, 17, 18,
            18, 19, 17,
            20, 21, 22,
            22, 23, 21,
            0, 2, 8, // connecting squares
            2, 8, 10,
            1, 3, 12,
            3, 12, 14,
            9, 11, 6,
            4, 6, 9,
            13, 15, 5,
            15, 5, 7,
            16, 17, 0,
            0, 1, 17,
            18, 19, 4,
            19, 4, 5,
            16, 18, 8,
            18, 8, 9,
            17, 19, 12,
            19, 12, 13,
            20, 21, 2,
            2, 3, 21,
            22, 23, 6,
            6, 7, 23,
            20, 22, 10,
            10, 11, 22,
            21, 23, 14,
            14, 15, 23,
            0, 8, 16, // triangles front
            1, 12, 17,
            2, 10, 20,
            3, 14, 21,
            4, 9, 18, // triangles back
            5, 13, 19,
            6, 11, 22,
            7, 15, 23
        ];

        Mesh { vertices, indices }
    }

    fn create_wing_mesh() -> Mesh {
        let vertices = vec![
            0.0, 0.0, 0.0,
            0.0, 5.0, 0.0,
            0.5, 5.0, 0.0,
            0.125, 5.5, 0.0,
            0.375, 5.5, 0.0,
            0.25, 5.75, 0.0,
        ];

        let indices = vec![
            0, 1, 2,  3, 4, 5,
            1, 2, 3,  3, 4, 2,
        ];

        Mesh { vertices, indices }
    }
}
