use gl::types::*;
use std::f32::consts::PI;
use glam::{Vec3, Vec4, Quat, Mat3, Mat4};
use rand::RngExt;

use crate::things::{Transform, Motive, Mesh, Model};

#[derive(Default)]
pub struct LegSeg {
    transform: Transform,
    length: f32,
}

pub struct LegAttach {
    offset: Vec3,
    base_yaw: f32,
}

#[derive(Default)]
pub struct Leg {
    coxa: LegSeg,
    femur: LegSeg,
    tibia: LegSeg,
    tarsus: LegSeg,

    coxa_base_yaw: f32,
    coxa_base_pitch: f32,
    knee_max_yaw: f32,

    rest_position: Vec3,
    current_target: Vec3,
    step_start: Vec3,
    step_end: Vec3,
    step_offset: f32,
    step_time: f32,
    step_cool: f32,
}

impl Leg {
    fn solve_leg_ik(&mut self, world_target: Vec3, inv_body: Mat4, delta_time: f32) {
        let local_target: Vec3 = (inv_body * Vec4::from((world_target, 1.0))).truncate();

        let hip_pos = Vec3::new(
            self.coxa.transform.x,
            self.coxa.transform.y,
            self.coxa.transform.z,
        );

        let l0 = self.coxa.length;
        let l1 = self.femur.length;
        let l2 = self.tibia.length;
        let l3 = self.tarsus.length;

        let mut positions = [Vec3::ZERO; 5];
        positions[0] = hip_pos;

        // Coxa
        let to_target_full = local_target - hip_pos;

        let target_yaw = -(to_target_full).x.atan2(-to_target_full.z);

        let mut yaw_offset = target_yaw - self.coxa_base_yaw;

        while yaw_offset > PI {
            yaw_offset -= 2.0 * PI;
        }
        while yaw_offset < -PI {
            yaw_offset += 2.0 * PI;
        }

        let max_yaw_offset = self.knee_max_yaw.to_radians();
        yaw_offset = yaw_offset.clamp(-max_yaw_offset, max_yaw_offset);

        let coxa_yaw = self.coxa_base_yaw + yaw_offset;
        let coxa_pitch = self.coxa_base_pitch;

        let coxa_dir = Vec3::new(
            -coxa_yaw.sin() * coxa_pitch.cos(),
            -coxa_pitch.sin(),
            -coxa_yaw.cos() * coxa_pitch.cos(),
        );

        positions[1] = hip_pos + coxa_dir * l0;

        // Femur
        let knee_base = positions[1];
        let to_target = local_target - knee_base;
        let mut dist_to_target = to_target.length();

        let l_eff = l2 + l3;
        let max_reach = l1 + l_eff;
        let min_reach = (l1 - l_eff).abs();
        dist_to_target = dist_to_target.clamp(min_reach + 0.01, max_reach - 0.01);

        let to_target_dir = if to_target.length() > 1e-4 {
            to_target.normalize()
        } else {
            Vec3::new(0.0, -1.0, 0.0)
        };

        let cos_knee = ((l1 * l1 + dist_to_target * dist_to_target - l_eff * l_eff)
            / (2.0 * l1 * dist_to_target))
            .clamp(-1.0, 1.0);
        let knee_angle = cos_knee.acos();

        let coxa_horizontal = Vec3::new(coxa_dir.x, 0.0, coxa_dir.z).normalize();
        let up = Vec3::Y;

        let bend_axis_raw = coxa_horizontal.cross(up);
        let bend_axis_len = bend_axis_raw.length();
        let bend_axis = if bend_axis_len < 1e-3 {
            if coxa_horizontal.x.abs() < coxa_horizontal.z.abs() {
                Vec3::X
            } else {
                Vec3::Z
            }
        } else {
            bend_axis_raw / bend_axis_len
        };

        let rot = Quat::from_axis_angle(bend_axis, knee_angle);
        let mut femur_dir = rot * to_target_dir;
        femur_dir = Self::clamp_dir_pitch(femur_dir, coxa_horizontal, 5.0_f32.to_radians(), 45.0_f32.to_radians());
        femur_dir = Self::clamp_dir_yaw(femur_dir, coxa_yaw, self.knee_max_yaw.to_radians());
        positions[2] = knee_base + femur_dir * l1;

        // Tibia
        let knee_to_target = local_target - positions[2];
        let knee_to_target_dist = knee_to_target.length();
        let knee_to_target_dir = if knee_to_target_dist > 1e-4 {
            knee_to_target / knee_to_target_dist
        } else {
            Vec3::new(0.0, -1.0, 0.0)
        };

        let knee_to_target_dir = Self::clamp_dir_yaw(knee_to_target_dir, coxa_yaw, self.knee_max_yaw.to_radians());

        positions[3] = positions[2] + knee_to_target_dir * l2;
        positions[4] = positions[3] + knee_to_target_dir * l3;

        Self::set_seg_angles(&mut self.coxa.transform, Mat3::IDENTITY, positions[0], positions[1], delta_time);

        let coxa_rot = Mat3::from_mat4(self.coxa.transform.to_matrix());
        Self::set_seg_angles(&mut self.femur.transform, coxa_rot, positions[1], positions[2], delta_time);

        let femur_rot = coxa_rot * Mat3::from_mat4(self.femur.transform.to_matrix());
        Self::set_seg_angles(&mut self.tibia.transform, femur_rot, positions[2], positions[3], delta_time);

        let tibia_rot = femur_rot * Mat3::from_mat4(self.tibia.transform.to_matrix());
        Self::set_seg_angles(&mut self.tarsus.transform, tibia_rot, positions[3], positions[4], delta_time);
    }

    fn clamp_dir_pitch(dir: Vec3, coxa_horizontal_fallback: Vec3, min_pit: f32, max_pit: f32) -> Vec3 {
        let d = dir.normalize();
        let horiz = (d.x * d.x + d.z * d.z).sqrt();
        let pitch = (-d.y).atan2(horiz).clamp(min_pit, max_pit);

        let h = if horiz > 1e-4 {
            Vec3::new(d.x, 0.0, d.z) / horiz
        } else {
            coxa_horizontal_fallback
        };

        Vec3::new(h.x * pitch.cos(), -pitch.sin(), h.z * pitch.cos())
    }

    fn clamp_dir_yaw(dir: Vec3, center_yaw: f32, max_offset: f32) -> Vec3 {
        let d = dir.normalize();
        let horiz = (d.x * d.x + d.z * d.z).sqrt();
        if horiz < 1e-4 {
            return d;
        }

        let yaw = (-d.x).atan2(-d.z); 

        let mut offset = yaw - center_yaw;
        while offset > PI {
            offset -= 2.0 * PI;
        }
        while offset < -PI {
            offset += 2.0 * PI;
        }
        let offset = offset.clamp(-max_offset, max_offset);

        let y = center_yaw + offset;
        Vec3::new(-y.sin() * horiz, d.y, -y.cos() * horiz)
    }

    fn set_seg_angles(transform: &mut Transform, parent_world_rot: Mat3, from_pos: Vec3, to_pos: Vec3, delta_time: f32) {
        const DEADZONE: f32 = 0.1;
        const SMOOTH_RATE: f32 = 15.0;

        let diff = to_pos - from_pos;
        if diff.length() < 1e-5 {
            return;
        }

        let world_dir = diff.normalize();
        let local_dir = parent_world_rot.inverse() * world_dir;

        let horiz = (local_dir.x * local_dir.x + local_dir.z * local_dir.z).sqrt();
        let smoothing = 1.0 - (-SMOOTH_RATE * delta_time).exp();

        if horiz > DEADZONE {
            let target_yaw = (-local_dir.x).atan2(-local_dir.z);

            let mut diff_angle = target_yaw - transform.yaw;
            while diff_angle > PI {
                diff_angle -= 2.0 * PI;
            }
            while diff_angle < -PI {
                diff_angle += 2.0 * PI;
            }

            transform.yaw += diff_angle * smoothing;
        }

        let target_pitch = horiz.atan2(-local_dir.y);
        transform.pit += (target_pitch - transform.pit) * smoothing;

        transform.rol = 0.0;
    }
}

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
            leg.coxa.length = 6.0 * scale;

            leg.femur.transform.x = 0.0;
            leg.femur.transform.y = -leg.coxa.length;
            leg.femur.transform.z = 0.0;
            leg.femur.transform.pit = 40.0_f32.to_radians();
            leg.femur.length = 6.0 * scale;

            leg.tibia.transform.y = -leg.femur.length;
            leg.tibia.transform.pit = (-70.0_f32).to_radians();
            leg.tibia.length = 8.0 * scale;

            leg.tarsus.transform.y = -leg.tibia.length;
            leg.tarsus.length = 1.5 * scale;

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
        const STEP_TRIGGER_DIST: f32 = 5.0;
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
            (self.transform.x > 120.0 && face_x > 0.0) ||
            (self.transform.x < -120.0 && face_x < 0.0) ||
            (self.transform.z > 80.0 && face_z > 0.0) ||
            (self.transform.z < -80.0 && face_z < 0.0);

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
            LegAttach { offset: Vec3::new(0.25, -0.5, 0.3), base_yaw: (-140.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, 0.0), base_yaw: (-110.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.2), base_yaw: (-80.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.75), base_yaw: (-45.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, 0.3), base_yaw: (140.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, 0.0), base_yaw: (110.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.2), base_yaw: (80.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.75), base_yaw: (45.0_f32).to_radians() },
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
            -0.1875, -0.2025, -1.25,  // front square
            0.1875, -0.2025, -1.25,
            -0.1875, -0.6975, -1.25,
            0.1875, -0.6975, -1.25,
            -0.1875, -0.2025,  0.25,  // back square
            0.1875, -0.2025,  0.25,
            -0.1875, -0.6975,  0.25,
            0.1875, -0.6975,  0.25,
            -0.375,  -0.2025, -0.875, // left square
            -0.375,  -0.2025, -0.125,
            -0.375,  -0.6975, -0.875,
            -0.375,  -0.6975, -0.125,
            0.375,  -0.2025, -0.875, // right square
            0.375,  -0.2025, -0.125,
            0.375,  -0.6975, -0.875,
            0.375,  -0.6975, -0.125,
            -0.1875,  0.045,  -0.875, // top square
            0.1875,  0.045,  -0.875,
            -0.1875,  0.045,  -0.125,
            0.1875,  0.045,  -0.125,
            -0.1875, -0.945,  -0.875, // bottom square
            0.1875, -0.945,  -0.875,
            -0.1875, -0.945,  -0.125,
            0.1875, -0.945,  -0.125,

            -0.25,  0.25, -0.3125,  // front square
            0.25,  0.25, -0.3125,
            -0.25, -0.25, -0.3125,
            0.25, -0.25, -0.3125,
            -0.125, 0.125, 2.1875,  // back square
            0.125, 0.125, 2.1875,
            -0.125,-0.125, 2.1875,
            0.125,-0.125, 2.1875,
            -0.5,   0.25,  0.3125,  // left square
            -0.4,   0.2,   1.5625,
            -0.5,  -0.25,  0.3125,
            -0.5,  -0.25,  1.5625,
            0.5,   0.25,  0.3125,  // right square
            0.4,   0.2,   1.5625,
            0.5,  -0.25,  0.3125,
            0.5,  -0.25,  1.5625,
            -0.25,  0.5,   0.3125,  // top square
            0.25,  0.5,   0.3125,
            -0.2,   0.4,   1.5625,
            0.2,   0.4,   1.5625,
            -0.25, -0.5,   0.3125,  // bottom square
            0.25, -0.5,   0.3125,
            -0.25, -0.5,   1.5625,
            0.25, -0.5,   1.5625,
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
            -0.025, -0.25, -0.05, // front square
            0.025, -0.25, -0.05,
            -0.025, -0.75, -0.05,
            0.025, -0.75, -0.05,
            -0.025, -0.25, 0.05, // back square
            0.025, -0.25, 0.05,
            -0.025, -0.75, 0.05,
            0.025, -0.75, 0.05,
            -0.05, -0.25, -0.025, // left square
            -0.05, -0.25, 0.025,
            -0.05, -0.75, -0.025,
            -0.05, -0.75, 0.025,
            0.05, -0.25, -0.025, // right square
            0.05, -0.25, 0.025,
            0.05, -0.75, -0.025,
            0.05, -0.75, 0.025,
            -0.025, 0.0, -0.025, // top square
            0.025, 0.0, -0.025,
            -0.025, 0.0, 0.025,
            0.025, 0.0, 0.025,
            -0.025, -1.0, -0.025, // bottom square
            0.025, -1.0, -0.025,
            -0.025, -1.0, 0.025,
            0.025, -1.0, 0.025
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
            leg.coxa.length = 6.0 * scale;

            leg.femur.transform.x = 0.0;
            leg.femur.transform.y = -leg.coxa.length;
            leg.femur.transform.z = 0.0;
            leg.femur.transform.pit = 40.0_f32.to_radians();
            leg.femur.length = 6.0 * scale;

            leg.tibia.transform.y = -leg.femur.length;
            leg.tibia.transform.pit = (-70.0_f32).to_radians();
            leg.tibia.length = 8.0 * scale;

            leg.tarsus.transform.y = -leg.tibia.length;
            leg.tarsus.length = 1.5 * scale;

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
        const STEP_TRIGGER_DIST: f32 = 10.0;
        const STEP_HEIGHT: f32 = 5.0;

        let stride = STEP_TRIGGER_DIST + 0.1 * self.motive.speed;
        let step_speed = (self.motive.speed / (2.0 * stride)).max(100.0);
        let group_stagger = 0.1 / self.motive.speed;

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
                self.motive.speed = rng.random_range(5.0..100.0); 
                self.motive.yaw_time = 0.1;
                self.motive.delta_yaw = rng.random_range(-5.0..5.0);
            }
        }

        let face_x = -self.transform.yaw.sin() * self.transform.pit.cos();
        let face_z = -self.transform.yaw.cos() * self.transform.pit.cos();

        let facing_out = 
            (self.transform.x > 120.0 && face_x > 0.0) ||
            (self.transform.x < -120.0 && face_x < 0.0) ||
            (self.transform.z > 80.0 && face_z > 0.0) ||
            (self.transform.z < -80.0 && face_z < 0.0);

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
            LegAttach { offset: Vec3::new(0.25, -0.5, 0.3), base_yaw: (-150.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, 0.0), base_yaw: (-120.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.2), base_yaw: (-60.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.75), base_yaw: (-30.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, 0.3), base_yaw: (150.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, 0.0), base_yaw: (120.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.2), base_yaw: (60.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.75), base_yaw: (30.0_f32).to_radians() },
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
            -0.375, -0.405, -2.5,  // front square
            0.375, -0.405, -2.5,
            -0.375, -1.395, -2.5,
            0.375, -1.395, -2.5,
            -0.375, -0.405,  0.5,  // back square
            0.375, -0.405,  0.5,
            -0.375, -1.395,  0.5,
            0.375, -1.395,  0.5,
            -0.75,  -0.405, -1.75, // left square
            -0.75,  -0.405, -0.25,
            -0.75,  -1.395, -1.75,
            -0.75,  -1.395, -0.25,
            0.75,  -0.405, -1.75, // right square
            0.75,  -0.405, -0.25,
            0.75,  -1.395, -1.75,
            0.75,  -1.395, -0.25,
            -0.375,  0.09,  -1.75, // top square
            0.375,  0.09,  -1.75,
            -0.375,  0.09,  -0.25,
            0.375,  0.09,  -0.25,
            -0.375, -1.89,  -1.75, // bottom square
            0.375, -1.89,  -1.75,
            -0.375, -1.89,  -0.25,
            0.375, -1.89,  -0.25,

            -0.5,   0.5,   -0.625,  // front square
            0.5,   0.5,   -0.625,
            -0.5,  -0.5,   -0.625,
            0.5,  -0.5,   -0.625,
            -0.25,  0.25,   4.375,  // back square
            0.25,  0.25,   4.375,
            -0.25, -0.25,   4.375,
            0.25, -0.25,   4.375,
            -1.0,   0.5,    0.625,  // left square
            -0.8,   0.4,    3.125,
            -1.0,  -0.5,    0.625,
            -1.0,  -0.5,    3.125,
            1.0,   0.5,    0.625,  // right square
            0.8,   0.4,    3.125,
            1.0,  -0.5,    0.625,
            1.0,  -0.5,    3.125,
            -0.5,   1.0,    0.625,  // top square
            0.5,   1.0,    0.625,
            -0.4,   0.8,    3.125,
            0.4,   0.8,    3.125,
            -0.5,  -1.0,    0.625,  // bottom square
            0.5,  -1.0,    0.625,
            -0.5,  -1.0,    3.125,
            0.5,  -1.0,    3.125,
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
            leg.coxa.length = 3.0 * scale;

            leg.femur.transform.x = 0.0;
            leg.femur.transform.y = -leg.coxa.length;
            leg.femur.transform.z = 0.0;
            leg.femur.transform.pit = 40.0_f32.to_radians();
            leg.femur.length = 3.0 * scale;

            leg.tibia.transform.y = -leg.femur.length;
            leg.tibia.transform.pit = (-70.0_f32).to_radians();
            leg.tibia.length = 4.0 * scale;

            leg.tarsus.transform.y = -leg.tibia.length;
            leg.tarsus.length = 0.75 * scale;

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
        const STEP_HEIGHT: f32 = 20.0;

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
                self.motive.speed = rng.random_range(5.0..50.0);
                self.motive.yaw_time = 0.5;
                self.motive.delta_yaw = rng.random_range(-5.0..5.0);
            }
        }

        let face_x = -self.transform.yaw.sin() * self.transform.pit.cos();
        let face_z = -self.transform.yaw.cos() * self.transform.pit.cos();

        let facing_out = 
            (self.transform.x > 120.0 && face_x > 0.0) ||
            (self.transform.x < -120.0 && face_x < 0.0) ||
            (self.transform.z > 80.0 && face_z > 0.0) ||
            (self.transform.z < -80.0 && face_z < 0.0);

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
            LegAttach { offset: Vec3::new(0.25, -0.5,  2.25), base_yaw: (-175.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5,  1.8393), base_yaw: (-165.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5,  1.4286), base_yaw: (-150.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5,  1.0179), base_yaw: (-140.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5,  0.6071), base_yaw: (-130.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5,  0.1964), base_yaw: (-120.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.2143), base_yaw: (-115.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.6250), base_yaw: (-110.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -1.0357), base_yaw: (-105.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -1.4464), base_yaw: (-100.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -1.8571), base_yaw: (-95.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -2.2679), base_yaw: (-90.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -2.6786), base_yaw: (-80.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -3.0893), base_yaw: (-60.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -3.5), base_yaw: (-50.0_f32).to_radians() },

            LegAttach { offset: Vec3::new(-0.25, -0.5,  2.25), base_yaw: (175.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5,  1.8393), base_yaw: (165.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5,  1.4286), base_yaw: (150.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5,  1.0179), base_yaw: (140.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5,  0.6071), base_yaw: (130.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5,  0.1964), base_yaw: (120.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.2143), base_yaw: (115.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.6250), base_yaw: (110.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -1.0357), base_yaw: (105.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -1.4464), base_yaw: (100.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -1.8571), base_yaw: (95.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -2.2679), base_yaw: (90.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -2.6786), base_yaw: (80.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -3.0893), base_yaw: (60.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -3.5), base_yaw: (50.0_f32).to_radians() },
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
            -0.375, -0.405, -4.0,  // front square
            0.375, -0.405, -4.0,
            -0.375, -1.395, -4.0,
            0.375, -1.395, -4.0,
            -0.375, -0.405,  5.0,  // back square
            0.375, -0.405,  5.0,
            -0.375, -1.395,  5.0,
            0.375, -1.395,  5.0,
            -0.75,  -0.405, -1.75, // left square
            -0.75,  -0.405,  2.75,
            -0.75,  -1.395, -1.75,
            -0.75,  -1.395,  2.75,
            0.75,  -0.405, -1.75, // right square
            0.75,  -0.405,  2.75,
            0.75,  -1.395, -1.75,
            0.75,  -1.395,  2.75,
            -0.375,  0.09,  -1.75, // top square
            0.375,  0.09,  -1.75,
            -0.375,  0.09,   2.75,
            0.375,  0.09,   2.75,
            -0.375, -1.89,  -1.75, // bottom square
            0.375, -1.89,  -1.75,
            -0.375, -1.89,   2.75,
            0.375, -1.89,   2.75,
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
            -0.09375, -0.1875, -0.05, // front square
            0.09375, -0.1875, -0.05,
            -0.09375, -0.5625, -0.05,
            0.09375, -0.5625, -0.05,
            -0.09375, -0.1875,  0.05, // back square
            0.09375, -0.1875,  0.05,
            -0.09375, -0.5625,  0.05,
            0.09375, -0.5625,  0.05,
            -0.1875,  -0.1875, -0.025, // left square
            -0.1875,  -0.1875,  0.025,
            -0.1875,  -0.5625, -0.025,
            -0.1875,  -0.5625,  0.025,
            0.1875,  -0.1875, -0.025, // right square
            0.1875,  -0.1875,  0.025,
            0.1875,  -0.5625, -0.025,
            0.1875,  -0.5625,  0.025,
            -0.09375,  0.0,  -0.025, // top square
            0.09375,  0.0,  -0.025,
            -0.09375,  0.0,   0.025,
            0.09375,  0.0,   0.025,
            -0.09375, -0.75,  -0.025, // bottom square
            0.09375, -0.75,  -0.025,
            -0.09375, -0.75,   0.025,
            0.09375, -0.75,   0.025
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
            leg.coxa.length = 6.0 * scale;

            leg.femur.transform.x = 0.0;
            leg.femur.transform.y = -leg.coxa.length;
            leg.femur.transform.z = 0.0;
            leg.femur.transform.pit = 40.0_f32.to_radians();
            leg.femur.length = 6.0 * scale;

            leg.tibia.transform.y = -leg.femur.length;
            leg.tibia.transform.pit = (-70.0_f32).to_radians();
            leg.tibia.length = 8.0 * scale;

            leg.tarsus.transform.y = -leg.tibia.length;
            leg.tarsus.length = 1.5 * scale;

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

            leg.coxa_base_pitch = (-30.0_f32).to_radians();

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
        const STEP_TRIGGER_DIST: f32 = 5.0;
        const STEP_HEIGHT: f32 = 20.0;

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
            (self.transform.x > 120.0 && face_x > 0.0) ||
            (self.transform.x < -120.0 && face_x < 0.0) ||
            (self.transform.z > 80.0 && face_z > 0.0) ||
            (self.transform.z < -80.0 && face_z < 0.0);

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
            LegAttach { offset: Vec3::new(0.25, -0.5, 0.3), base_yaw: (-140.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, 0.0), base_yaw: (-110.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.2), base_yaw: (-80.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(0.25, -0.5, -0.75), base_yaw: (-45.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, 0.3), base_yaw: (140.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, 0.0), base_yaw: (110.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.2), base_yaw: (80.0_f32).to_radians() },
            LegAttach { offset: Vec3::new(-0.25, -0.5, -0.75), base_yaw: (45.0_f32).to_radians() },
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
            -0.375, -0.405, -2.5,  // front square
            0.375, -0.405, -2.5,
            -0.375, -1.395, -2.5,
            0.375, -1.395, -2.5,
            -0.375, -0.405,  0.5,  // back square
            0.375, -0.405,  0.5,
            -0.375, -1.395,  0.5,
            0.375, -1.395,  0.5,
            -0.75,  -0.405, -1.75, // left square
            -0.75,  -0.405, -0.25,
            -0.75,  -1.395, -1.75,
            -0.75,  -1.395, -0.25,
            0.75,  -0.405, -1.75, // right square
            0.75,  -0.405, -0.25,
            0.75,  -1.395, -1.75,
            0.75,  -1.395, -0.25,
            -0.375,  0.09,  -1.75, // top square
            0.375,  0.09,  -1.75,
            -0.375,  0.09,  -0.25,
            0.375,  0.09,  -0.25,
            -0.375, -1.89,  -1.75, // bottom square
            0.375, -1.89,  -1.75,
            -0.375, -1.89,  -0.25,
            0.375, -1.89,  -0.25,
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
            -0.025, -0.25, -0.05, // front square
            0.025, -0.25, -0.05,
            -0.025, -0.75, -0.05,
            0.025, -0.75, -0.05,
            -0.025, -0.25, 0.05, // back square
            0.025, -0.25, 0.05,
            -0.025, -0.75, 0.05,
            0.025, -0.75, 0.05,
            -0.05, -0.25, -0.025, // left square
            -0.05, -0.25, 0.025,
            -0.05, -0.75, -0.025,
            -0.05, -0.75, 0.025,
            0.05, -0.25, -0.025, // right square
            0.05, -0.25, 0.025,
            0.05, -0.75, -0.025,
            0.05, -0.75, 0.025,
            -0.025, 0.0, -0.025, // top square
            0.025, 0.0, -0.025,
            -0.025, 0.0, 0.025,
            0.025, 0.0, 0.025,
            -0.025, -1.0, -0.025, // bottom square
            0.025, -1.0, -0.025,
            -0.025, -1.0, 0.025,
            0.025, -1.0, 0.025
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

