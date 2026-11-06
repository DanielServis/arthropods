use glam::{Vec3, Vec4, Quat, Mat3, Mat4};
use std::f32::consts::PI;

use crate::things::{Transform};

#[derive(Default)]
pub struct LegSeg {
    pub transform: Transform,
    pub length: f32,
}

pub struct LegAttach {
    pub offset: Vec3,
    pub base_yaw: f32,
}

#[derive(Default)]
pub struct Leg {
    pub coxa: LegSeg,
    pub femur: LegSeg,
    pub tibia: LegSeg,
    pub tarsus: LegSeg,

    pub coxa_base_yaw: f32,
    pub coxa_base_pitch: f32,
    pub knee_max_yaw: f32,

    pub rest_position: Vec3,
    pub current_target: Vec3,
    pub step_start: Vec3,
    pub step_end: Vec3,
    pub step_offset: f32,
    pub step_time: f32,
    pub step_cool: f32,
}

impl Leg {
    pub fn solve_leg_ik(&mut self, world_target: Vec3, inv_body: Mat4, delta_time: f32) {
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

pub struct WingAttach {
    pub offset: Vec3,
    pub base_yaw: f32,
}

#[derive(Default)]
pub struct Wing {
    pub transform: Transform,

    pub rest_position: Vec3,
}

impl Wing {
    pub fn flap(&mut self, inv_body: Mat4, lr: f32, delta_time: f32) {
        if lr % 2.0 == 0.0 {
            self.transform.pit = 0.0;
            self.transform.yaw = 0.0;
            self.transform.rol = 90.0;
        }
        else {
            self.transform.pit = 0.0;
            self.transform.yaw = 0.0;
            self.transform.rol = -90.0;
        }
    }
}
