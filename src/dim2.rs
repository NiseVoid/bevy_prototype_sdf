use crate::{Dim, Sdf, SdfBounding, SdfTree};

use bevy::math::{bounding::*, primitives::*, Isometry2d, Rot2, Vec2};
use bevy::reflect::Reflect;

#[cfg(feature = "serialize")]
use bevy::reflect::ReflectDeserialize;

#[derive(Reflect, Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Deserialize))]
pub struct Dim2;

impl Dim for Dim2 {
    const POS_SIZE: usize = 2;
    const ROT_SIZE: usize = 1;
    type Position = Vec2;
    type Rotation = Rot2;
    type Isometry = Isometry2d;

    type Aabb = Aabb2d;
    type Ball = BoundingCircle;
}

impl<P: Sdf<Dim2> + Bounded2d> SdfBounding<Dim2> for P {
    fn aabb(&self, iso: Isometry2d) -> Aabb2d {
        self.aabb_2d(iso)
    }

    fn bounding_ball(&self, iso: Isometry2d) -> BoundingCircle {
        self.bounding_circle(iso)
    }
}

pub type Sdf2d = SdfTree<Dim2>;

impl Sdf<Dim2> for Circle {
    fn distance(&self, pos: Vec2) -> f32 {
        pos.length() - self.radius
    }

    fn gradient(&self, pos: Vec2) -> Vec2 {
        if pos == Vec2::ZERO {
            Vec2::ZERO
        } else {
            pos.normalize()
        }
    }
}

impl Sdf<Dim2> for Rectangle {
    fn distance(&self, pos: Vec2) -> f32 {
        let q = pos.abs() - self.half_size;
        let l = q.max(Vec2::ZERO).length();
        l + q.max_element().min(0.)
    }

    fn gradient(&self, pos: Vec2) -> Vec2 {
        if pos == Vec2::ZERO {
            return Vec2::ZERO;
        }

        let abs_pos = pos.abs();
        let q = abs_pos - self.half_size;
        let q_or_zero = q.max(Vec2::ZERO);
        let l = q_or_zero.length_squared();
        if l > 0. {
            // If we are outside the box, we can normalize q_or_zero and match it to the
            // pos sign (so we get the direction relative to the octant we are on)
            q_or_zero / l.sqrt() * pos.signum()
        } else {
            // If we are on the inside, the gradient points to a normalized vector of the
            // closests sides
            let distance = -q;
            let min = distance.min_element();
            Vec2::select(distance.cmpeq(Vec2::splat(min)), pos.signum(), Vec2::ZERO).normalize()
        }
    }
}

#[derive(Reflect, Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Deserialize))]
pub struct Arc {
    pub radius: f32,
    pub thickness: f32,
    pub segment: f32,
}

impl Arc {
    pub fn new(radius: f32, thickness: f32, angle: f32) -> Self {
        Self {
            radius,
            thickness,
            segment: (angle / 2.).clamp(0., core::f32::consts::PI),
        }
    }
}

impl Sdf<Dim2> for Arc {
    fn distance(&self, mut pos: Vec2) -> f32 {
        let q = pos;
        let sc = Vec2::new(self.segment.sin(), self.segment.cos());
        pos.x = pos.x.abs();
        if sc.y * pos.x > sc.x * pos.y {
            let w = pos - self.radius * sc;
            let l = w.length();
            l - self.thickness
        } else {
            let l = q.length();
            let w = l - self.radius;
            w.abs() - self.thickness
        }
    }

    fn gradient(&self, mut pos: Vec2) -> Vec2 {
        let q = pos;
        let sc = Vec2::new(self.segment.sin(), self.segment.cos());
        let s = pos.x.signum();
        pos.x = pos.x.abs();
        if sc.y * pos.x > sc.x * pos.y {
            let w = pos - self.radius * sc;
            let l = w.length();
            Vec2::new(s * w.x, w.y) / l
        } else {
            let l = q.length();
            let w = l - self.radius;
            w.signum() * q / l
        }
    }
}

impl Bounded2d for Arc {
    fn aabb_2d(&self, iso: impl Into<Isometry2d>) -> Aabb2d {
        let iso = iso.into();
        // TODO
        let r = self.radius + self.thickness;
        let half_width = if self.segment < core::f32::consts::PI / 2. {
            self.radius * self.segment.sin() + self.thickness
        } else {
            r
        };
        let bottom = self.segment.cos() * self.radius - self.thickness;
        Aabb2d {
            min: Vec2::new(-half_width, bottom) + iso.translation,
            max: Vec2::new(half_width, r) + iso.translation,
        }
    }

    fn bounding_circle(&self, iso: impl Into<Isometry2d>) -> BoundingCircle {
        let iso = iso.into();
        // TODO: This isn't an optimal bounding circle
        BoundingCircle::new(iso.translation, self.radius + self.thickness)
    }
}
