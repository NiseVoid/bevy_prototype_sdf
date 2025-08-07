use crate::{Dim, Sdf, SdfBounding, SdfTree};

use bevy::math::{Isometry2d, Rot2, Vec2, bounding::*, primitives::*};
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
        pos.normalize_or_zero()
    }

    fn dist_grad(&self, pos: Vec2) -> (f32, Vec2) {
        if pos == Vec2::ZERO {
            (-self.radius, Vec2::ZERO)
        } else {
            let l = pos.length();
            (l - self.radius, pos / l)
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

        let q = pos.abs() - self.half_size;
        if q.cmpgt(Vec2::ZERO).any() {
            // If we are outside the box, we can normalize q_or_zero and match it to the
            // pos sign (so we get the direction relative to the octant we are on)
            q.max(Vec2::ZERO).normalize().copysign(pos)
        } else {
            // If we are on the inside, the gradient points to a normalized vector of the
            // closests sides
            let distance = -q;
            let min = distance.min_element();
            Vec2::select(distance.cmpeq(Vec2::splat(min)), pos.signum(), Vec2::ZERO).normalize()
        }
    }

    fn dist_grad(&self, pos: Vec2) -> (f32, Vec2) {
        if pos == Vec2::ZERO {
            return (-self.half_size.min_element(), Vec2::ZERO);
        }

        let q = pos.abs() - self.half_size;
        if q.cmpgt(Vec2::ZERO).any() {
            let q_or_zero = q.max(Vec2::ZERO);
            let l = q_or_zero.length();
            (l + q.max_element().min(0.), (q_or_zero / l).copysign(pos))
        } else {
            let distances = -q;
            let dist = distances.min_element();
            (
                dist,
                Vec2::select(distances.cmpeq(Vec2::splat(dist)), pos.signum(), Vec2::ZERO)
                    .normalize(),
            )
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

impl Arc {
    fn base<O>(
        &self,
        mut pos: Vec2,
        a: impl FnOnce(Vec2, f32) -> O,
        b: impl FnOnce(f32, f32) -> O,
    ) -> O {
        let q = pos;
        let sc = Vec2::new(self.segment.sin(), self.segment.cos());
        pos.x = pos.x.abs();
        if sc.y * pos.x > sc.x * pos.y {
            let w = pos - self.radius * sc;
            let l = w.length();
            a(w, l)
        } else {
            let l = q.length();
            let w = l - self.radius;
            b(w, l)
        }
    }
}

impl Sdf<Dim2> for Arc {
    fn distance(&self, pos: Vec2) -> f32 {
        self.base(
            pos,
            |_, l| l - self.thickness,
            |w, _| w.abs() - self.thickness,
        )
    }

    fn gradient(&self, pos: Vec2) -> Vec2 {
        self.base(
            pos,
            |w, l| Vec2::new(w.x.copysign(pos.x), w.y) / l,
            |w, l| w.signum() * pos / l,
        )
    }

    fn dist_grad(&self, pos: Vec2) -> (f32, Vec2) {
        self.base(
            pos,
            |w, l| (l - self.thickness, Vec2::new(w.x.copysign(pos.x), w.y) / l),
            |w, l| (w.abs() - self.thickness, w.signum() * pos / l),
        )
    }
}

impl Bounded2d for Arc {
    fn aabb_2d(&self, iso: impl Into<Isometry2d>) -> Aabb2d {
        Arc2d::new(self.radius, self.segment)
            .aabb_2d(iso)
            .grow(Vec2::splat(self.thickness))
    }

    fn bounding_circle(&self, iso: impl Into<Isometry2d>) -> BoundingCircle {
        Arc2d::new(self.radius, self.segment)
            .bounding_circle(iso)
            .grow(self.thickness)
    }
}
