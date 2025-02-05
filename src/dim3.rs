use crate::{Dim, Sdf, SdfBounding, SdfTree};

use bevy::math::{bounding::*, primitives::*, Isometry3d, Quat, Vec3};
use bevy::reflect::Reflect;

#[cfg(feature = "serialize")]
use bevy::reflect::ReflectDeserialize;

#[derive(Reflect, Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Deserialize))]
pub struct Dim3;

impl Dim for Dim3 {
    const POS_SIZE: usize = 2;
    const ROT_SIZE: usize = 4;
    type Position = Vec3;
    type Rotation = Quat;
    type Isometry = Isometry3d;

    type Aabb = Aabb3d;
    type Ball = BoundingSphere;
}

impl<P: Sdf<Dim3> + Bounded3d> SdfBounding<Dim3> for P {
    fn aabb(&self, iso: Isometry3d) -> Aabb3d {
        self.aabb_3d(iso)
    }

    fn bounding_ball(&self, iso: Isometry3d) -> BoundingSphere {
        self.bounding_sphere(iso)
    }
}

pub type Sdf3d = SdfTree<Dim3>;

impl Sdf<Dim3> for Sphere {
    fn distance(&self, pos: Vec3) -> f32 {
        pos.length() - self.radius
    }

    fn gradient(&self, pos: Vec3) -> Vec3 {
        if pos == Vec3::ZERO {
            Vec3::ZERO
        } else {
            pos.normalize()
        }
    }
}

impl Sdf<Dim3> for Capsule3d {
    fn distance(&self, pos: Vec3) -> f32 {
        let mut pa = pos;
        pa.y += self.half_length;
        let length = self.half_length * 2.;
        let ba = Vec3::new(0., length, 0.);
        let h = (pa.dot(ba) / length.powi(2)).clamp(0., 1.);
        let q = pa - ba * h;
        q.length() - self.radius
    }

    fn gradient(&self, pos: Vec3) -> Vec3 {
        let mut pa = pos;
        pa.y += self.half_length;
        let length = self.half_length * 2.;
        let ba = Vec3::new(0., length, 0.);
        let h = (pa.dot(ba) / length.powi(2)).clamp(0., 1.);
        let q = pa - ba * h;
        if q == Vec3::ZERO {
            Vec3::ZERO
        } else {
            q.normalize()
        }
    }
}

impl Sdf<Dim3> for Cuboid {
    fn distance(&self, pos: Vec3) -> f32 {
        let q = pos.abs() - self.half_size;
        let l = q.max(Vec3::ZERO).length();
        l + q.max_element().min(0.)
    }

    fn gradient(&self, pos: Vec3) -> Vec3 {
        if pos == Vec3::ZERO {
            return Vec3::ZERO;
        }

        let abs_pos = pos.abs();
        let q = abs_pos - self.half_size;
        let q_or_zero = q.max(Vec3::ZERO);
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
            Vec3::select(distance.cmpeq(Vec3::splat(min)), pos.signum(), Vec3::ZERO).normalize()
        }
    }
}

impl Sdf<Dim3> for Cylinder {
    fn distance(&self, pos: Vec3) -> f32 {
        use bevy::math::{Vec2, Vec3Swizzles};
        let p2d = pos.xz();
        let l = p2d.length();
        let w = Vec2::new(l, pos.y).abs() - Vec2::new(self.radius, self.half_height);
        w.x.max(w.y).min(0.0) + w.max(Vec2::ZERO).length()
    }

    fn gradient(&self, pos: Vec3) -> Vec3 {
        use bevy::math::{Vec2, Vec3Swizzles};
        let p2d = pos.xz();
        let l = p2d.length();
        let w = Vec2::new(l, pos.y).abs() - Vec2::new(self.radius, self.half_height);

        let grad2d = p2d / l;
        if w.y <= 0. {
            if w.x <= 0. && w.y > w.x {
                Vec3::Y
            } else {
                Vec3::new(grad2d.x, 0., grad2d.y)
            }
        } else {
            let d_or_zero = w.x.max(0.);
            Vec3::new(
                d_or_zero * grad2d.x,
                w.y.max(0.) * pos.y.signum(),
                d_or_zero * grad2d.y,
            )
            .normalize()
        }
    }
}

impl Sdf<Dim3> for InfinitePlane3d {
    fn distance(&self, pos: Vec3) -> f32 {
        pos.dot(self.normal.into())
    }

    fn gradient(&self, _: Vec3) -> Vec3 {
        self.normal.into()
    }
}
