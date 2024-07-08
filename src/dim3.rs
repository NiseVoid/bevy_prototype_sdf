use crate::{Dim, Sdf, Sdf2d, SdfBounding, SdfTree};

use bevy_math::{bounding::*, primitives::*, Quat, Vec3};

#[cfg(all(feature = "bevy_reflect", feature = "serialize"))]
use bevy_reflect::ReflectDeserialize;

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "bevy_reflect", derive(bevy_reflect::Reflect))]
#[cfg_attr(
    all(feature = "bevy_reflect", feature = "serialize"),
    reflect(Deserialize)
)]
pub struct Dim3;

impl Dim for Dim3 {
    const POS_SIZE: usize = 2;
    const ROT_SIZE: usize = 4;
    type Position = bevy_math::Vec3;
    type Rotation = bevy_math::Quat;

    type Aabb = Aabb3d;
    type Ball = BoundingSphere;
}

impl<P: Sdf<Dim3> + Bounded3d> SdfBounding<Dim3> for P {
    fn aabb(&self, translation: Vec3, rotation: impl Into<Quat>) -> Aabb3d {
        self.aabb_3d(translation, rotation.into())
    }

    fn bounding_ball(&self, translation: Vec3, rotation: impl Into<Quat>) -> BoundingSphere {
        self.bounding_sphere(translation, rotation.into())
    }
}

pub type Sdf3d = SdfTree<Dim3, Sdf3dShape>;

impl<IntoShape: Into<Sdf3dShape>> From<IntoShape> for Sdf3d {
    fn from(value: IntoShape) -> Self {
        let shape = value.into();
        Self {
            operations: Vec::new(),
            shapes: vec![shape],
        }
    }
}

/// An enum dispatch version of Sdf<Vec3> with support for extruded 2d sdfs
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "bevy_reflect", derive(bevy_reflect::Reflect))]
#[cfg_attr(
    all(feature = "bevy_reflect", feature = "serialize"),
    reflect(Deserialize)
)]
pub enum Sdf3dShape {
    Sphere(Sphere),
    Capsule(Capsule3d),
    Cylinder(Cylinder),
    Cuboid(Cuboid),
    InfinitePlane(InfinitePlane3d),
    Extruded(Extruded<Sdf2d>),
}

impl From<Sphere> for Sdf3dShape {
    fn from(value: Sphere) -> Self {
        Sdf3dShape::Sphere(value)
    }
}

impl From<Capsule3d> for Sdf3dShape {
    fn from(value: Capsule3d) -> Self {
        Sdf3dShape::Capsule(value)
    }
}

impl From<Cylinder> for Sdf3dShape {
    fn from(value: Cylinder) -> Self {
        Sdf3dShape::Cylinder(value)
    }
}

impl From<Cuboid> for Sdf3dShape {
    fn from(value: Cuboid) -> Self {
        Sdf3dShape::Cuboid(value)
    }
}

impl From<InfinitePlane3d> for Sdf3dShape {
    fn from(value: InfinitePlane3d) -> Self {
        Sdf3dShape::InfinitePlane(value)
    }
}

impl From<Extruded<Sdf2d>> for Sdf3dShape {
    fn from(value: Extruded<Sdf2d>) -> Self {
        Sdf3dShape::Extruded(value)
    }
}

impl SdfBounding<Dim3> for Sdf3dShape {
    fn aabb(&self, translation: Vec3, rotation: impl Into<Quat>) -> Aabb3d {
        let rotation = rotation.into();
        use Sdf3dShape::*;
        match self {
            Sphere(s) => s.aabb(translation, rotation),
            Capsule(c) => c.aabb(translation, rotation),
            Cylinder(c) => c.aabb(translation, rotation),
            Cuboid(b) => b.aabb(translation, rotation),
            InfinitePlane(p) => p.aabb(translation, rotation),
            Extruded(e) => e.aabb(translation, rotation),
        }
    }

    fn bounding_ball(&self, translation: Vec3, rotation: impl Into<Quat>) -> BoundingSphere {
        let rotation = rotation.into();
        use Sdf3dShape::*;
        match self {
            Sphere(s) => s.bounding_ball(translation, rotation),
            Capsule(c) => c.bounding_ball(translation, rotation),
            Cylinder(c) => c.bounding_ball(translation, rotation),
            Cuboid(b) => b.bounding_ball(translation, rotation),
            InfinitePlane(b) => b.bounding_ball(translation, rotation),
            Extruded(e) => e.bounding_ball(translation, rotation),
        }
    }
}

impl Sdf<Dim3> for Sdf3dShape {
    fn distance(&self, pos: Vec3) -> f32 {
        use Sdf3dShape::*;
        match self {
            Sphere(s) => s.distance(pos),
            Capsule(c) => c.distance(pos),
            Cylinder(c) => c.distance(pos),
            Cuboid(b) => b.distance(pos),
            InfinitePlane(p) => p.distance(pos),
            Extruded(e) => e.distance(pos),
        }
    }

    fn gradient(&self, pos: Vec3) -> Vec3 {
        use Sdf3dShape::*;
        match self {
            Sphere(s) => s.gradient(pos),
            Capsule(c) => c.gradient(pos),
            Cylinder(c) => c.gradient(pos),
            Cuboid(b) => b.gradient(pos),
            InfinitePlane(p) => p.gradient(pos),
            Extruded(e) => e.gradient(pos),
        }
    }
}

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
        use bevy_math::{Vec2, Vec3Swizzles};
        let p2d = pos.xz();
        let l = p2d.length();
        let w = Vec2::new(l, pos.y).abs() - Vec2::new(self.radius, self.half_height);
        w.x.max(w.y).min(0.0) + w.max(Vec2::ZERO).length()
    }

    fn gradient(&self, pos: Vec3) -> Vec3 {
        use bevy_math::{Vec2, Vec3Swizzles};
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

/// A 2d sdf, extruded into 3D space
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serialize",
    serde(bound(deserialize = "Sdf2d: for<'de2> serde::Deserialize<'de2>"))
)]
#[cfg_attr(feature = "bevy_reflect", derive(bevy_reflect::Reflect))]
#[cfg_attr(
    all(feature = "bevy_reflect", feature = "serialize"),
    reflect(Deserialize)
)]
pub struct Extruded<Sdf2d: Sdf<super::dim2::Dim2>> {
    /// The 2D sdf used
    pub sdf: Sdf2d,
    /// The half height for the extruded shape
    pub half_height: f32,
}

impl super::dim2::Sdf2d {
    pub fn extruded(self, height: f32) -> Sdf3d {
        Sdf3d::from(Extruded {
            sdf: self,
            half_height: height / 2.,
        })
    }
}

impl<Sdf2d: Sdf<super::dim2::Dim2>> SdfBounding<Dim3> for Extruded<Sdf2d> {
    fn aabb(&self, translation: Vec3, rotation: impl Into<Quat>) -> Aabb3d {
        let rotation = rotation.into();
        use bevy_math::{Mat3, Vec2};
        let rect = self.sdf.aabb(Vec2::ZERO, 0.);
        let rect_size = rect.half_size();

        let rot_mat = Mat3::from_quat(rotation);
        let abs_rot_mat = Mat3::from_cols(
            rot_mat.x_axis.abs(),
            rot_mat.y_axis.abs(),
            rot_mat.z_axis.abs(),
        );
        let half_size = abs_rot_mat * Vec3::new(rect_size.x, self.half_height, rect_size.y);

        let mut offset = translation;
        let rect_center = rect.center();
        if rect_center != Vec2::ZERO {
            offset += rotation * Vec3::new(rect_center.x, 0., rect_center.y);
        }
        Aabb3d::new(offset, half_size)
    }

    fn bounding_ball(&self, translation: Vec3, rotation: impl Into<Quat>) -> BoundingSphere {
        let rotation = rotation.into();
        use bevy_math::Vec2;
        let circle = self.sdf.bounding_ball(Vec2::ZERO, 0.);

        let radius = circle.radius().hypot(self.half_height);

        let mut offset = translation;
        let rect_center = circle.center();
        if rect_center != Vec2::ZERO {
            offset += rotation * Vec3::new(rect_center.x, 0., rect_center.y);
        }
        BoundingSphere::new(translation, radius)
    }
}

impl<Sdf2d: Sdf<super::dim2::Dim2>> Sdf<Dim3> for Extruded<Sdf2d> {
    fn distance(&self, pos: Vec3) -> f32 {
        use bevy_math::Vec2;
        let d = self.sdf.distance(Vec2::new(pos.x, pos.z));

        let w = Vec2::new(d, pos.y.abs() - self.half_height);
        w.x.max(w.y).min(0.0) + w.max(Vec2::ZERO).length()
    }

    fn gradient(&self, pos: Vec3) -> Vec3 {
        use bevy_math::Vec2;
        let d = self.sdf.distance(Vec2::new(pos.x, pos.z));
        let grad = self.sdf.gradient(Vec2::new(pos.x, pos.z));

        let w = Vec2::new(d, pos.y.abs() - self.half_height);

        if w.y <= 0. {
            if w.y > w.x {
                Vec3::Y
            } else {
                Vec3::new(grad.x, 0., grad.y)
            }
        } else {
            let d_or_zero = d.max(0.);
            Vec3::new(
                d_or_zero * grad.x,
                w.y.max(0.) * pos.y.signum(),
                d_or_zero * grad.y,
            )
            .normalize()
        }
    }
}
