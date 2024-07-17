use crate::{Dim, Sdf, SdfBounding, SdfTree};

use bevy_math::{bounding::*, primitives::*, Rot2, Vec2};

#[cfg(all(feature = "bevy_reflect", feature = "serialize"))]
use bevy_reflect::ReflectDeserialize;

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "bevy_reflect", derive(bevy_reflect::Reflect))]
#[cfg_attr(
    all(feature = "bevy_reflect", feature = "serialize"),
    reflect(Deserialize)
)]
pub struct Dim2;

impl Dim for Dim2 {
    const POS_SIZE: usize = 2;
    const ROT_SIZE: usize = 1;
    type Position = bevy_math::Vec2;
    type Rotation = Rot2;

    type Aabb = Aabb2d;
    type Ball = BoundingCircle;
}

impl<P: Sdf<Dim2> + Bounded2d> SdfBounding<Dim2> for P {
    fn aabb(&self, translation: Vec2, rotation: impl Into<Rot2>) -> Aabb2d {
        self.aabb_2d(translation, rotation)
    }

    fn bounding_ball(&self, translation: Vec2, rotation: impl Into<Rot2>) -> BoundingCircle {
        self.bounding_circle(translation, rotation)
    }
}

pub type Sdf2d = SdfTree<Dim2, Sdf2dPrimitive>;

impl<IntoShape: Into<Sdf2dPrimitive>> From<IntoShape> for Sdf2d {
    fn from(value: IntoShape) -> Self {
        let shape = value.into();
        Self {
            operations: Vec::new(),
            shapes: vec![shape],
        }
    }
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "bevy_reflect", derive(bevy_reflect::Reflect))]
#[cfg_attr(
    all(feature = "bevy_reflect", feature = "serialize"),
    reflect(Deserialize)
)]
pub enum Sdf2dPrimitive {
    Circle(Circle),
    // Triangle(Triangle2d),
    Rectangle(Rectangle),
    // TODO:
    // Rhombus
    // Parallelogram
    // Vesica
    // Heart
    // Moon
    Arc(Arc),
}

impl From<Circle> for Sdf2dPrimitive {
    fn from(value: Circle) -> Self {
        Self::Circle(value)
    }
}

impl From<Rectangle> for Sdf2dPrimitive {
    fn from(value: Rectangle) -> Self {
        Self::Rectangle(value)
    }
}

impl From<Arc> for Sdf2dPrimitive {
    fn from(value: Arc) -> Self {
        Sdf2dPrimitive::Arc(value)
    }
}

impl SdfBounding<Dim2> for Sdf2dPrimitive {
    fn aabb(&self, translation: Vec2, rotation: impl Into<Rot2>) -> Aabb2d {
        use Sdf2dPrimitive::*;
        match self {
            Circle(c) => c.aabb_2d(translation, rotation),
            // Triangle(t) => t.aabb_2d(translation, rotation),
            Rectangle(r) => r.aabb_2d(translation, rotation),
            Arc(a) => a.aabb(translation, rotation),
        }
    }

    fn bounding_ball(&self, translation: Vec2, rotation: impl Into<Rot2>) -> BoundingCircle {
        use Sdf2dPrimitive::*;
        match self {
            Circle(c) => c.bounding_circle(translation, rotation),
            // Triangle(t) => t.bounding_circle(translation, rotation),
            Rectangle(r) => r.bounding_circle(translation, rotation),
            Arc(a) => a.bounding_ball(translation, rotation),
        }
    }
}

impl Sdf<Dim2> for Sdf2dPrimitive {
    fn distance(&self, pos: Vec2) -> f32 {
        use Sdf2dPrimitive::*;
        match self {
            Circle(c) => c.distance(pos),
            // Triangle(t) => t.distance(pos),
            Rectangle(r) => r.distance(pos),
            Arc(a) => a.distance(pos),
        }
    }

    fn gradient(&self, pos: Vec2) -> Vec2 {
        use Sdf2dPrimitive::*;
        match self {
            Circle(c) => c.gradient(pos),
            // Triangle(t) => t.gradient(pos),
            Rectangle(r) => r.gradient(pos),
            Arc(a) => a.gradient(pos),
        }
    }
}

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

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "bevy_reflect", derive(bevy_reflect::Reflect))]
#[cfg_attr(
    all(feature = "bevy_reflect", feature = "serialize"),
    reflect(Deserialize)
)]
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
            segment: (angle / 2.).clamp(0., std::f32::consts::PI),
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
    fn aabb_2d(&self, translation: Vec2, rotation: impl Into<Rot2>) -> Aabb2d {
        // TODO
        _ = rotation;
        let r = self.radius + self.thickness;
        let half_width = if self.segment < std::f32::consts::PI / 2. {
            self.radius * self.segment.sin() + self.thickness
        } else {
            r
        };
        let bottom = self.segment.cos() * self.radius - self.thickness;
        Aabb2d {
            min: Vec2::new(-half_width, bottom) + translation,
            max: Vec2::new(half_width, r) + translation,
        }
    }

    fn bounding_circle(&self, translation: Vec2, rotation: impl Into<Rot2>) -> BoundingCircle {
        // TODO: This isn't an optimal bounding circle
        _ = rotation;
        BoundingCircle::new(translation, self.radius + self.thickness)
    }
}
