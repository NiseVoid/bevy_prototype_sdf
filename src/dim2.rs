use crate::{Dim, Sdf, SdfBounding, SdfTree};
use bevy_math::{bounding::*, Vec2};

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct Dim2;

impl Dim for Dim2 {
    type Position = bevy_math::Vec2;
    type Rotation = f32;

    type Aabb = Aabb2d;
    type Ball = BoundingCircle;
}

impl<P: Sdf<Dim2> + Bounded2d> SdfBounding<Dim2> for P {
    fn aabb(&self, translation: Vec2, rotation: f32) -> Aabb2d {
        self.aabb_2d(translation, rotation)
    }

    fn bounding_ball(&self, translation: Vec2, rotation: f32) -> BoundingCircle {
        self.bounding_circle(translation, rotation)
    }
}

pub type Sdf2d = SdfTree<Dim2, Sdf2dPrimitive>;

impl<IntoShape: Into<Sdf2dPrimitive>> From<IntoShape> for Sdf2d {
    fn from(value: IntoShape) -> Self {
        let shape = value.into();
        Self {
            operations: Box::new([]),
            shapes: Box::new([shape]),
        }
    }
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub enum Sdf2dPrimitive {
    Arc(Arc),
}

impl From<Arc> for Sdf2dPrimitive {
    fn from(value: Arc) -> Self {
        Sdf2dPrimitive::Arc(value)
    }
}

impl SdfBounding<Dim2> for Sdf2dPrimitive {
    fn aabb(&self, translation: Vec2, rotation: f32) -> Aabb2d {
        use Sdf2dPrimitive::*;
        match self {
            Arc(a) => a.aabb(translation, rotation),
        }
    }

    fn bounding_ball(&self, translation: Vec2, rotation: f32) -> BoundingCircle {
        use Sdf2dPrimitive::*;
        match self {
            Arc(a) => a.bounding_ball(translation, rotation),
        }
    }
}

impl Sdf<Dim2> for Sdf2dPrimitive {
    fn distance(&self, pos: Vec2) -> f32 {
        use Sdf2dPrimitive::*;
        match self {
            Arc(a) => a.distance(pos),
        }
    }

    fn gradient(&self, pos: Vec2) -> Vec2 {
        use Sdf2dPrimitive::*;
        match self {
            Arc(a) => a.gradient(pos),
        }
    }
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
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
    fn aabb_2d(&self, translation: Vec2, rotation: f32) -> Aabb2d {
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

    fn bounding_circle(&self, translation: Vec2, rotation: f32) -> BoundingCircle {
        // TODO: This isn't an optimal bounding circle
        _ = rotation;
        BoundingCircle::new(translation, self.radius)
    }
}
