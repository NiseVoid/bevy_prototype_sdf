use bevy_math::{bounding::*, *};

mod dim2;
mod dim3;

pub use dim2::{Arc, Sdf2d, Sdf2dPrimitive};
pub use dim3::{Sdf3d, Sdf3dShape};

#[cfg(not(feature = "serialize"))]
pub trait ConditionalSerialize {}

#[cfg(not(feature = "serialize"))]
impl<T> ConditionalSerialize for T {}

#[cfg(feature = "serialize")]
pub trait ConditionalSerialize: serde::Serialize + for<'de> serde::Deserialize<'de> {}

#[cfg(feature = "serialize")]
impl<T: serde::Serialize + for<'de> serde::Deserialize<'de>> ConditionalSerialize for T {}

#[cfg(not(feature = "bevy_asset"))]
pub trait ConditionalBevyReflect {}

#[cfg(not(feature = "bevy_asset"))]
impl<T> ConditionalBevyReflect for T {}

#[cfg(feature = "bevy_asset")]
pub trait ConditionalBevyReflect: bevy_reflect::TypePath {}

#[cfg(feature = "bevy_asset")]
impl<T: bevy_reflect::TypePath> ConditionalBevyReflect for T {}

pub trait Sdf<D: Dim>:
    SdfBounding<D>
    + Clone
    + Sync
    + Send
    + std::fmt::Debug
    + ConditionalSerialize
    + ConditionalBevyReflect
{
    fn distance(&self, pos: D::Position) -> f32;
    fn gradient(&self, pos: D::Position) -> D::Position;
}

pub trait SdfBounding<D: Dim> {
    fn aabb(&self, translation: D::Position, rotation: D::Rotation) -> D::Aabb;
    fn bounding_ball(&self, translation: D::Position, rotation: D::Rotation) -> D::Ball;
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serialize",
    serde(bound(
        deserialize = "D: for<'de2> serde::Deserialize<'de2>, Shape: for<'de2> serde::Deserialize<'de2>"
    ))
)]
#[cfg_attr(
    feature = "bevy_asset",
    derive(bevy_asset::Asset, bevy_reflect::TypePath)
)]
pub struct SdfTree<D: Dim, Shape: Sdf<D>> {
    // TODO: These should probably not be public?
    pub operations: Box<[SdfOperation<D>]>,
    pub shapes: Box<[Shape]>,
}

impl<D: Dim, Shape: Sdf<D>> Sdf<D> for SdfTree<D, Shape> {
    fn distance(&self, pos: D::Position) -> f32 {
        if self.operations.is_empty() {
            self.shapes[0].distance(pos)
        } else {
            self.operations[0].get_distance(pos, &self.operations, &self.shapes)
        }
    }

    fn gradient(&self, pos: D::Position) -> D::Position {
        if self.operations.is_empty() {
            self.shapes[0].gradient(pos)
        } else {
            self.operations[0].get_gradient(pos, &self.operations, &self.shapes)
        }
    }
}

impl<D: Dim, Shape: Sdf<D>> SdfBounding<D> for SdfTree<D, Shape> {
    fn aabb(&self, translation: D::Position, rotation: D::Rotation) -> D::Aabb {
        if self.operations.is_empty() {
            self.shapes[0].aabb(translation, rotation)
        } else {
            self.operations[0].get_aabb(translation, rotation, &self.operations, &self.shapes)
        }
    }

    fn bounding_ball(&self, translation: D::Position, rotation: D::Rotation) -> D::Ball {
        if self.operations.is_empty() {
            self.shapes[0].bounding_ball(translation, rotation)
        } else {
            self.operations[0].get_bounding_ball(
                translation,
                rotation,
                &self.operations,
                &self.shapes,
            )
        }
    }
}

impl<D: Dim, Shape: Sdf<D>> SdfTree<D, Shape> {
    pub fn union(self, b: impl Into<Self>) -> Self {
        let b = b.into();

        let start = SdfOperation::Union(
            self.start(1, 0),
            b.start(1 + self.operations.len(), self.shapes.len()),
        );
        Self::merged(start, self, b)
    }

    pub fn invert(self) -> Self {
        let start = SdfOperation::Invert(self.start(1, 0));
        Self::with_op(start, self)
    }

    pub fn translated(self, translation: D::Position) -> Self {
        let start = SdfOperation::Translate(self.start(1, 0), translation);
        Self::with_op(start, self)
    }

    pub fn rotated(self, rotation: D::Rotation) -> Self {
        let start = SdfOperation::Rotate(self.start(1, 0), rotation);
        Self::with_op(start, self)
    }

    fn merged(start: SdfOperation<D>, a: Self, b: Self) -> Self {
        let op_offset = a.operations.len();
        let shape_offset = a.operations.len();
        let operations = [start]
            .iter()
            .cloned()
            .chain(a.operations.iter().map(|op| op.fix(1, 0)))
            .chain(
                b.operations
                    .iter()
                    .map(|op| op.fix(1 + op_offset, shape_offset)),
            )
            .collect();
        let shapes = a.shapes.iter().chain(b.shapes.iter()).cloned().collect();

        Self { operations, shapes }
    }

    fn with_op(start: SdfOperation<D>, a: Self) -> Self {
        let operations = [start]
            .iter()
            .cloned()
            .chain(a.operations.iter().map(|op| op.fix(1, 0)))
            .collect();

        Self {
            operations,
            shapes: a.shapes,
        }
    }

    fn start(&self, op_offset: usize, shape_offset: usize) -> Index {
        if self.operations.is_empty() {
            Index::shape(shape_offset as u8)
        } else {
            Index::operation(op_offset as u8)
        }
    }

    pub fn get_point(&self, point: D::Position) -> D::Position {
        if self.operations.is_empty() {
            point
        } else {
            self.operations[0].get_point(point, &self.operations)
        }
    }
}

pub trait Dim:
    Clone + Copy + std::fmt::Debug + ConditionalSerialize + ConditionalBevyReflect
{
    type Position: Clone
        + Copy
        + Sync
        + Send
        + std::fmt::Debug
        + std::ops::Add<Output = Self::Position>
        + std::ops::Sub<Output = Self::Position>
        + std::ops::Neg<Output = Self::Position>
        + ConditionalSerialize;
    type Rotation: Clone
        + Copy
        + Sync
        + Send
        + std::fmt::Debug
        + Rotation<Self::Position>
        + ConditionalSerialize;

    type Aabb: BoundingVolume;
    type Ball: BoundingVolume;
}

pub trait Rotation<Pos> {
    fn inverse(self) -> Self;
    fn apply(self, pos: Pos) -> Pos;
    fn rotate(self, other: Self) -> Self;
}

impl Rotation<bevy_math::Vec2> for f32 {
    fn inverse(self) -> Self {
        -self
    }

    fn apply(self, pos: bevy_math::Vec2) -> bevy_math::Vec2 {
        let cos = self.cos();
        let sin = self.sin();

        bevy_math::Vec2 {
            x: pos.x * cos - pos.y * sin,
            y: pos.y * cos + pos.x * sin,
        }
    }

    fn rotate(self, other: Self) -> Self {
        self + other
    }
}

impl Rotation<bevy_math::Vec3> for bevy_math::Quat {
    fn inverse(self) -> Self {
        self.inverse()
    }

    fn apply(self, pos: bevy_math::Vec3) -> bevy_math::Vec3 {
        self * pos
    }

    fn rotate(self, other: Self) -> Self {
        self * other
    }
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct Index(u8);

impl Index {
    fn shape(idx: u8) -> Self {
        debug_assert!(idx <= 0b0111_1111);
        Index(idx)
    }

    fn operation(idx: u8) -> Self {
        debug_assert!(idx <= 0b0111_1111);
        Index(idx & 0b1000_0000)
    }

    #[inline(always)]
    fn fix(self, op_offset: usize, shape_offset: usize) -> Self {
        match self.0 & 0b1000_0000 {
            0 => Self::shape(self.0 + shape_offset as u8),
            _ => Self::operation((self.0 & 0b0111_1111) + op_offset as u8),
        }
    }

    fn get_distance<D: Dim>(
        &self,
        pos: D::Position,
        operations: &[SdfOperation<D>],
        shapes: &[impl Sdf<D>],
    ) -> f32 {
        let idx = (self.0 & 0b0111_1111) as usize;
        match self.0 & 0b1000_0000 {
            0 => shapes[idx].distance(pos),
            _ => operations[idx].get_distance(pos, operations, shapes),
        }
    }

    fn get_gradient<D: Dim>(
        &self,
        pos: D::Position,
        operations: &[SdfOperation<D>],
        shapes: &[impl Sdf<D>],
    ) -> D::Position {
        let idx = (self.0 & 0b0111_1111) as usize;
        match self.0 & 0b1000_0000 {
            0 => shapes[idx].gradient(pos),
            _ => operations[idx].get_gradient(pos, operations, shapes),
        }
    }

    fn get_aabb<D: Dim>(
        &self,
        translation: D::Position,
        rotation: D::Rotation,
        operations: &[SdfOperation<D>],
        shapes: &[impl Sdf<D>],
    ) -> D::Aabb {
        let idx = (self.0 & 0b0111_1111) as usize;
        match self.0 & 0b1000_0000 {
            0 => shapes[idx].aabb(translation, rotation),
            _ => operations[idx].get_aabb(translation, rotation, operations, shapes),
        }
    }

    fn get_bounding_ball<D: Dim>(
        &self,
        translation: D::Position,
        rotation: D::Rotation,
        operations: &[SdfOperation<D>],
        shapes: &[impl Sdf<D>],
    ) -> D::Ball {
        let idx = (self.0 & 0b0111_1111) as usize;
        match self.0 & 0b1000_0000 {
            0 => shapes[idx].bounding_ball(translation, rotation),
            _ => operations[idx].get_bounding_ball(translation, rotation, operations, shapes),
        }
    }

    fn get_point<D: Dim>(&self, point: D::Position, operations: &[SdfOperation<D>]) -> D::Position {
        let idx = (self.0 & 0b0111_1111) as usize;
        match self.0 & 0b1000_0000 {
            0 => point,
            _ => operations[idx].get_point(point, operations),
        }
    }
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub enum SdfOperation<D: Dim> {
    Union(Index, Index),
    Invert(Index),
    Translate(Index, D::Position),
    Rotate(Index, D::Rotation),
}

impl<D: Dim> SdfOperation<D> {
    fn get_distance(&self, pos: D::Position, operations: &[Self], shapes: &[impl Sdf<D>]) -> f32 {
        use SdfOperation::*;
        match *self {
            Union(a, b) => {
                let a = a.get_distance(pos, operations, shapes);
                let b = b.get_distance(pos, operations, shapes);
                a.min(b)
            }
            Invert(a) => -a.get_distance(pos, operations, shapes),
            Translate(a, translation) => a.get_distance(pos - translation, operations, shapes),
            Rotate(a, rotation) => {
                a.get_distance(rotation.inverse().apply(pos), operations, shapes)
            }
        }
    }

    fn get_gradient(
        &self,
        pos: D::Position,
        operations: &[Self],
        shapes: &[impl Sdf<D>],
    ) -> D::Position {
        use SdfOperation::*;
        match *self {
            Union(a, b) => {
                let a_dist = a.get_distance(pos, operations, shapes);
                let b_dist = b.get_distance(pos, operations, shapes);
                if a_dist < b_dist {
                    a.get_gradient(pos, operations, shapes)
                } else {
                    b.get_gradient(pos, operations, shapes)
                }
            }
            Invert(a) => -a.get_gradient(pos, operations, shapes),
            Translate(a, translation) => a.get_gradient(pos - translation, operations, shapes),
            Rotate(a, rotation) => {
                a.get_gradient(rotation.inverse().apply(pos), operations, shapes)
            }
        }
    }

    fn get_aabb(
        &self,
        translation: D::Position,
        rotation: D::Rotation,
        operations: &[Self],
        shapes: &[impl Sdf<D>],
    ) -> D::Aabb {
        use SdfOperation::*;
        match *self {
            Union(a, b) => a
                .get_aabb(translation, rotation, operations, shapes)
                .merge(&b.get_aabb(translation, rotation, operations, shapes)),
            Invert(a) => a.get_aabb(translation, rotation, operations, shapes),
            Translate(a, trans) => a.get_aabb(translation + trans, rotation, operations, shapes),
            Rotate(a, rot) => a.get_aabb(translation, rotation.rotate(rot), operations, shapes),
        }
    }

    fn get_bounding_ball(
        &self,
        translation: D::Position,
        rotation: D::Rotation,
        operations: &[Self],
        shapes: &[impl Sdf<D>],
    ) -> D::Ball {
        use SdfOperation::*;
        match *self {
            Union(a, b) => a
                .get_bounding_ball(translation, rotation, operations, shapes)
                .merge(&b.get_bounding_ball(translation, rotation, operations, shapes)),
            Invert(a) => a.get_bounding_ball(translation, rotation, operations, shapes),
            Translate(a, trans) => {
                a.get_bounding_ball(translation + trans, rotation, operations, shapes)
            }
            Rotate(a, rot) => {
                a.get_bounding_ball(translation, rotation.rotate(rot), operations, shapes)
            }
        }
    }

    fn get_point(&self, point: D::Position, operations: &[Self]) -> D::Position {
        use SdfOperation::*;
        match *self {
            Union(_, _) => todo!(),
            Invert(_) => point,
            Translate(a, translation) => a.get_point(point, operations) + translation,
            Rotate(a, rotation) => rotation.apply(a.get_point(point, operations)),
        }
    }

    fn fix(self, op: usize, shape: usize) -> Self {
        use SdfOperation::*;
        match self {
            Union(a, b) => Self::Union(a.fix(op, shape), b.fix(op, shape)),
            Invert(a) => Self::Invert(a.fix(op, shape)),
            Translate(a, translation) => Self::Translate(a.fix(op, shape), translation),
            Rotate(a, rotation) => Self::Rotate(a.fix(op, shape), rotation),
        }
    }
}
