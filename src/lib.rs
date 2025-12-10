#![no_std]

pub mod dim2;
pub mod dim3;

#[cfg(feature = "alloc")]
mod tree;
#[cfg(feature = "alloc")]
pub use tree::SdfTree;

#[cfg(feature = "bevy")]
mod plugin;
#[cfg(feature = "bevy")]
pub use plugin::*;

#[cfg(feature = "serialize")]
pub mod binary_serde;
#[cfg(feature = "serialize")]
pub mod readable_serde;

pub use dim2::Arc;

#[cfg(feature = "alloc")]
pub use dim2::Sdf2d;
#[cfg(feature = "alloc")]
pub use dim3::Sdf3d;

#[cfg(feature = "alloc")]
extern crate alloc;
#[cfg(feature = "alloc")]
use alloc::vec::Vec;

use arrayvec::ArrayVec;
#[cfg(feature = "bevy")]
use bevy::reflect::{GetTypeRegistration, Typed, prelude::*};
#[cfg(feature = "shader")]
use bevy::render::render_resource::encase::internal::{BufferMut, WriteInto, Writer};
use bevy_math::{
    Dir3, EulerRot, Isometry2d, Isometry3d, Quat, Rot2, Vec2, Vec3, Vec3A, Vec3Swizzles,
    bounding::*, primitives::*, vec3, vec3a,
};

type Stack<T> = ArrayVec<T, 16>;

#[cfg(feature = "bevy")]
pub trait BevyReflect: Reflect + FromReflect + TypePath + GetTypeRegistration + Typed {}

#[cfg(feature = "bevy")]
impl<T: Reflect + FromReflect + TypePath + GetTypeRegistration + Typed> BevyReflect for T {}

#[cfg(not(feature = "bevy"))]
pub trait BevyReflect {}

#[cfg(not(feature = "bevy"))]
impl<T> BevyReflect for T {}

pub trait Sdf<D: Dim>: SdfBounding<D> + Clone + Sync + Send + core::fmt::Debug {
    fn distance(&self, pos: D::Position) -> f32;
    fn gradient(&self, pos: D::Position) -> D::Position;
    fn dist_grad(&self, pos: D::Position) -> (f32, D::Position);
}

pub trait SdfBounding<D: Dim> {
    fn aabb(&self, iso: D::Isometry) -> D::Aabb;
    fn bounding_ball(&self, iso: D::Isometry) -> D::Ball;
}

#[cfg_attr(feature = "bevy", derive(Reflect))]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    all(feature = "serialize", feature = "bevy"),
    reflect(Serialize, Deserialize)
)]
#[repr(u32)]
enum AnySdf {
    // Universal operations
    Union = 0b0000_0000_0000_0000,
    Subtract,
    Intersect,
    Invert,
    Shell,

    // 2D-only operations
    Translate2d = 0b0001_0000_0000_0000,
    Rotate2d,

    // 3D-only operations
    Translate3d = 0b0010_0000_0000_0000,
    Rotate3d,
    Extrude,
    Revolve,

    // 2D Primitives
    Circle = 0b1000_0000_0000_0000,
    Rectangle,
    Arc,

    // 3D primitives
    Sphere = 0b1001_0000_0000_0000,
    Capsule3d,
    Cylinder,
    Cuboid,
    InfinitePlane3d,
}

#[allow(dead_code)]
impl AnySdf {
    fn is_2d(&self) -> bool {
        use AnySdf::*;
        match self {
            Union | Subtract | Intersect | Invert | Translate2d | Rotate2d | Circle | Rectangle
            | Arc => true,
            _ => false,
        }
    }

    fn is_3d(&self) -> bool {
        use AnySdf::*;
        match self {
            Translate2d | Rotate2d | Circle | Rectangle | Arc => false,
            _ => true,
        }
    }
}

trait StorablePrimitive {
    const SIZE: u32;

    #[cfg(feature = "alloc")]
    fn store(&self, data: &mut Vec<f32>);
    fn load(data: &[f32]) -> Self;
}

impl StorablePrimitive for Circle {
    const SIZE: u32 = 1;

    #[cfg(feature = "alloc")]
    fn store(&self, data: &mut Vec<f32>) {
        data.push(self.radius);
    }
    fn load(data: &[f32]) -> Self {
        Self { radius: data[0] }
    }
}

impl StorablePrimitive for Rectangle {
    const SIZE: u32 = 2;

    #[cfg(feature = "alloc")]
    fn store(&self, data: &mut Vec<f32>) {
        data.extend(self.half_size.to_array());
    }
    fn load(data: &[f32]) -> Self {
        Self {
            half_size: Vec2::from_slice(data),
        }
    }
}

impl StorablePrimitive for Arc {
    const SIZE: u32 = 3;

    #[cfg(feature = "alloc")]
    fn store(&self, data: &mut Vec<f32>) {
        data.extend([self.radius, self.thickness, self.segment]);
    }
    fn load(data: &[f32]) -> Self {
        Self {
            radius: data[0],
            thickness: data[1],
            segment: data[2],
        }
    }
}

impl StorablePrimitive for Sphere {
    const SIZE: u32 = 1;

    #[cfg(feature = "alloc")]
    fn store(&self, data: &mut Vec<f32>) {
        data.push(self.radius);
    }
    fn load(data: &[f32]) -> Self {
        Self { radius: data[0] }
    }
}

impl StorablePrimitive for Capsule3d {
    const SIZE: u32 = 2;

    #[cfg(feature = "alloc")]
    fn store(&self, data: &mut Vec<f32>) {
        data.extend([self.radius, self.half_length]);
    }
    fn load(data: &[f32]) -> Self {
        Self {
            radius: data[0],
            half_length: data[1],
        }
    }
}

impl StorablePrimitive for Cylinder {
    const SIZE: u32 = 2;

    #[cfg(feature = "alloc")]
    fn store(&self, data: &mut Vec<f32>) {
        data.extend([self.radius, self.half_height]);
    }
    fn load(data: &[f32]) -> Self {
        Self {
            radius: data[0],
            half_height: data[1],
        }
    }
}

impl StorablePrimitive for Cuboid {
    const SIZE: u32 = 3;

    #[cfg(feature = "alloc")]
    fn store(&self, data: &mut Vec<f32>) {
        data.extend(self.half_size.to_array());
    }
    fn load(data: &[f32]) -> Self {
        Self {
            half_size: Vec3::from_slice(data),
        }
    }
}

impl StorablePrimitive for InfinitePlane3d {
    const SIZE: u32 = 3;

    #[cfg(feature = "alloc")]
    fn store(&self, data: &mut Vec<f32>) {
        data.extend(self.normal.to_array());
    }
    fn load(data: &[f32]) -> Self {
        Self {
            normal: Dir3::new_unchecked(Vec3::from_slice(data)),
        }
    }
}

trait ToNode {
    type D: Dim;
    fn node() -> AnySdf;
}

impl ToNode for Circle {
    type D = dim2::Dim2;
    fn node() -> AnySdf {
        AnySdf::Circle
    }
}

impl ToNode for Rectangle {
    type D = dim2::Dim2;
    fn node() -> AnySdf {
        AnySdf::Rectangle
    }
}

impl ToNode for Arc {
    type D = dim2::Dim2;
    fn node() -> AnySdf {
        AnySdf::Arc
    }
}

impl ToNode for Sphere {
    type D = dim3::Dim3;
    fn node() -> AnySdf {
        AnySdf::Sphere
    }
}

impl ToNode for Capsule3d {
    type D = dim3::Dim3;
    fn node() -> AnySdf {
        AnySdf::Capsule3d
    }
}

impl ToNode for Cylinder {
    type D = dim3::Dim3;
    fn node() -> AnySdf {
        AnySdf::Cylinder
    }
}

impl ToNode for Cuboid {
    type D = dim3::Dim3;
    fn node() -> AnySdf {
        AnySdf::Cuboid
    }
}

impl ToNode for InfinitePlane3d {
    type D = dim3::Dim3;
    fn node() -> AnySdf {
        AnySdf::InfinitePlane3d
    }
}

#[cfg_attr(feature = "bevy", derive(Reflect))]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    all(feature = "serialize", feature = "bevy"),
    reflect(Serialize, Deserialize)
)]
pub struct TreeNode {
    sdf: AnySdf,
    value: u32,
}

impl TreeNode {
    fn new(sdf: AnySdf, value: u32) -> Self {
        Self { sdf, value }
    }

    fn no_data(sdf: AnySdf) -> Self {
        Self { sdf, value: 0 }
    }
}

#[track_caller]
fn verify_primitive<P: StorablePrimitive>(data_len: u32, data: u32) -> u32 {
    assert!(data_len >= (data + P::SIZE - 1));
    P::SIZE
}

impl TreeNode {
    fn verify(&self, index: u32, node_len: u32, data_len: u32) -> u32 {
        let next = index + 1;
        match self.sdf {
            AnySdf::Union | AnySdf::Subtract | AnySdf::Intersect => {
                assert!(node_len >= next + self.value);
                0
            }
            AnySdf::Invert => {
                assert!(node_len >= next);
                0
            }
            AnySdf::Shell => {
                assert!(node_len >= next);
                assert!(data_len >= self.value);
                1
            }

            AnySdf::Translate2d => {
                assert!(node_len >= next);
                assert!(data_len >= self.value + 1);
                2
            }
            AnySdf::Rotate2d => {
                assert!(node_len >= next);
                assert!(data_len >= self.value);
                1
            }

            AnySdf::Translate3d => {
                assert!(node_len >= next);
                assert!(data_len >= self.value + 2);
                3
            }
            AnySdf::Rotate3d => {
                assert!(node_len >= next);
                assert!(data_len >= self.value + 3);
                4
            }
            AnySdf::Extrude => {
                assert!(node_len >= next);
                assert!(data_len >= self.value);
                1
            }
            AnySdf::Revolve => {
                assert!(node_len >= next);
                assert!(data_len >= self.value);
                1
            }

            AnySdf::Circle => verify_primitive::<Circle>(data_len, self.value),
            AnySdf::Rectangle => verify_primitive::<Rectangle>(data_len, self.value),
            AnySdf::Arc => verify_primitive::<Arc>(data_len, self.value),

            AnySdf::Sphere => verify_primitive::<Sphere>(data_len, self.value),
            AnySdf::Cylinder => verify_primitive::<Cylinder>(data_len, self.value),
            AnySdf::Capsule3d => verify_primitive::<Capsule3d>(data_len, self.value),
            AnySdf::Cuboid => verify_primitive::<Cuboid>(data_len, self.value),
            AnySdf::InfinitePlane3d => verify_primitive::<InfinitePlane3d>(data_len, self.value),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ExecutionNode {
    exec: AnyExec,
    value: u32,
}

#[cfg(feature = "shader")]
impl WriteInto for ExecutionNode {
    fn write_into<B>(&self, writer: &mut Writer<B>)
    where
        B: BufferMut,
    {
        writer.write(&(self.exec as u32).to_le_bytes());
        writer.write(&self.value.to_le_bytes());
    }
}

impl ExecutionNode {
    pub fn new(exec: AnyExec, value: u32) -> Self {
        Self { exec, value }
    }

    pub fn no_data(exec: AnyExec) -> Self {
        Self { exec, value: 0 }
    }

    pub fn exec(&self) -> u32 {
        self.exec as u32
    }

    pub fn value(&self) -> u32 {
        self.value
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u32)]
pub enum AnyExec {
    PushStack,
    PopPosition,
    Union,
    Subtract,
    Intersect,
    Invert,
    Shell,

    Translate2d,
    Rotate2d,

    Translate3d,
    Rotate3d,
    PreExtrude,
    Extrude,
    Revolve,
    PostRevolve,

    Circle,
    Rectangle,
    Arc,

    Sphere,
    Capsule,
    Cylinder,
    Cuboid,
    InfinitePlane3d,
}

#[cfg(feature = "alloc")]
#[derive(Debug, Default)]
pub struct ExecutionOrder(Vec<ExecutionNode>);

#[cfg(feature = "alloc")]
impl core::ops::Deref for ExecutionOrder {
    type Target = [ExecutionNode];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

pub trait ExecutableExt {
    fn distance(&self, pos: Vec3, slice: &[f32]) -> f32;
    fn gradient(&self, pos: Vec3, slice: &[f32]) -> Vec3;
    fn aabb(&self, iso: Isometry3d, slice: &[f32]) -> Aabb3d;
}

impl ExecutableExt for [ExecutionNode] {
    fn distance(&self, mut pos: Vec3, slice: &[f32]) -> f32 {
        use AnyExec::*;

        let mut dist = f32::INFINITY;
        let mut dist_stack = Stack::<f32>::new();
        let mut pos_stack = Stack::<Vec3>::new();

        for node in self.iter() {
            let data = node.value as usize;
            match node.exec {
                PushStack => {
                    dist_stack.push(dist);
                }
                Union => {
                    let a = dist_stack.pop().unwrap();
                    dist = a.min(dist)
                }
                Subtract => {
                    let a = dist_stack.pop().unwrap();
                    dist = a.max(-dist);
                }
                Intersect => {
                    let a = dist_stack.pop().unwrap();
                    dist = a.max(dist);
                }
                Invert => {
                    dist = -dist;
                }
                Shell => {
                    dist = dist.abs() - slice[data];
                }

                Translate2d => {
                    pos_stack.push(pos);
                    let data = Vec2::from_slice(&slice[data..]);
                    pos -= data.extend(0.);
                }
                Rotate2d => {
                    pos_stack.push(pos);
                    let angle = slice[data];
                    let rot = Rot2::radians(angle);
                    pos = (rot.inverse() * pos.xy()).extend(0.);
                }

                Translate3d => {
                    pos_stack.push(pos);
                    let data = &slice[data..(data + 3)];
                    pos -= Vec3::from_slice(data);
                }
                Rotate3d => {
                    pos_stack.push(pos);
                    let data = &slice[data..(data + 4)];
                    let rot = Quat::from_slice(data);
                    pos = rot.inverse() * pos;
                }
                PopPosition => {
                    pos = pos_stack.pop().unwrap();
                }
                PreExtrude => {
                    pos_stack.push(pos);
                    pos = pos.xz().extend(0.);
                }
                Extrude => {
                    pos = pos_stack.pop().unwrap();

                    let half_height = slice[data];
                    let w = Vec2::new(dist, pos.y.abs() - half_height);
                    dist = w.max_element().min(0.) + w.max(Vec2::ZERO).length();
                }
                Revolve => {
                    pos_stack.push(pos);
                    let offset = slice[data];
                    pos = Vec3::new(pos.xz().length() - offset, pos.y, 0.);
                }
                PostRevolve => {
                    pos = pos_stack.pop().unwrap();
                }

                Circle => {
                    let circle = bevy_math::primitives::Circle::load(&slice[data..]);
                    dist = circle.distance(pos.xy());
                }
                Rectangle => {
                    let rect = bevy_math::primitives::Rectangle::load(&slice[data..]);
                    dist = rect.distance(pos.xy());
                }
                Arc => {
                    let arc = dim2::Arc::load(&slice[data..]);
                    dist = arc.distance(pos.xy());
                }

                Sphere => {
                    let sphere = bevy_math::primitives::Sphere::load(&slice[data..]);
                    dist = sphere.distance(pos);
                }
                Capsule => {
                    let capsule = bevy_math::primitives::Capsule3d::load(&slice[data..]);
                    dist = capsule.distance(pos);
                }
                Cylinder => {
                    let cylinder = bevy_math::primitives::Cylinder::load(&slice[data..]);
                    dist = cylinder.distance(pos);
                }
                Cuboid => {
                    let cuboid = bevy_math::primitives::Cuboid::load(&slice[data..]);
                    dist = cuboid.distance(pos);
                }
                InfinitePlane3d => {
                    let plane = bevy_math::primitives::InfinitePlane3d::load(&slice[data..]);
                    dist = plane.distance(pos);
                }
            }
        }
        pos_stack.clear();
        dist_stack.clear();
        dist
    }

    fn gradient(&self, mut pos: Vec3, slice: &[f32]) -> Vec3 {
        use AnyExec::*;

        let (mut dist, mut grad) = (0., Vec3::ZERO);
        let mut stack = Stack::<(f32, Vec3)>::new();
        let mut pos_stack = Stack::<Vec3>::new();

        for node in self.iter() {
            let data = node.value as usize;
            match node.exec {
                PushStack => {
                    stack.push((dist, grad));
                }
                Union => {
                    // TODO: We need distances of a and b for this
                    todo!()
                }
                Subtract => {
                    // TODO: We need distances of a and b for this
                    todo!()
                }
                Intersect => {
                    // TODO: We need distances of a and b for this
                    todo!()
                }
                Invert => {
                    grad = -grad;
                }
                Shell => {
                    let old_dist = dist.abs();
                    dist = old_dist - slice[data];
                    if old_dist < 0. {
                        grad = -grad;
                    }
                }

                Translate2d | Rotate2d | Revolve => {
                    // TODO
                    todo!()
                }

                Translate3d => {
                    pos_stack.push(pos);
                    let data = &slice[data..(data + 3)];
                    pos -= Vec3::from_slice(data);
                }
                Rotate3d => {
                    pos_stack.push(pos);
                    let data = &slice[data..(data + 4)];
                    let rot = Quat::from_slice(data);
                    pos = rot.inverse() * pos;
                }
                PopPosition => {
                    pos = pos_stack.pop().unwrap();
                }
                PreExtrude => {
                    pos_stack.push(pos);
                    pos = pos.xz().extend(0.);
                }
                Extrude => {
                    pos = pos_stack.pop().unwrap();
                    use bevy_math::Vec2;

                    let w = Vec2::new(dist, pos.y.abs() - slice[data]);

                    if w.y <= 0. {
                        if w.y > w.x {
                            grad = Vec3::Y;
                        } else {
                            grad = Vec3::new(grad.x, 0., grad.y)
                        }
                    } else {
                        let d_or_zero = dist.max(0.);
                        grad = Vec3::new(
                            d_or_zero * grad.x,
                            w.y.max(0.) * pos.y.signum(),
                            d_or_zero * grad.y,
                        )
                        .normalize()
                    }
                    dist = w.x.max(w.y).min(0.0) + w.max(Vec2::ZERO).length();
                }
                PostRevolve => {
                    // TODO
                    todo!()
                }

                Circle => {
                    let circle = bevy_math::primitives::Circle::load(&slice[data..]);
                    let grad2d: Vec2;
                    (dist, grad2d) = circle.dist_grad(pos.xy());
                    grad = grad2d.extend(0.);
                }
                Rectangle => {
                    let rect = bevy_math::primitives::Rectangle::load(&slice[data..]);
                    let grad2d: Vec2;
                    (dist, grad2d) = rect.dist_grad(pos.xy());
                    grad = grad2d.extend(0.);
                }
                Arc => {
                    let arc = dim2::Arc::load(&slice[data..]);
                    let grad2d: Vec2;
                    (dist, grad2d) = arc.dist_grad(pos.xy());
                    grad = grad2d.extend(0.);
                }

                Sphere => {
                    let radius = slice[data];
                    (dist, grad) = bevy_math::primitives::Sphere { radius }.dist_grad(pos);
                }
                Capsule => {
                    let capsule = bevy_math::primitives::Capsule3d::load(&slice[data..]);
                    (dist, grad) = capsule.dist_grad(pos);
                }
                Cylinder => {
                    let cylinder = bevy_math::primitives::Cylinder::load(&slice[data..]);
                    (dist, grad) = cylinder.dist_grad(pos);
                }
                Cuboid => {
                    let cuboid = bevy_math::primitives::Cuboid::load(&slice[data..]);
                    (dist, grad) = cuboid.dist_grad(pos);
                }
                InfinitePlane3d => {
                    let plane = bevy_math::primitives::InfinitePlane3d::load(&slice[data..]);
                    (dist, grad) = plane.dist_grad(pos);
                }
            }
        }

        grad
    }

    fn aabb(&self, mut iso: Isometry3d, slice: &[f32]) -> Aabb3d {
        use AnyExec::*;

        let mut aabb = Aabb3d::new(Vec3A::ZERO, Vec3A::ZERO);
        let mut aabb_stack = Stack::<Aabb3d>::new();
        let mut pos_stack = Stack::<Isometry3d>::new();

        for node in self.iter() {
            let data = node.value as usize;
            match node.exec {
                PushStack => {
                    aabb_stack.push(aabb);
                }
                Union => {
                    let a = aabb_stack.pop().unwrap();
                    aabb = a.merge(&aabb);
                }
                Subtract => {
                    let a = aabb_stack.pop().unwrap();
                    aabb = a;
                }
                Intersect => {
                    let a = aabb_stack.pop().unwrap();
                    aabb = Aabb3d {
                        min: a.min.max(aabb.min),
                        max: a.max.min(aabb.max),
                    };
                    if aabb.min.cmpgt(aabb.max).any() {
                        aabb = Aabb3d {
                            min: Vec3A::ZERO,
                            max: Vec3A::ZERO,
                        };
                    }
                }
                Invert => {}
                Shell => {
                    aabb = aabb.grow(Vec3A::splat(slice[data]));
                }

                Translate2d => {
                    pos_stack.push(iso);
                    let x = slice[data];
                    let y = slice[data + 1];
                    iso.translation += vec3a(x, y, 0.);
                }
                Rotate2d => {
                    pos_stack.push(iso);

                    let angle = slice[data];
                    let rot = Quat::from_rotation_z(angle);
                    iso = iso.rotate(rot)
                }

                Translate3d => {
                    pos_stack.push(iso);
                    let data = &slice[data..(data + 3)];
                    iso = iso.translate(Vec3::from_slice(data))
                }
                Rotate3d => {
                    pos_stack.push(iso);
                    let data = &slice[data..(data + 4)];
                    let rot = Quat::from_slice(data);
                    iso = iso.rotate(rot);
                }
                PopPosition => {
                    iso = pos_stack.pop().unwrap();
                }
                PreExtrude => {
                    pos_stack.push(iso);
                    let (base_angle, _, _) = iso.rotation.to_euler(EulerRot::YXZ);

                    iso = Isometry3d {
                        translation: Vec3A::ZERO,
                        rotation: Quat::from_rotation_z(-base_angle),
                    }
                }
                Extrude => {
                    iso = pos_stack.pop().unwrap();

                    let (_, x, z) = iso.rotation.to_euler(EulerRot::YXZ);
                    let rot = Quat::from_euler(EulerRot::YXZ, 0., x, z);

                    let half_height = slice[data];
                    let min = aabb.min.xy();
                    let max = aabb.max.xy();

                    let a = rot * vec3a(min.x, -half_height, min.y);
                    let b = rot * vec3a(max.x, half_height, max.y);
                    aabb = Aabb3d {
                        min: a.min(b),
                        max: a.max(b),
                    }
                    .translated_by(iso.translation);
                }
                Revolve => {
                    pos_stack.push(iso);

                    let offset = slice[data];

                    iso = Isometry3d {
                        translation: Vec3A::new(offset, 0., 0.),
                        // TODO: Set rotation
                        rotation: Quat::default(),
                    };
                }
                PostRevolve => {
                    iso = pos_stack.pop().unwrap();

                    let min = aabb.min.xy();
                    let max = aabb.max.xy();
                    let x = max.x.max(0.);
                    // TODO: Rotation
                    aabb = Aabb3d {
                        min: vec3a(-x, min.y, -x),
                        max: vec3a(x, max.y, x),
                    }
                    .translated_by(iso.translation);
                }

                Circle => {
                    let comp = iso.rotation * vec3(1., 0., 0.);
                    let rotation = Rot2::from_sin_cos(comp.y, comp.x);
                    let iso = Isometry2d {
                        translation: iso.translation.xy(),
                        rotation,
                    };
                    let aabb2d = bevy_math::primitives::Circle::load(&slice[data..]).aabb_2d(iso);
                    aabb = Aabb3d {
                        min: aabb2d.min.extend(0.).into(),
                        max: aabb2d.max.extend(0.).into(),
                    };
                }
                Rectangle => {
                    let comp = iso.rotation * vec3(1., 0., 0.);
                    let rotation = Rot2::from_sin_cos(comp.y, comp.x);
                    let iso = Isometry2d {
                        translation: iso.translation.xy(),
                        rotation,
                    };
                    let aabb2d =
                        bevy_math::primitives::Rectangle::load(&slice[data..]).aabb_2d(iso);
                    aabb = Aabb3d {
                        min: aabb2d.min.extend(0.).into(),
                        max: aabb2d.max.extend(0.).into(),
                    };
                }
                Arc => {
                    let comp = iso.rotation * vec3(1., 0., 0.);
                    let rotation = Rot2::from_sin_cos(comp.y, comp.x);
                    let iso = Isometry2d {
                        translation: iso.translation.xy(),
                        rotation,
                    };
                    let aabb2d = dim2::Arc::load(&slice[data..]).aabb_2d(iso);
                    aabb = Aabb3d {
                        min: aabb2d.min.extend(0.).into(),
                        max: aabb2d.max.extend(0.).into(),
                    };
                }

                Sphere => {
                    aabb = bevy_math::primitives::Sphere::load(&slice[data..]).aabb_3d(iso);
                }
                Capsule => {
                    aabb = bevy_math::primitives::Capsule3d::load(&slice[data..]).aabb_3d(iso);
                }
                Cylinder => {
                    aabb = bevy_math::primitives::Cylinder::load(&slice[data..]).aabb_3d(iso);
                }
                Cuboid => {
                    aabb = bevy_math::primitives::Cuboid::load(&slice[data..]).aabb_3d(iso);
                }
                InfinitePlane3d => {
                    aabb =
                        bevy_math::primitives::InfinitePlane3d::load(&slice[data..]).aabb_3d(iso);
                }
            }
        }
        pos_stack.clear();
        aabb_stack.clear();
        aabb
    }
}

impl SdfBounding<dim3::Dim3> for (&[ExecutionNode], &[f32]) {
    fn aabb(&self, iso: Isometry3d) -> Aabb3d {
        self.0.aabb(iso, self.1)
    }

    fn bounding_ball(&self, iso: Isometry3d) -> BoundingSphere {
        _ = iso;
        todo!()
    }
}

pub trait Dim: Clone + Copy + Sync + Send + core::fmt::Debug + BevyReflect {
    const POS_SIZE: usize;
    const ROT_SIZE: usize;
    type Position: Clone
        + Copy
        + Sync
        + Send
        + core::fmt::Debug
        + core::ops::Add<Output = Self::Position>
        + core::ops::Sub<Output = Self::Position>
        + core::ops::Neg<Output = Self::Position>
        + BevyReflect;
    type Rotation: Clone
        + Copy
        + Sync
        + Send
        + core::fmt::Debug
        + Rotation<Self::Position>
        + BevyReflect;
    type Isometry: Clone
        + Copy
        + Sync
        + Send
        + core::fmt::Debug
        + BevyReflect
        + core::ops::Mul<Self::Position>
        + Isometry<Self::Position, Self::Rotation>;

    type Aabb: BoundingVolume;
    type Ball: BoundingVolume;
}

pub trait Isometry<Pos, Rot> {
    fn translate(self, trans: Pos) -> Self;
    fn rotate(self, rot: Rot) -> Self;
}

impl Isometry<Vec3, Quat> for Isometry3d {
    fn translate(self, trans: Vec3) -> Self {
        Self {
            rotation: self.rotation,
            translation: self.translation + self.rotation.inverse() * bevy_math::Vec3A::from(trans),
        }
    }

    fn rotate(self, rot: Quat) -> Self {
        Self {
            rotation: rot * self.rotation,
            translation: self.translation,
        }
    }
}

impl Isometry<Vec2, Rot2> for Isometry2d {
    fn translate(self, trans: Vec2) -> Self {
        Self {
            rotation: self.rotation,
            translation: self.translation + self.rotation.inverse() * trans,
        }
    }

    fn rotate(self, rot: Rot2) -> Self {
        Self {
            rotation: rot * self.rotation,
            translation: self.translation,
        }
    }
}

pub trait Rotation<Pos> {
    fn inverse(self) -> Self;
    fn apply(self, pos: Pos) -> Pos;
    fn rotate(self, other: Self) -> Self;
}

impl Rotation<Vec2> for Rot2 {
    fn inverse(self) -> Self {
        self.inverse()
    }

    fn apply(self, pos: Vec2) -> Vec2 {
        Vec2 {
            x: pos.x * self.cos - pos.y * self.sin,
            y: pos.y * self.cos + pos.x * self.sin,
        }
    }

    fn rotate(self, other: Self) -> Self {
        self * other
    }
}

impl Rotation<Vec3> for Quat {
    fn inverse(self) -> Self {
        self.inverse()
    }

    fn apply(self, pos: Vec3) -> Vec3 {
        self * pos
    }

    fn rotate(self, other: Self) -> Self {
        self * other
    }
}
