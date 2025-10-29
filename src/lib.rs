#![no_std]

pub mod dim2;
pub mod dim3;

use core::marker::PhantomData;

use arrayvec::ArrayVec;
pub use dim2::{Arc, Sdf2d};
pub use dim3::Sdf3d;

use bevy::{
    math::{bounding::*, vec3, vec3a},
    prelude::*,
    reflect::{GetTypeRegistration, Typed},
};

#[cfg(feature = "bevy_asset")]
use bevy::{
    asset::{AssetEventSystems, AssetIndex, AssetLoader, LoadContext, io::Reader},
    ecs::system::SystemParam,
};
#[cfg(feature = "bevy_asset")]
use thiserror::Error;

type Stack<T> = ArrayVec<T, 16>;

pub struct SdfPlugin;

impl Plugin for SdfPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<Sdf2d>().register_type::<Sdf3d>();

        #[cfg(feature = "serialize")]
        app.register_type::<PrettySerdeTree2d>()
            .register_type::<PrettySerdeTree3d>();

        #[cfg(feature = "bevy_asset")]
        app.init_asset::<Sdf3d>()
            .init_asset::<Sdf2d>()
            .init_asset_loader::<SdfAssetLoader<dim3::Dim3>>()
            .init_resource::<ProcessedSdfs<dim2::Dim2>>()
            .init_resource::<ProcessedSdfs<dim3::Dim3>>()
            .add_systems(
                PostUpdate,
                (
                    process_sdf_trees::<dim2::Dim2>,
                    process_sdf_trees::<dim3::Dim3>,
                )
                    .in_set(SdfProcessing)
                    .after(AssetEventSystems),
            );

        #[cfg(feature = "shader")]
        {
            bevy::asset::embedded_asset!(app, "sdf.wgsl");
            core::mem::forget(
                app.world()
                    .resource::<AssetServer>()
                    .load::<Shader>("embedded://bevy_prototype_sdf/sdf.wgsl"),
            );
        }
    }
}

#[cfg(feature = "bevy_asset")]
struct SdfAssetLoader<D: Dim>(PhantomData<D>);

#[cfg(feature = "bevy_asset")]
impl<D: Dim> Default for SdfAssetLoader<D> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

#[cfg(feature = "bevy_asset")]
extern crate std;

/// Possible errors that can be produced by [`SdfAssetLoader`]
#[cfg(feature = "bevy_asset")]
#[non_exhaustive]
#[derive(Debug, Error)]
enum SdfAssetLoaderError {
    /// An [IO](std::io) Error
    #[error("Could not load asset: {0}")]
    Io(#[from] std::io::Error),
    /// A [RON](ron) Error
    #[error("Could not parse RON: {0}")]
    RonSpannedError(#[from] ron::error::SpannedError),
}

#[cfg(feature = "bevy_asset")]
impl AssetLoader for SdfAssetLoader<dim3::Dim3> {
    type Asset = SdfTree<dim3::Dim3>;
    type Settings = ();
    type Error = SdfAssetLoaderError;
    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &(),
        _load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        let serde_tree = ron::de::from_bytes::<PrettySerdeTree3d>(&bytes)?;
        Ok(serde_tree.to_tree())
    }

    fn extensions(&self) -> &[&str] {
        &["sdf3d"]
    }
}

/// A system set that handles processing of SDFs, runs in Last
#[cfg(feature = "bevy_asset")]
#[derive(SystemSet, Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct SdfProcessing;

pub trait BevyReflect: Reflect + FromReflect + TypePath + GetTypeRegistration + Typed {}

impl<T: Reflect + FromReflect + TypePath + GetTypeRegistration + Typed> BevyReflect for T {}

pub trait Sdf<D: Dim>:
    SdfBounding<D> + Clone + Sync + Send + core::fmt::Debug + BevyReflect
{
    fn distance(&self, pos: D::Position) -> f32;
    fn gradient(&self, pos: D::Position) -> D::Position;
    fn dist_grad(&self, pos: D::Position) -> (f32, D::Position);
}

pub trait SdfBounding<D: Dim> {
    fn aabb(&self, iso: D::Isometry) -> D::Aabb;
    fn bounding_ball(&self, iso: D::Isometry) -> D::Ball;
}

#[derive(Reflect, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
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

    fn store(&self, data: &mut Vec<f32>);
    fn load(data: &[f32]) -> Self;
}

impl StorablePrimitive for Circle {
    const SIZE: u32 = 1;

    fn store(&self, data: &mut Vec<f32>) {
        data.push(self.radius);
    }
    fn load(data: &[f32]) -> Self {
        Self { radius: data[0] }
    }
}

impl StorablePrimitive for Rectangle {
    const SIZE: u32 = 2;

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

    fn store(&self, data: &mut Vec<f32>) {
        data.push(self.radius);
    }
    fn load(data: &[f32]) -> Self {
        Self { radius: data[0] }
    }
}

impl StorablePrimitive for Capsule3d {
    const SIZE: u32 = 2;

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

#[derive(Reflect, Clone, Debug)]
#[cfg_attr(feature = "bevy_asset", derive(Asset))]
pub struct SdfTree<D: Dim> {
    nodes: Vec<TreeNode>,
    data: Vec<f32>,
    #[reflect(ignore)]
    phantom: PhantomData<D>,
}

impl<D: Dim> SdfTree<D> {
    fn add_primitive<P: StorablePrimitive + ToNode>(&mut self, p: P) {
        self.nodes
            .push(TreeNode::new(P::node(), self.data.len() as u32));
        p.store(&mut self.data);
    }
}

impl<D: Dim, P: StorablePrimitive + ToNode<D = D>> From<P> for SdfTree<D> {
    fn from(p: P) -> Self {
        let mut tree = SdfTree {
            nodes: Vec::new(),
            data: Vec::new(),
            phantom: PhantomData,
        };
        tree.add_primitive(p);
        tree
    }
}

impl<P: StorablePrimitive + ToNode<D = dim2::Dim2>> From<(P, f32)> for SdfTree<dim3::Dim3> {
    fn from((p, half_height): (P, f32)) -> Self {
        let mut tree = SdfTree {
            nodes: vec![TreeNode::new(AnySdf::Extrude, 0)],
            data: vec![half_height],
            phantom: PhantomData,
        };
        tree.add_primitive(p);
        tree
    }
}

impl<D: Dim> PartialEq for SdfTree<D> {
    fn eq(&self, other: &Self) -> bool {
        self.nodes == other.nodes && self.data == other.data
    }
}

impl<D: Dim> SdfTree<D> {
    pub fn data(&self) -> &[f32] {
        &self.data
    }
}

#[derive(Reflect, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
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

impl<D: Dim> SdfTree<D> {
    fn verify(&self) {
        let data_len = self.data.len() as u32;
        let node_len = self.nodes.len() as u32;
        let mut total_data = 0;

        for (index, node) in self.nodes.iter().enumerate() {
            let index = index as u32;
            total_data += node.verify(index as u32, node_len, data_len);
        }

        assert_eq!(total_data, data_len);
    }
}

#[cfg(feature = "serialize")]
#[derive(Deref, DerefMut)]
/// A wrapper providing raw serialization for SDF trees. This serialization is not intended to be
/// stable or human readable. If you want either of those properties, use [`PrettySerdeTree3d`] instead.
pub struct RawSerdeTree<D: Dim>(pub SdfTree<D>);

#[cfg(feature = "serialize")]
impl<D: Dim> serde::Serialize for RawSerdeTree<D> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut s = serializer.serialize_struct("SdfTree", 2)?;
        s.serialize_field("nodes", &self.0.nodes)?;
        s.serialize_field("data", &self.0.data)?;
        s.end()
    }
}

#[cfg(feature = "serialize")]
impl<'de, Di: Dim> serde::Deserialize<'de> for RawSerdeTree<Di> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(serde::Deserialize)]
        #[serde(field_identifier, rename_all = "lowercase")]
        enum Field {
            Nodes,
            Data,
        }

        struct StructVisitor<Di: Dim>(core::marker::PhantomData<Di>);
        impl<'de3, Di: Dim> serde::de::Visitor<'de3> for StructVisitor<Di> {
            type Value = SdfTree<Di>;

            fn expecting(&self, formatter: &mut core::fmt::Formatter) -> core::fmt::Result {
                formatter.write_str("struct SdfTree")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: serde::de::SeqAccess<'de3>,
            {
                let nodes = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(0, &self))?;
                let data = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(1, &self))?;
                let tree = SdfTree {
                    nodes,
                    data,
                    phantom: PhantomData::<Di>,
                };
                Ok(tree)
            }

            fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
            where
                V: serde::de::MapAccess<'de3>,
            {
                let mut nodes = None;
                let mut data = None;
                while let Some(key) = map.next_key()? {
                    match key {
                        Field::Nodes => {
                            if nodes.is_some() {
                                return Err(serde::de::Error::duplicate_field("nodes"));
                            }
                            nodes = Some(map.next_value()?);
                        }
                        Field::Data => {
                            if data.is_some() {
                                return Err(serde::de::Error::duplicate_field("data"));
                            }
                            data = Some(map.next_value()?);
                        }
                    }
                }
                let nodes = nodes.ok_or_else(|| serde::de::Error::missing_field("nodes"))?;
                let data = data.ok_or_else(|| serde::de::Error::missing_field("data"))?;
                Ok(SdfTree {
                    nodes,
                    data,
                    phantom: PhantomData::<Di>,
                })
            }
        }

        let tree = deserializer.deserialize_struct(
            "SdfTree",
            &["nodes", "data"],
            StructVisitor(core::marker::PhantomData::<Di>),
        )?;

        tree.verify();
        Ok(Self(tree))
    }
}

#[cfg(feature = "serialize")]
#[test]
fn test_raw_serde() {
    use AnySdf::*;
    let tree = SdfTree::<dim3::Dim3> {
        nodes: vec![
            TreeNode::new(Union, 1),
            TreeNode::new(Sphere, 0),
            TreeNode::new(Translate3d, 1),
            TreeNode::new(Cuboid, 4),
        ],
        data: vec![1., 0., 0., 0.2, 1., 1., 0.5],
        phantom: PhantomData,
    };
    let raw = RawSerdeTree(tree);

    // Test with ron
    let serialized = ron::ser::to_string(&raw).unwrap();
    let de: RawSerdeTree<dim3::Dim3> = ron::de::from_str(&serialized).unwrap();
    assert_eq!(de.0, raw.0);

    // Test with postcard
    let serialized = postcard::to_stdvec(&raw).unwrap();
    let de: RawSerdeTree<dim3::Dim3> = postcard::from_bytes(&serialized).unwrap();
    assert_eq!(de.0, raw.0);
}

#[cfg(feature = "serialize")]
#[test]
#[should_panic]
fn test_invalid_tree() {
    let tree = SdfTree::<dim3::Dim3> {
        nodes: vec![TreeNode::new(AnySdf::Union, 1)],
        data: vec![],
        phantom: PhantomData,
    };
    let raw = RawSerdeTree(tree);

    let serialized = postcard::to_stdvec(&raw).unwrap();
    let _: RawSerdeTree<dim3::Dim3> = postcard::from_bytes(&serialized).unwrap();
}

#[cfg(feature = "serialize")]
#[derive(Clone, Reflect, Debug, serde::Serialize, serde::Deserialize)]
#[reflect(opaque)]
#[reflect(Clone, Serialize, Deserialize)]
pub enum PrettySerdeTree2d {
    // Operations
    Union(Box<Self>, Box<Self>),
    Subtract(Box<Self>, Box<Self>),
    Intersect(Box<Self>, Box<Self>),
    Invert(Box<Self>),
    Shell(Box<Self>, f32),

    Translate(Box<Self>, (f32, f32)),
    Rotate(Box<Self>, f32),

    // 2D primitives
    Circle(f32),
    Rectangle(f32, f32),
    Arc {
        radius: f32,
        angle: f32,
        thickness: f32,
    },
}

#[cfg(feature = "serialize")]
#[derive(Clone, Reflect, Debug, serde::Serialize, serde::Deserialize)]
#[reflect(opaque)]
#[reflect(Clone, Serialize, Deserialize)]
pub enum PrettySerdeTree3d {
    // Operations
    Union(Box<Self>, Box<Self>),
    Subtract(Box<Self>, Box<Self>),
    Intersect(Box<Self>, Box<Self>),
    Invert(Box<Self>),
    Shell(Box<Self>, f32),

    Translate(Box<Self>, (f32, f32, f32)),
    Rotate(Box<Self>, (f32, f32, f32)),
    Extrude(Box<PrettySerdeTree2d>, f32), // f32 is height
    Revolve(Box<PrettySerdeTree2d>, f32), // f32 is offset

    // 3D primitives
    Sphere(f32),
    Cylinder { radius: f32, half_height: f32 },
    Capsule { radius: f32, half_length: f32 },
    Cuboid(f32, f32, f32),
    InfinitePlane(f32, f32, f32),
}

#[cfg(feature = "serialize")]
impl PrettySerdeTree3d {
    pub fn from_tree(tree: &SdfTree<dim3::Dim3>) -> Self {
        Self::from_node(&tree, 0)
    }

    fn from_node(tree: &SdfTree<dim3::Dim3>, index: u32) -> Self {
        let next = index + 1;
        let node = tree.nodes[index as usize];
        let data = node.value as usize;
        match node.sdf {
            AnySdf::Union => Self::Union(
                Box::new(Self::from_node(&tree, next)),
                Box::new(Self::from_node(&tree, next + node.value)),
            ),
            AnySdf::Subtract => Self::Subtract(
                Box::new(Self::from_node(&tree, next)),
                Box::new(Self::from_node(&tree, next + node.value)),
            ),
            AnySdf::Intersect => Self::Intersect(
                Box::new(Self::from_node(&tree, next)),
                Box::new(Self::from_node(&tree, next + node.value)),
            ),
            AnySdf::Invert => Self::Invert(Box::new(Self::from_node(&tree, next))),
            AnySdf::Shell => Self::Shell(Box::new(Self::from_node(&tree, next)), tree.data[data]),

            AnySdf::Translate3d => Self::Translate(
                Box::new(Self::from_node(&tree, next)),
                (tree.data[data], tree.data[data + 1], tree.data[data + 2]),
            ),
            AnySdf::Rotate3d => {
                let quat = Quat::from_slice(&tree.data[data..]);
                let (y, x, z) = quat.to_euler(EulerRot::YXZ);
                Self::Rotate(Box::new(Self::from_node(&tree, next)), (x, y, z))
            }
            AnySdf::Extrude => Self::Extrude(
                Box::new(PrettySerdeTree2d::from_node(&tree, next)),
                tree.data[data],
            ),
            AnySdf::Revolve => Self::Revolve(
                Box::new(PrettySerdeTree2d::from_node(&tree, next)),
                tree.data[data],
            ),

            AnySdf::Translate2d
            | AnySdf::Rotate2d
            | AnySdf::Circle
            | AnySdf::Rectangle
            | AnySdf::Arc => {
                panic!("2D primitives and operations should only exist in 2D context");
            }

            AnySdf::Sphere => {
                let circle = Sphere::load(&tree.data[data..]);
                Self::Sphere(circle.radius)
            }
            AnySdf::Cylinder => {
                let cylinder = Cylinder::load(&tree.data[data..]);
                Self::Cylinder {
                    radius: cylinder.radius,
                    half_height: cylinder.half_height,
                }
            }
            AnySdf::Capsule3d => {
                let capsule = Capsule3d::load(&tree.data[data..]);
                Self::Capsule {
                    radius: capsule.radius,
                    half_length: capsule.half_length,
                }
            }
            AnySdf::Cuboid => {
                let half_size = Cuboid::load(&tree.data[data..]).half_size;
                Self::Cuboid(half_size.x, half_size.y, half_size.z)
            }
            AnySdf::InfinitePlane3d => {
                let normal = InfinitePlane3d::load(&tree.data[data..]).normal;
                Self::InfinitePlane(normal.x, normal.y, normal.z)
            }
        }
    }

    pub fn to_tree(self) -> SdfTree<dim3::Dim3> {
        let mut tree = SdfTree::<dim3::Dim3> {
            nodes: default(),
            data: default(),
            phantom: PhantomData,
        };
        self.add_to_tree(&mut tree);

        #[cfg(debug_assertions)]
        tree.verify();

        tree
    }

    fn add_to_tree(self, tree: &mut SdfTree<dim3::Dim3>) {
        let node_idx = tree.nodes.len() as u32;
        let data_idx = tree.data.len() as u32;
        match self {
            Self::Union(a, b) => {
                tree.nodes.push(TreeNode::no_data(AnySdf::Union));
                a.add_to_tree(tree);
                tree.nodes[node_idx as usize].value = tree.nodes.len() as u32 - node_idx - 1;
                b.add_to_tree(tree);
            }
            Self::Subtract(a, b) => {
                tree.nodes.push(TreeNode::no_data(AnySdf::Subtract));
                a.add_to_tree(tree);
                tree.nodes[node_idx as usize].value = tree.nodes.len() as u32 - node_idx - 1;
                b.add_to_tree(tree);
            }
            Self::Intersect(a, b) => {
                tree.nodes.push(TreeNode::no_data(AnySdf::Intersect));
                a.add_to_tree(tree);
                tree.nodes[node_idx as usize].value = tree.nodes.len() as u32 - node_idx - 1;
                b.add_to_tree(tree);
            }
            Self::Invert(next) => {
                tree.nodes.push(TreeNode::no_data(AnySdf::Invert));
                next.add_to_tree(tree);
            }
            Self::Shell(next, thickness) => {
                tree.nodes.push(TreeNode::new(AnySdf::Shell, data_idx));
                tree.data.push(thickness);
                next.add_to_tree(tree);
            }

            Self::Translate(next, (x, y, z)) => {
                tree.nodes
                    .push(TreeNode::new(AnySdf::Translate3d, data_idx));
                tree.data.extend([x, y, z]);
                next.add_to_tree(tree);
            }
            Self::Rotate(next, (x, y, z)) => {
                let quat = Quat::from_euler(EulerRot::YXZ, y, x, z);
                tree.nodes.push(TreeNode::new(AnySdf::Rotate3d, data_idx));
                tree.data.extend(quat.to_array());
                next.add_to_tree(tree);
            }
            Self::Extrude(next_2d, half_depth) => {
                tree.nodes.push(TreeNode::new(AnySdf::Extrude, data_idx));
                tree.data.push(half_depth);
                next_2d.add_to_tree(tree);
            }
            Self::Revolve(next_2d, offset) => {
                tree.nodes.push(TreeNode::new(AnySdf::Revolve, data_idx));
                tree.data.push(offset);
                next_2d.add_to_tree(tree);
            }

            Self::Sphere(radius) => {
                tree.add_primitive(Sphere { radius });
            }
            Self::Cylinder {
                radius,
                half_height,
            } => {
                tree.add_primitive(Cylinder {
                    radius,
                    half_height,
                });
            }
            Self::Capsule {
                radius,
                half_length,
            } => {
                tree.add_primitive(Capsule3d {
                    radius,
                    half_length,
                });
            }
            Self::Cuboid(x, y, z) => {
                tree.add_primitive(Cuboid {
                    half_size: Vec3::new(x, y, z),
                });
            }
            Self::InfinitePlane(x, y, z) => {
                tree.add_primitive(InfinitePlane3d {
                    normal: Dir3::new(Vec3::new(x, y, z)).unwrap(),
                });
            }
        }
    }
}

#[cfg(feature = "serialize")]
impl PrettySerdeTree2d {
    pub fn from_tree(tree: &SdfTree<dim2::Dim2>) -> Self {
        Self::from_node(&tree, 0)
    }

    fn from_node<D: Dim>(tree: &SdfTree<D>, index: u32) -> Self {
        let next = index + 1;
        let node = tree.nodes[index as usize];
        let data = node.value as usize;
        match node.sdf {
            AnySdf::Union => Self::Union(
                Box::new(Self::from_node(&tree, next)),
                Box::new(Self::from_node(&tree, next + node.value)),
            ),
            AnySdf::Subtract => Self::Subtract(
                Box::new(Self::from_node(&tree, next)),
                Box::new(Self::from_node(&tree, next + node.value)),
            ),
            AnySdf::Intersect => Self::Intersect(
                Box::new(Self::from_node(&tree, next)),
                Box::new(Self::from_node(&tree, next + node.value)),
            ),
            AnySdf::Invert => Self::Invert(Box::new(Self::from_node(&tree, next))),
            AnySdf::Shell => Self::Shell(Box::new(Self::from_node(&tree, next)), tree.data[data]),

            AnySdf::Translate2d => Self::Translate(
                Box::new(Self::from_node(&tree, next)),
                (tree.data[data], tree.data[data + 1]),
            ),
            AnySdf::Rotate2d => {
                Self::Rotate(Box::new(Self::from_node(&tree, next)), tree.data[data])
            }

            AnySdf::Translate3d | AnySdf::Rotate3d | AnySdf::Extrude | AnySdf::Revolve => {
                panic!("3D operations can't be turned into a 2D tree");
            }

            AnySdf::Circle => {
                let circle = Circle::load(&tree.data[data..]);
                Self::Circle(circle.radius)
            }
            AnySdf::Rectangle => {
                let half_size = Rectangle::load(&tree.data[data..]).half_size;
                Self::Rectangle(half_size.x, half_size.y)
            }
            AnySdf::Arc => {
                let arc = Arc::load(&tree.data[data..]);
                Self::Arc {
                    radius: arc.radius,
                    thickness: arc.thickness,
                    angle: arc.segment * 2.,
                }
            }

            AnySdf::Sphere
            | AnySdf::Capsule3d
            | AnySdf::Cylinder
            | AnySdf::Cuboid
            | AnySdf::InfinitePlane3d => {
                panic!("3D primitives can't be turned into a 2D tree");
            }
        }
    }

    pub fn to_tree(self) -> SdfTree<dim2::Dim2> {
        let mut tree = SdfTree::<dim2::Dim2> {
            nodes: default(),
            data: default(),
            phantom: PhantomData,
        };
        self.add_to_tree(&mut tree);

        #[cfg(debug_assertions)]
        tree.verify();

        tree
    }

    fn add_to_tree<D: Dim>(self, tree: &mut SdfTree<D>) {
        let node_idx = tree.nodes.len() as u32;
        let data_idx = tree.data.len() as u32;
        match self {
            Self::Union(a, b) => {
                tree.nodes.push(TreeNode::no_data(AnySdf::Union));
                a.add_to_tree(tree);
                tree.nodes[node_idx as usize].value = tree.nodes.len() as u32 - node_idx - 1;
                b.add_to_tree(tree);
            }
            Self::Subtract(a, b) => {
                tree.nodes.push(TreeNode::no_data(AnySdf::Subtract));
                a.add_to_tree(tree);
                tree.nodes[node_idx as usize].value = tree.nodes.len() as u32 - node_idx - 1;
                b.add_to_tree(tree);
            }
            Self::Intersect(a, b) => {
                tree.nodes.push(TreeNode::no_data(AnySdf::Intersect));
                a.add_to_tree(tree);
                tree.nodes[node_idx as usize].value = tree.nodes.len() as u32 - node_idx - 1;
                b.add_to_tree(tree);
            }
            Self::Invert(next) => {
                tree.nodes.push(TreeNode::no_data(AnySdf::Invert));
                next.add_to_tree(tree);
            }
            Self::Shell(next, thickness) => {
                tree.nodes
                    .push(TreeNode::new(AnySdf::Shell, data_idx as u32));
                tree.data.push(thickness);
                next.add_to_tree(tree);
            }

            Self::Translate(next, (x, y)) => {
                tree.nodes
                    .push(TreeNode::new(AnySdf::Translate2d, data_idx as u32));
                tree.data.extend([x, y]);
                next.add_to_tree(tree);
            }
            Self::Rotate(next, angle) => {
                tree.nodes
                    .push(TreeNode::new(AnySdf::Rotate2d, data_idx as u32));
                tree.data.push(angle);
                next.add_to_tree(tree);
            }

            Self::Circle(radius) => {
                tree.nodes.push(TreeNode::new(AnySdf::Circle, data_idx));
                Circle { radius }.store(&mut tree.data);
            }
            Self::Rectangle(x, y) => {
                tree.nodes.push(TreeNode::new(AnySdf::Rectangle, data_idx));
                Rectangle {
                    half_size: Vec2::new(x, y),
                }
                .store(&mut tree.data);
            }
            Self::Arc {
                radius,
                angle,
                thickness,
            } => {
                tree.nodes.push(TreeNode::new(AnySdf::Arc, data_idx));
                Arc {
                    radius,
                    thickness,
                    segment: angle * 0.5,
                }
                .store(&mut tree.data);
            }
        }
    }
}

#[cfg(feature = "serialize")]
#[test]
fn tree_serialize() {
    use AnySdf::*;
    let tree = SdfTree::<dim3::Dim3> {
        nodes: vec![
            TreeNode::new(Union, 1),
            TreeNode::new(Sphere, 0),
            TreeNode::new(Translate3d, 1),
            TreeNode::new(Cuboid, 4),
        ],
        data: vec![1., 0., 0., 0.2, 1., 0.5, 0.25],
        phantom: PhantomData,
    };
    let de_node = PrettySerdeTree3d::from_tree(&tree);

    let serialized = ron::ser::to_string(&de_node).unwrap();
    assert_eq!(
        "Union(Sphere(1.0),Translate(Cuboid(1.0,0.5,0.25),(0.0,0.0,0.2)))",
        serialized
    );
}

#[cfg(feature = "serialize")]
#[test]
fn tree_deserialize() {
    let de_node = ron::from_str::<PrettySerdeTree3d>("Subtract(Cuboid(0.2, 0.4, 0.5), Sphere(1.))");
    let tree = de_node.unwrap().to_tree();

    use AnySdf::*;
    assert_eq!(
        tree.nodes,
        [
            TreeNode::new(Subtract, 1),
            TreeNode::new(Cuboid, 0),
            TreeNode::new(Sphere, 3),
        ]
    );
    assert_eq!(tree.data, [0.2, 0.4, 0.5, 1.]);

    let de_node = ron::from_str::<PrettySerdeTree3d>(
        "Rotate(Extrude(Rotate(Rectangle(0.2, 0.4), 0.3), 0.5), (0., 0., 1.57079632679))",
    );
    let tree = de_node.unwrap().to_tree();

    assert_eq!(
        tree.nodes,
        [
            TreeNode::new(Rotate3d, 0),
            TreeNode::new(Extrude, 4),
            TreeNode::new(Rotate2d, 5),
            TreeNode::new(Rectangle, 6),
        ]
    );
    assert_eq!(
        tree.data,
        [0., 0., 0.70710677, 0.70710677, 0.5, 0.3, 0.2, 0.4]
    );
}

#[derive(PartialEq, Eq, Debug)]
pub struct ExecutionNode {
    exec: AnyExec,
    value: u32,
}

impl ExecutionNode {
    fn new(exec: AnyExec, value: u32) -> Self {
        Self { exec, value }
    }

    fn no_data(exec: AnyExec) -> Self {
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
enum AnyExec {
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

#[derive(Debug, Default, Deref)]
pub struct ExecutionOrder(Vec<ExecutionNode>);

#[cfg(feature = "bevy_asset")]
#[derive(Resource)]
struct ProcessedSdfs<D> {
    generations: Vec<u32>,
    items: Vec<ExecutionOrder>,
    phantom: PhantomData<D>,
}

#[derive(Event)]
pub struct SdfProcessed(pub AssetIndex);

#[cfg(feature = "bevy_asset")]
impl<D> ProcessedSdfs<D> {
    fn index_and_generation(index: AssetIndex) -> (usize, u32) {
        let index = index.to_bits();
        let generation = (index >> 32) as u32;
        let index = (index & ((1 << 32) - 1)) as usize;
        (index, generation)
    }

    fn insert(&mut self, index: AssetIndex, order: ExecutionOrder) {
        let (index, generation) = Self::index_and_generation(index);
        if let Some(&cur_gen) = self.generations.get(index) {
            if cur_gen > generation {
                return;
            }
        }
        while self.generations.len() <= index {
            self.generations.push(0);
            self.items.push(ExecutionOrder(Vec::new()));
        }
        self.generations[index] = generation;
        self.items[index] = order;
    }

    fn get(&self, index: AssetIndex) -> Option<&ExecutionOrder> {
        let (index, generation) = Self::index_and_generation(index);
        if self.generations.get(index).copied() != Some(generation) {
            return None;
        }
        self.items.get(index)
    }

    fn remove(&mut self, index: AssetIndex) {
        let (index, generation) = Self::index_and_generation(index);
        let cur_gen = self.generations.get(index).copied();
        if cur_gen != Some(generation) {
            return;
        }
        self.generations[index] = 0;
        self.items[index] = ExecutionOrder::default();
    }
}

#[cfg(feature = "bevy_asset")]
impl<D: Dim> Default for ProcessedSdfs<D> {
    fn default() -> Self {
        Self {
            generations: default(),
            items: default(),
            phantom: PhantomData,
        }
    }
}

#[cfg(feature = "bevy_asset")]
fn process_sdf_trees<D: Dim>(
    mut commands: Commands,
    sdfs: Res<Assets<SdfTree<D>>>,
    mut events: MessageReader<AssetEvent<SdfTree<D>>>,
    mut processed: ResMut<ProcessedSdfs<D>>,
) {
    for event in events.read() {
        match event {
            &AssetEvent::Added { id } | &AssetEvent::Modified { id } => {
                let AssetId::Index { index, .. } = id else {
                    continue;
                };

                let sdf = sdfs.get(id).unwrap();
                let mut order = ExecutionOrder::default();
                sdf.get_execution_order(&mut order);
                processed.insert(index, order);
                commands.trigger(SdfProcessed(index));
            }
            &AssetEvent::Removed { id } => {
                let AssetId::Index { index, .. } = id else {
                    continue;
                };
                processed.remove(index);
            }
            _ => {}
        }
    }
}

#[cfg(feature = "bevy_asset")]
#[derive(SystemParam)]
pub struct ExecutableSdfs<'w, D: Dim> {
    assets: Res<'w, Assets<SdfTree<D>>>,
    processed: Res<'w, ProcessedSdfs<D>>,
}

pub struct ExecutableSdf<'a, D: Dim> {
    pub order: &'a ExecutionOrder,
    pub data: &'a [f32],
    phantom: PhantomData<D>,
}

impl ExecutableSdf<'_, dim3::Dim3> {
    pub fn distance(&self, p: Vec3) -> f32 {
        self.order.distance(p, self.data)
    }

    pub fn gradient(&self, p: Vec3) -> Vec3 {
        self.order.gradient(p, self.data)
    }

    pub fn aabb(&self, iso: Isometry3d) -> Aabb3d {
        self.order.aabb(iso, self.data)
    }
}

pub type ExecutableSdf3d<'a> = ExecutableSdf<'a, dim3::Dim3>;

#[cfg(feature = "bevy_asset")]
impl<'w, D: Dim> ExecutableSdfs<'w, D> {
    fn index_usize(index: AssetIndex) -> usize {
        (index.to_bits() & ((1 << 32) - 1)) as usize
    }

    pub fn len(&self) -> usize {
        self.assets.len().min(self.processed.items.len())
    }

    pub fn is_empty(&self) -> bool {
        self.assets.is_empty() || self.processed.items.is_empty()
    }

    pub fn get(&self, id: AssetId<SdfTree<D>>) -> Option<(usize, ExecutableSdf<D>)> {
        let AssetId::Index { index, .. } = id else {
            return None;
        };
        self.processed.get(index).and_then(|order| {
            self.assets.get(id).map(|sdf| {
                (
                    Self::index_usize(index),
                    ExecutableSdf {
                        order,
                        data: sdf.data(),
                        phantom: PhantomData,
                    },
                )
            })
        })
    }

    pub fn iter(&self) -> impl Iterator<Item = (usize, ExecutableSdf<D>)> {
        self.assets.iter().filter_map(|(id, sdf)| {
            let AssetId::Index { index, .. } = id else {
                return None;
            };
            self.processed.get(index).map(|order| {
                (
                    Self::index_usize(index),
                    ExecutableSdf {
                        order,
                        data: sdf.data(),
                        phantom: PhantomData,
                    },
                )
            })
        })
    }
}

enum StackItem {
    Index(usize),
    Node(ExecutionNode),
}

impl<D: Dim> SdfTree<D> {
    pub fn new_execution_order(&self) -> ExecutionOrder {
        let mut order = ExecutionOrder(Vec::with_capacity(10));
        self.get_execution_order(&mut order);

        order
    }

    pub fn get_execution_order(&self, out: &mut ExecutionOrder) {
        out.0.clear();
        if self.nodes.is_empty() {
            return;
        }

        let mut stack = Stack::<StackItem>::new();
        stack.push(StackItem::Index(0));

        use StackItem::*;
        while let Some(item) = stack.pop() {
            let index = match item {
                Index(index) => index,
                Node(n) => {
                    out.0.push(n);
                    continue;
                }
            };
            let next = index + 1;

            let node = self.nodes[index];
            let node = match node.sdf {
                AnySdf::Union => {
                    stack.extend([
                        Node(ExecutionNode::no_data(AnyExec::Union)),
                        Index(next + node.value as usize),
                        Node(ExecutionNode::no_data(AnyExec::PushStack)),
                        Index(next),
                    ]);
                    continue;
                }
                AnySdf::Subtract => {
                    stack.extend([
                        Node(ExecutionNode::no_data(AnyExec::Subtract)),
                        Index(next + node.value as usize),
                        Node(ExecutionNode::no_data(AnyExec::PushStack)),
                        Index(next),
                    ]);
                    continue;
                }
                AnySdf::Intersect => {
                    stack.extend([
                        Node(ExecutionNode::no_data(AnyExec::Intersect)),
                        Index(next + node.value as usize),
                        Node(ExecutionNode::no_data(AnyExec::PushStack)),
                        Index(next),
                    ]);
                    continue;
                }
                AnySdf::Invert => {
                    stack.extend([Node(ExecutionNode::no_data(AnyExec::Invert)), Index(next)]);
                    continue;
                }
                AnySdf::Shell => {
                    stack.extend([
                        Node(ExecutionNode::new(AnyExec::Shell, node.value)),
                        Index(next),
                    ]);
                    continue;
                }

                AnySdf::Translate2d => {
                    stack.extend([
                        Node(ExecutionNode::no_data(AnyExec::PopPosition)),
                        Index(next),
                    ]);
                    ExecutionNode::new(AnyExec::Translate2d, node.value)
                }
                AnySdf::Rotate2d => {
                    stack.extend([
                        Node(ExecutionNode::no_data(AnyExec::PopPosition)),
                        Index(next),
                    ]);
                    ExecutionNode::new(AnyExec::Rotate2d, node.value)
                }

                AnySdf::Translate3d => {
                    stack.extend([
                        Node(ExecutionNode::no_data(AnyExec::PopPosition)),
                        Index(next),
                    ]);
                    ExecutionNode::new(AnyExec::Translate3d, node.value)
                }
                AnySdf::Rotate3d => {
                    stack.extend([
                        Node(ExecutionNode::no_data(AnyExec::PopPosition)),
                        Index(next),
                    ]);
                    ExecutionNode::new(AnyExec::Rotate3d, node.value)
                }
                AnySdf::Extrude => {
                    stack.extend([
                        Node(ExecutionNode::new(AnyExec::Extrude, node.value)),
                        Index(next),
                    ]);
                    ExecutionNode::new(AnyExec::PreExtrude, node.value)
                }
                AnySdf::Revolve => {
                    stack.extend([
                        Node(ExecutionNode::no_data(AnyExec::PostRevolve)),
                        Index(next),
                    ]);
                    ExecutionNode::new(AnyExec::Revolve, node.value)
                }

                AnySdf::Circle => ExecutionNode::new(AnyExec::Circle, node.value),
                AnySdf::Rectangle => ExecutionNode::new(AnyExec::Rectangle, node.value),
                AnySdf::Arc => ExecutionNode::new(AnyExec::Arc, node.value),

                AnySdf::Sphere => ExecutionNode::new(AnyExec::Sphere, node.value),
                AnySdf::Capsule3d => ExecutionNode::new(AnyExec::Capsule, node.value),
                AnySdf::Cylinder => ExecutionNode::new(AnyExec::Cylinder, node.value),
                AnySdf::Cuboid => ExecutionNode::new(AnyExec::Cuboid, node.value),
                AnySdf::InfinitePlane3d => ExecutionNode::new(AnyExec::InfinitePlane3d, node.value),
            };

            out.0.push(node);
        }
    }
}

#[test]
fn tree_iter() {
    let tree = SdfTree::<dim3::Dim3> {
        nodes: vec![
            TreeNode::new(AnySdf::Union, 1),
            TreeNode::new(AnySdf::Sphere, 0),
            TreeNode::new(AnySdf::Translate3d, 1),
            TreeNode::new(AnySdf::Cuboid, 4),
        ],
        data: vec![1., 0., 0., 0.2, 1., 1., 0.5],
        phantom: PhantomData,
    };

    use AnyExec::*;
    let actual = tree.new_execution_order().0;
    let expected = [
        ExecutionNode::new(Sphere, 0),
        ExecutionNode::no_data(PushStack),
        ExecutionNode::new(Translate3d, 1),
        ExecutionNode::new(Cuboid, 4),
        ExecutionNode::no_data(PopPosition),
        ExecutionNode::no_data(Union),
    ];
    assert_eq!(actual, expected);
}

use bevy::math::Vec3A;

#[test]
fn tree_distance() {
    use AnySdf::*;
    let tree = SdfTree::<dim3::Dim3> {
        nodes: vec![
            TreeNode::new(Union, 2),
            TreeNode::new(Translate3d, 0),
            TreeNode::new(Sphere, 3),
            TreeNode::new(Cuboid, 4),
        ],
        data: vec![0.4, 0.4, 0.4, 1., 1., 1., 1.],
        phantom: PhantomData,
    };

    let order = tree.new_execution_order();

    assert_eq!(0.17320512, order.distance(Vec3::splat(1.1), &tree.data));

    let tree = SdfTree::<dim3::Dim3> {
        nodes: vec![
            TreeNode::new(Subtract, 1),
            TreeNode::new(Cuboid, 0),
            TreeNode::new(Sphere, 3),
        ],
        data: vec![1., 1., 1., 1.],
        phantom: PhantomData,
    };

    let order = tree.new_execution_order();

    assert_eq!(1., order.distance(Vec3::ZERO, &tree.data));
}

impl ExecutionOrder {
    pub fn distance(&self, mut pos: Vec3, slice: &[f32]) -> f32 {
        use AnyExec::*;

        let mut dist = f32::INFINITY;
        let mut dist_stack = Stack::<f32>::new();
        let mut pos_stack = Stack::<Vec3>::new();

        for node in self.0.iter() {
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
                    let circle = bevy::math::primitives::Circle::load(&slice[data..]);
                    dist = circle.distance(pos.xy());
                }
                Rectangle => {
                    let rect = bevy::math::primitives::Rectangle::load(&slice[data..]);
                    dist = rect.distance(pos.xy());
                }
                Arc => {
                    let arc = dim2::Arc::load(&slice[data..]);
                    dist = arc.distance(pos.xy());
                }

                Sphere => {
                    let sphere = bevy::math::primitives::Sphere::load(&slice[data..]);
                    dist = sphere.distance(pos);
                }
                Capsule => {
                    let capsule = bevy::math::primitives::Capsule3d::load(&slice[data..]);
                    dist = capsule.distance(pos);
                }
                Cylinder => {
                    let cylinder = bevy::math::primitives::Cylinder::load(&slice[data..]);
                    dist = cylinder.distance(pos);
                }
                Cuboid => {
                    let cuboid = bevy::math::primitives::Cuboid::load(&slice[data..]);
                    dist = cuboid.distance(pos);
                }
                InfinitePlane3d => {
                    let plane = bevy::math::primitives::InfinitePlane3d::load(&slice[data..]);
                    dist = plane.distance(pos);
                }
            }
        }
        pos_stack.clear();
        dist_stack.clear();
        dist
    }

    pub fn gradient(&self, mut pos: Vec3, slice: &[f32]) -> Vec3 {
        use AnyExec::*;

        let (mut dist, mut grad) = (0., Vec3::ZERO);
        let mut stack = Stack::<(f32, Vec3)>::new();
        let mut pos_stack = Stack::<Vec3>::new();

        for node in self.0.iter() {
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
                    use bevy::math::Vec2;

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
                    let circle = bevy::math::primitives::Circle::load(&slice[data..]);
                    let grad2d: Vec2;
                    (dist, grad2d) = circle.dist_grad(pos.xy());
                    grad = grad2d.extend(0.);
                }
                Rectangle => {
                    let rect = bevy::math::primitives::Rectangle::load(&slice[data..]);
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
                    (dist, grad) = bevy::math::primitives::Sphere { radius }.dist_grad(pos);
                }
                Capsule => {
                    let capsule = bevy::math::primitives::Capsule3d::load(&slice[data..]);
                    (dist, grad) = capsule.dist_grad(pos);
                }
                Cylinder => {
                    let cylinder = bevy::math::primitives::Cylinder::load(&slice[data..]);
                    (dist, grad) = cylinder.dist_grad(pos);
                }
                Cuboid => {
                    let cuboid = bevy::math::primitives::Cuboid::load(&slice[data..]);
                    (dist, grad) = cuboid.dist_grad(pos);
                }
                InfinitePlane3d => {
                    let plane = bevy::math::primitives::InfinitePlane3d::load(&slice[data..]);
                    (dist, grad) = plane.dist_grad(pos);
                }
            }
        }

        grad
    }

    pub fn aabb(&self, mut iso: Isometry3d, slice: &[f32]) -> Aabb3d {
        use AnyExec::*;

        let mut aabb = Aabb3d::new(Vec3A::ZERO, Vec3A::ZERO);
        let mut aabb_stack = Stack::<Aabb3d>::new();
        let mut pos_stack = Stack::<Isometry3d>::new();

        for node in self.0.iter() {
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
                    let aabb2d = bevy::math::primitives::Circle::load(&slice[data..]).aabb_2d(iso);
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
                        bevy::math::primitives::Rectangle::load(&slice[data..]).aabb_2d(iso);
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
                    aabb = bevy::math::primitives::Sphere::load(&slice[data..]).aabb_3d(iso);
                }
                Capsule => {
                    aabb = bevy::math::primitives::Capsule3d::load(&slice[data..]).aabb_3d(iso);
                }
                Cylinder => {
                    aabb = bevy::math::primitives::Cylinder::load(&slice[data..]).aabb_3d(iso);
                }
                Cuboid => {
                    aabb = bevy::math::primitives::Cuboid::load(&slice[data..]).aabb_3d(iso);
                }
                InfinitePlane3d => {
                    aabb =
                        bevy::math::primitives::InfinitePlane3d::load(&slice[data..]).aabb_3d(iso);
                }
            }
        }
        pos_stack.clear();
        aabb_stack.clear();
        aabb
    }
}

impl SdfBounding<dim3::Dim3> for (&ExecutionOrder, &[f32]) {
    fn aabb(&self, iso: Isometry3d) -> Aabb3d {
        self.0.aabb(iso, self.1)
    }

    fn bounding_ball(&self, iso: Isometry3d) -> BoundingSphere {
        _ = iso;
        todo!()
    }
}

#[test]
fn tree_aabb() {
    use AnySdf::*;
    const PI_4: f32 = core::f32::consts::PI / 4.;
    let [x, y, z, w] = Quat::from_rotation_x(PI_4).to_array();
    let tree = SdfTree::<dim3::Dim3> {
        nodes: vec![
            TreeNode::new(Union, 2),
            TreeNode::new(Rotate3d, 0),
            TreeNode::new(Cuboid, 4),
            TreeNode::new(Translate3d, 7),
            TreeNode::new(Sphere, 10),
        ],
        data: vec![x, y, z, w, 1., 1., 1., 1., 0., 0., 1.],
        phantom: PhantomData,
    };

    let sqrt2 = 2f32.sqrt();
    let order = tree.new_execution_order();
    let actual = order.aabb(Isometry3d::default(), &tree.data);
    let expected = Aabb3d {
        min: Vec3A::new(-1., -sqrt2, -sqrt2),
        max: Vec3A::new(2., sqrt2, sqrt2),
    };
    assert_eq!((expected.min, expected.max), (actual.min, actual.max));

    let actual = (&order, tree.data.as_slice()).aabb(Isometry3d::default());
    assert_eq!((expected.min, expected.max), (actual.min, actual.max));
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
            translation: self.translation
                + self.rotation.inverse() * bevy::math::Vec3A::from(trans),
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
