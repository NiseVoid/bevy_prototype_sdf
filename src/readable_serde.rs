use core::marker::PhantomData;

#[cfg(feature = "bevy")]
use bevy::prelude::*;

use crate::{AnySdf, StorablePrimitive, TreeNode, dim3::Dim3, tree::SdfTree};
#[cfg(feature = "serialize")]
use crate::{Dim, dim2::Dim2};

#[cfg_attr(feature = "bevy", derive(Reflect))]
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "bevy", reflect(opaque, Clone))]
#[cfg_attr(
    all(feature = "serialize", feature = "bevy"),
    reflect(Serialize, Deserialize)
)]
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
#[cfg_attr(feature = "bevy", derive(Reflect))]
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "bevy", reflect(opaque, Clone))]
#[cfg_attr(
    all(feature = "serialize", feature = "bevy"),
    reflect(Serialize, Deserialize)
)]
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
    Capsule { radius: f32, half_length: f32 },
    Cylinder { radius: f32, half_height: f32 },
    Torus { major: f32, minor: f32 },
    Cuboid(f32, f32, f32),
    InfinitePlane(f32, f32, f32),
}

impl PrettySerdeTree3d {
    pub fn from_tree(tree: &SdfTree<Dim3>) -> Self {
        Self::from_node(&tree, 0)
    }

    fn from_node(tree: &SdfTree<Dim3>, index: u32) -> Self {
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
            AnySdf::Capsule3d => {
                let capsule = Capsule3d::load(&tree.data[data..]);
                Self::Capsule {
                    radius: capsule.radius,
                    half_length: capsule.half_length,
                }
            }
            AnySdf::Cylinder => {
                let cylinder = Cylinder::load(&tree.data[data..]);
                Self::Cylinder {
                    radius: cylinder.radius,
                    half_height: cylinder.half_height,
                }
            }
            AnySdf::Torus => {
                let torus = Torus::load(&tree.data[data..]);
                Self::Torus {
                    major: torus.major_radius,
                    minor: torus.minor_radius,
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

    pub fn to_tree(self) -> SdfTree<Dim3> {
        let mut tree = SdfTree::<Dim3> {
            nodes: default(),
            data: default(),
            phantom: PhantomData,
        };
        self.add_to_tree(&mut tree);

        #[cfg(debug_assertions)]
        tree.verify();

        tree
    }

    fn add_to_tree(self, tree: &mut SdfTree<Dim3>) {
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
            Self::Capsule {
                radius,
                half_length,
            } => {
                tree.add_primitive(Capsule3d {
                    radius,
                    half_length,
                });
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
            Self::Torus { major, minor } => {
                tree.add_primitive(Torus {
                    major_radius: major,
                    minor_radius: minor,
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
    pub fn from_tree(tree: &SdfTree<Dim2>) -> Self {
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
                use crate::Arc;

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
            | AnySdf::Torus
            | AnySdf::Cuboid
            | AnySdf::InfinitePlane3d => {
                panic!("3D primitives can't be turned into a 2D tree");
            }
        }
    }

    pub fn to_tree(self) -> SdfTree<Dim2> {
        let mut tree = SdfTree::<Dim2> {
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
                use crate::Arc;

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

#[test]
fn tree_serialize() {
    use AnySdf::*;
    let tree = SdfTree::<Dim3> {
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
