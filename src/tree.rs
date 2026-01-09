use core::marker::PhantomData;

#[cfg(feature = "bevy")]
use bevy::prelude::*;

#[cfg(test)]
use super::ExecutableExt;
use super::{
    AnyExec, AnySdf, Dim, ExecutionNode, ExecutionOrder, Stack, StorablePrimitive, ToNode,
    TreeNode, dim2::Dim2, dim3::Dim3,
};

extern crate alloc;

#[cfg_attr(feature = "bevy", derive(Reflect))]
#[derive(Clone, Debug)]
#[cfg_attr(feature = "bevy_asset", derive(Asset))]
pub struct SdfTree<D: Dim> {
    pub(super) nodes: Vec<TreeNode>,
    pub(super) data: Vec<f32>,
    #[reflect(ignore)]
    pub(super) phantom: PhantomData<D>,
}

impl<D: Dim> SdfTree<D> {
    pub(super) fn add_primitive<P: StorablePrimitive + ToNode>(&mut self, p: P) {
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

impl<P: StorablePrimitive + ToNode<D = Dim2>> From<(P, f32)> for SdfTree<Dim3> {
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

impl<D: Dim> SdfTree<D> {
    pub(super) fn verify(&self) {
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
        use StackItem::*;

        out.0.clear();
        if self.nodes.is_empty() {
            return;
        }

        let mut stack = Stack::<StackItem>::new();
        stack.push(StackItem::Index(0));

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
                AnySdf::Torus => ExecutionNode::new(AnyExec::Torus, node.value),
                AnySdf::Cuboid => ExecutionNode::new(AnyExec::Cuboid, node.value),
                AnySdf::InfinitePlane3d => ExecutionNode::new(AnyExec::InfinitePlane3d, node.value),
            };

            out.0.push(node);
        }
    }
}

#[test]
fn tree_iter() {
    let tree = SdfTree::<Dim3> {
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

#[test]
fn tree_distance() {
    use AnySdf::*;
    let tree = SdfTree::<Dim3> {
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

    let tree = SdfTree::<Dim3> {
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

#[test]
fn tree_aabb() {
    use crate::SdfBounding;
    use AnySdf::*;

    const PI_4: f32 = core::f32::consts::PI / 4.;
    let [x, y, z, w] = Quat::from_rotation_x(PI_4).to_array();
    let tree = SdfTree::<Dim3> {
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
    let expected = bevy_math::bounding::Aabb3d {
        min: Vec3A::new(-1., -sqrt2, -sqrt2),
        max: Vec3A::new(2., sqrt2, sqrt2),
    };
    assert_eq!((expected.min, expected.max), (actual.min, actual.max));

    let actual = (&*order, tree.data.as_slice()).aabb(Isometry3d::default());
    assert_eq!((expected.min, expected.max), (actual.min, actual.max));
}
