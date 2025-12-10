use core::marker::PhantomData;

use bevy::prelude::*;

#[cfg(test)]
use crate::{AnySdf, TreeNode, dim3::Dim3};
use crate::{Dim, tree::SdfTree};

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

#[test]
fn test_raw_serde() {
    use AnySdf::*;

    let tree = SdfTree::<Dim3> {
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
    let de: RawSerdeTree<Dim3> = ron::de::from_str(&serialized).unwrap();
    assert_eq!(de.0, raw.0);

    // Test with postcard
    let serialized = postcard::to_stdvec(&raw).unwrap();
    let de: RawSerdeTree<Dim3> = postcard::from_bytes(&serialized).unwrap();
    assert_eq!(de.0, raw.0);
}

#[test]
#[should_panic]
fn test_invalid_tree() {
    let tree = SdfTree::<Dim3> {
        nodes: vec![TreeNode::new(AnySdf::Union, 1)],
        data: vec![],
        phantom: PhantomData,
    };
    let raw = RawSerdeTree(tree);

    let serialized = postcard::to_stdvec(&raw).unwrap();
    let _: RawSerdeTree<Dim3> = postcard::from_bytes(&serialized).unwrap();
}
