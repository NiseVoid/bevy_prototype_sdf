#[cfg(feature = "bevy_asset")]
use core::marker::PhantomData;

#[cfg(feature = "bevy_asset")]
use super::{Dim, ExecutableExt, ExecutionOrder, dim2::Dim2, dim3::Dim3, tree::SdfTree};
use super::{Sdf2d, Sdf3d};

use bevy::prelude::*;
#[cfg(feature = "bevy_asset")]
use bevy::{
    asset::{AssetEventSystems, AssetIndex, AssetLoader, LoadContext, io::Reader},
    ecs::system::SystemParam,
};
use bevy_math::bounding::Aabb3d;
#[cfg(feature = "bevy_asset")]
use thiserror::Error;

pub struct SdfPlugin;

impl Plugin for SdfPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<Sdf2d>().register_type::<Sdf3d>();

        #[cfg(feature = "bevy_asset")]
        app.init_asset::<Sdf3d>()
            .init_asset::<Sdf2d>()
            .init_asset_loader::<SdfAssetLoader<Dim3>>()
            .init_resource::<ProcessedSdfs<Dim2>>()
            .init_resource::<ProcessedSdfs<Dim3>>()
            .add_systems(
                PostUpdate,
                (process_sdf_trees::<Dim2>, process_sdf_trees::<Dim3>)
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

/// A system set that handles processing of SDFs, runs in Last
#[cfg(feature = "bevy_asset")]
#[derive(SystemSet, Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct SdfProcessing;

#[cfg(feature = "bevy_asset")]
#[derive(TypePath)]
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
impl AssetLoader for SdfAssetLoader<Dim3> {
    type Asset = SdfTree<Dim3>;
    type Settings = ();
    type Error = SdfAssetLoaderError;
    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &(),
        _load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        use crate::readable_serde::PrettySerdeTree3d;

        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        let serde_tree = ron::de::from_bytes::<PrettySerdeTree3d>(&bytes)?;
        Ok(serde_tree.to_tree())
    }

    fn extensions(&self) -> &[&str] {
        &["sdf3d"]
    }
}

#[cfg(feature = "bevy_asset")]
#[derive(Resource)]
struct ProcessedSdfs<D> {
    generations: Vec<u32>,
    items: Vec<ExecutionOrder>,
    phantom: PhantomData<D>,
}

#[cfg(feature = "bevy_asset")]
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

#[cfg(feature = "bevy_asset")]
pub struct ExecutableSdf<'a, D: Dim> {
    pub order: &'a ExecutionOrder,
    pub data: &'a [f32],
    phantom: PhantomData<D>,
}

#[cfg(feature = "bevy_asset")]
impl ExecutableSdf<'_, Dim3> {
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

#[cfg(feature = "bevy_asset")]
pub type ExecutableSdf3d<'a> = ExecutableSdf<'a, Dim3>;

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

    pub fn get<'a>(&'a self, id: AssetId<SdfTree<D>>) -> Option<(usize, ExecutableSdf<'a, D>)> {
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

    pub fn iter<'a>(&'a self) -> impl Iterator<Item = (usize, ExecutableSdf<'a, D>)> {
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
