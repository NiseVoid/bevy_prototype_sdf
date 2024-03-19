use std::ops::Deref;

use crate::{to_buffer::SdfBuffered, Dim, SdfShape, SdfTree};
use bevy_ecs::system::{lifetimeless::SRes, SystemParamItem};
use bevy_render::{
    render_asset::{RenderAsset, RenderAssetUsages},
    render_resource::{Buffer, BufferInitDescriptor, BufferUsages},
    renderer::RenderDevice,
};

pub struct GpuSdfTree(pub Buffer);

impl Deref for GpuSdfTree {
    type Target = Buffer;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<D: Dim, S: SdfShape<D>> RenderAsset for SdfTree<D, S> {
    type PreparedAsset = GpuSdfTree;
    type Param = SRes<RenderDevice>;

    fn asset_usage(&self) -> RenderAssetUsages {
        RenderAssetUsages::default()
    }

    fn prepare_asset(
        self,
        render_device: &mut SystemParamItem<Self::Param>,
    ) -> Result<Self::PreparedAsset, bevy_render::render_asset::PrepareAssetError<Self>> {
        let data = self.to_gpu_bytes();

        let data_buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            usage: BufferUsages::STORAGE,
            label: Some("SDF Tree Buffer"),
            contents: &data,
        });
        Ok(GpuSdfTree(data_buffer))
    }
}

impl<D: Dim, S: SdfShape<D>> SdfTree<D, S> {
    pub fn to_gpu_bytes(&self) -> Vec<u8> {
        let mut data = Vec::with_capacity(8 + (self.operations.len() + self.shapes.len()) * 8);
        let mut size = (self.operations.len() + self.shapes.len()) * 2;

        data.extend((self.operations.len() as u32).to_le_bytes());
        data.extend((self.shapes.len() as u32).to_le_bytes());

        for op in self.operations.iter() {
            data.extend(op.id().to_le_bytes());
            data.extend((size as u32).to_le_bytes());

            size += op.n_values();
        }

        for shape in self.shapes.iter() {
            data.extend(shape.id().to_le_bytes());
            data.extend((size as u32).to_le_bytes());

            size += shape.n_values();
        }

        data.reserve_exact(size);

        for operation in self.operations.iter() {
            operation.write_values(&mut data);
        }
        for shape in self.shapes.iter() {
            shape.write_values(&mut data);
        }

        data
    }
}
