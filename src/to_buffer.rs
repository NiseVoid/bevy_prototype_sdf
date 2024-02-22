use crate::{writable::Writable, Dim, Sdf3dShape, SdfOperation};

pub trait ValueWriter {
    fn write(&mut self, v: impl Writable);
}

impl ValueWriter for Vec<u8> {
    fn write(&mut self, v: impl Writable) {
        v.as_bytes(self);
    }
}

pub trait SdfBuffered {
    fn id(&self) -> u32;
    fn n_values(&self) -> usize;
    fn write_values(&self, w: &mut impl ValueWriter);
}

impl<D: Dim> SdfBuffered for SdfOperation<D> {
    fn id(&self) -> u32 {
        use SdfOperation::*;
        match self {
            Union(_, _) => 0,
            Invert(_) => 1,
            Translate(_, _) => 2,
            Rotate(_, _) => 3,
        }
    }

    fn n_values(&self) -> usize {
        use SdfOperation::*;
        match self {
            Union(_, _) => 2,
            Invert(_) => 1,
            Translate(_, _) => 1 + D::POS_SIZE,
            Rotate(_, _) => 1 + D::ROT_SIZE,
        }
    }

    fn write_values(&self, w: &mut impl ValueWriter) {
        use SdfOperation::*;
        match *self {
            Union(a, b) => {
                w.write(a);
                w.write(b);
            }
            Invert(a) => {
                w.write(a);
            }
            Translate(a, translation) => {
                w.write(a);
                w.write(translation);
            }
            Rotate(a, rotation) => {
                w.write(a);
                w.write(rotation);
            }
        }
    }
}

impl SdfBuffered for Sdf3dShape {
    fn id(&self) -> u32 {
        use Sdf3dShape::*;
        match self {
            Sphere(_) => 0,
            Capsule(_) => 1,
            Cylinder(_) => 2,
            Cuboid(_) => 3,
            Extruded(_) => 4,
        }
    }

    fn n_values(&self) -> usize {
        use Sdf3dShape::*;
        match self {
            Sphere(_) => 1,
            Capsule(_) => 2,
            Cylinder(_) => 2,
            Cuboid(_) => 3,
            Extruded(_) => todo!(),
        }
    }

    fn write_values(&self, w: &mut impl ValueWriter) {
        use Sdf3dShape::*;
        match *self {
            Sphere(s) => {
                w.write(s.radius);
            }
            Capsule(c) => {
                w.write(c.half_length);
                w.write(c.radius);
            }
            Cylinder(c) => {
                w.write(c.half_height);
                w.write(c.radius);
            }
            Cuboid(b) => {
                w.write(b.half_size);
            }
            Extruded(_) => todo!(),
        }
    }
}
