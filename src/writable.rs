use bevy_math::{Quat, Vec2, Vec3};

pub trait Writable {
    fn as_bytes(&self, w: &mut Vec<u8>);
}

impl Writable for u32 {
    fn as_bytes(&self, w: &mut Vec<u8>) {
        w.extend(self.to_le_bytes())
    }
}

impl Writable for f32 {
    fn as_bytes(&self, w: &mut Vec<u8>) {
        w.extend(self.to_le_bytes())
    }
}

impl Writable for Vec2 {
    fn as_bytes(&self, w: &mut Vec<u8>) {
        w.extend(self.x.to_le_bytes());
        w.extend(self.y.to_le_bytes());
    }
}

impl Writable for Vec3 {
    fn as_bytes(&self, w: &mut Vec<u8>) {
        w.extend(self.x.to_le_bytes());
        w.extend(self.y.to_le_bytes());
        w.extend(self.z.to_le_bytes());
    }
}

impl Writable for Quat {
    fn as_bytes(&self, w: &mut Vec<u8>) {
        w.extend(self.x.to_le_bytes());
        w.extend(self.y.to_le_bytes());
        w.extend(self.z.to_le_bytes());
        w.extend(self.w.to_le_bytes());
    }
}

impl Writable for crate::Index {
    fn as_bytes(&self, w: &mut Vec<u8>) {
        let t = self.0 >> 7;
        let v = (t as u32) << 31 + (t & 0b0111_1111) as u32;
        w.extend(v.to_le_bytes());
    }
}
