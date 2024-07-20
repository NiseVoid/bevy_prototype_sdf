#define_import_path bevy_prototype_sdf

@group(2) @binding(0) var<storage, read> shape_data: array<u32>;

fn sdf(pos: vec3<f32>, start: u32) -> f32 {
    if shape_data[start] != 0 || shape_data[start+1] != 1 {
        return 1e9;
    }

    let shape_kind = shape_data[start+2];
    let offset = start + 4;

    if shape_kind == 0 { // Sphere
        return sd_sphere(pos, bitcast<f32>(shape_data[offset]));
    } else if shape_kind == 1 { // Capsule
        return 1e9; // TODO
    } else if shape_kind == 2 { // Cylinder
        return sd_cylinder(pos, bitcast<f32>(shape_data[offset]), bitcast<f32>(shape_data[offset+1]));
    } else if shape_kind == 3 { // Cuboid
        let x = bitcast<f32>(shape_data[offset]);
        let y = bitcast<f32>(shape_data[offset + 1]);
        let z = bitcast<f32>(shape_data[offset + 2]);
        return sd_cuboid(pos, vec3<f32>(x, y, z));
    } else if shape_kind == 4 { // Infinite plane
        let x = bitcast<f32>(shape_data[offset]);
        let y = bitcast<f32>(shape_data[offset + 1]);
        let z = bitcast<f32>(shape_data[offset + 2]);
        return dot(pos, vec3<f32>(x, y, z));
    } else if shape_kind == 5 { // Extruded
        let half_height = bitcast<f32>(shape_data[offset]);
        let extruded_kind = shape_data[offset+1];
        return sd_extrude(pos, half_height, extruded_kind, offset+2);
    }
    return 1e9;
}

fn sd_sphere(pos: vec3<f32>, radius: f32) -> f32 {
    return length(pos) - radius;
}

fn sd_cylinder(pos: vec3<f32>, half_height: f32, radius: f32) -> f32 {
    let d = abs(vec2<f32>(length(pos.xz), pos.y)) - vec2<f32>(radius, half_height);
    return min(max(d.x, d.y), 0.0) + length(max(d, vec2<f32>(0.)));
}

fn sd_cuboid(pos: vec3<f32>, bounds: vec3<f32>) -> f32 {
    let q = abs(pos) - bounds;
    return length(max(q, vec3<f32>(0.))) + min(max(q.x, max(q.y, q.z)), 0.);
}

fn sd_extrude(pos: vec3<f32>, half_height: f32, shape_kind: u32, offset: u32) -> f32 {
    let d = sdf2d(pos.xz, shape_kind, offset);
    let w = vec2<f32>(d, abs(pos.y) - half_height);

    return min(max(w.x, w.y), 0.) + length(max(w, vec2<f32>(0.)));
}

fn sdf2d(pos: vec2<f32>, shape_kind: u32, offset: u32) -> f32 {
    if shape_kind == 3 {
        let radius = bitcast<f32>(shape_data[offset]);
        let thickness = bitcast<f32>(shape_data[offset+1]);
        let segment = bitcast<f32>(shape_data[offset+2]);
        return sd_arc(pos, radius, thickness, segment);
    }

    return 1e9;
}

fn sd_arc(pos: vec2<f32>, radius: f32, thickness: f32, segment: f32) -> f32 {
    let sc = vec2<f32>(sin(segment), cos(segment));
    let p = vec2<f32>(abs(pos.x), pos.y);
    if sc.y * p.x > sc.x * p.y {
        let w = p - radius * sc;
        let l = length(w);
        return l - thickness;
    } else {
        let l = length(pos);
        let w = l - radius;
        return abs(w) - thickness;
    }
}
