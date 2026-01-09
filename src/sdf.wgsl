#define_import_path bevy_prototype_sdf

@group(2) @binding(0) var<storage, read> sdf_order: array<u32>;
@group(2) @binding(1) var<storage, read> sdf_data: array<f32>;

// Universal operations
const EXEC_PUSH_STACK = 0u;
const EXEC_POP_POSITION = 1u;
const EXEC_UNION = 2u;
const EXEC_SUBTRACT = 3u;
const EXEC_INTERSECT = 4u;
const EXEC_INVERT = 5u;
const EXEC_SHELL = 6u;

// 2D operations
const EXEC_TRANSLATE_2D = 7u;
const EXEC_ROTATE_2D = 8u;

// 3D operations
const EXEC_TRANSLATE_3D = 9u;
const EXEC_ROTATE_3D = 10u;
const EXEC_PRE_EXTRUDE = 11u;
const EXEC_EXTRUDE = 12u;
const EXEC_REVOLVE = 13u;
const EXEC_POST_REVOLVE = 14u;

// 2D primitives
const EXEC_CIRCLE = 15u;
const EXEC_RECTANGLE = 16u;
const EXEC_ARC = 17u;

// 3D primitives
const EXEC_SPHERE = 18u;
const EXEC_CAPSULE_3D = 19u;
const EXEC_CYLINDER = 20u;
const EXEC_TORUS = 21u;
const EXEC_CUBOID = 22u;
const EXEC_INFINITE_PLANE_3D = 23u;

fn sdf(p: vec3<f32>, order_start: u32, data_start: u32) -> f32 {
    let order_len = sdf_order[order_start];

    var dist = 1e9;
    var dist_stack: array<f32, 16>;
    var dist_stack_location = 0u;

    var pos = p;
    var pos_stack: array<vec3<f32>, 16>;
    var pos_stack_location = 0u;

    for (var i = 0u; i < order_len; i++) {
        let node_start = order_start + 1 + i * 2;
        let node_exec = sdf_order[node_start];
        let node_value = sdf_order[node_start + 1];

        // data offset if the node value is an index into data
        let offset = data_start + node_value;

        switch node_exec {
            default: {
                return sd_sphere(pos, 0.1);
            }

            // Universal operations
            case EXEC_PUSH_STACK: {
                dist_stack[dist_stack_location] = dist;
                dist_stack_location += 1u;
            }
            case EXEC_UNION: {
                dist_stack_location -= 1u;
                let a = dist_stack[dist_stack_location];
                dist = min(a, dist);
            }
            case EXEC_SUBTRACT: {
                dist_stack_location -= 1u;
                let a = dist_stack[dist_stack_location];
                dist = max(a, -dist);
            }
            case EXEC_INTERSECT: {
                dist_stack_location -= 1u;
                let a = dist_stack[dist_stack_location];
                dist = max(a, dist);
            }
            case EXEC_INVERT: {
                dist = -dist;
            }
            case EXEC_SHELL: {
                dist = abs(dist) - sdf_data[offset];
            }
            case EXEC_POP_POSITION: {
                pos_stack_location -= 1u;
                pos = pos_stack[pos_stack_location];
            }

            // 2D operations
            case EXEC_TRANSLATE_2D: {
                pos_stack[pos_stack_location] = pos;
                pos_stack_location += 1u;

                let x = sdf_data[offset];
                let y = sdf_data[offset + 1];
                pos -= vec3<f32>(x, 0., y);
            }
            case EXEC_ROTATE_2D: {
                pos_stack[pos_stack_location] = pos;
                pos_stack_location += 1u;

                let angle = sdf_data[offset];
                let s = sin(angle);
                let c = cos(angle);

                let rot = mat2x2(
                    c, -s,
                    s, c,
                );
                let pos2d = rot * pos.xz;
                pos = vec3<f32>(pos2d.x, pos.y, pos2d.y);
            }

            // 3D operations
            case EXEC_TRANSLATE_3D: {
                pos_stack[pos_stack_location] = pos;
                pos_stack_location += 1u;

                let x = sdf_data[offset];
                let y = sdf_data[offset + 1];
                let z = sdf_data[offset + 2];
                pos -= vec3<f32>(x, y, z);
            }
            case EXEC_ROTATE_3D: {
                pos_stack[pos_stack_location] = pos;
                pos_stack_location += 1u;

                let x = sdf_data[offset];
                let y = sdf_data[offset + 1];
                let z = sdf_data[offset + 2];
                let w = sdf_data[offset + 3];

                pos = qtransform(vec4<f32>(-x, -y, -z, w), pos);
            }
            case EXEC_PRE_EXTRUDE: {
                pos_stack[pos_stack_location] = pos;
                pos_stack_location += 1u;
            }
            case EXEC_EXTRUDE: {
                pos_stack_location -= 1u;
                pos = pos_stack[pos_stack_location];
                dist = sd_extrude(dist, pos, sdf_data[offset]);
            }
            case EXEC_REVOLVE: {
                pos_stack[pos_stack_location] = pos;
                pos_stack_location += 1u;

                let x_offset = sdf_data[offset];
                pos = vec3<f32>(length(pos.xz) - x_offset, 0., pos.y);
            }
            case EXEC_POST_REVOLVE: {
                pos_stack_location -= 1u;
                pos = pos_stack[pos_stack_location];
            }

            // 2D primitives
            case EXEC_CIRCLE: {
                dist = sd_circle(pos.xz, sdf_data[offset]);
            }
            case EXEC_RECTANGLE: {
                let x = sdf_data[offset];
                let y = sdf_data[offset+1];
                dist = sd_rectangle(pos.xz, vec2<f32>(x, y));
            }
            case EXEC_ARC: {
                let radius = sdf_data[offset];
                let thickness = sdf_data[offset+1];
                let segment = sdf_data[offset+2];
                dist = sd_arc(pos.xz, radius, thickness, segment);
            }

            // 3D primitives
            case EXEC_SPHERE: {
                 dist = sd_sphere(pos, sdf_data[offset]);
            }
            case EXEC_CAPSULE_3D: {
                let radius = sdf_data[offset];
                let half_length = sdf_data[offset+1];
                dist = sd_capsule3d(pos, radius, half_length);
            }
            case EXEC_CYLINDER: {
                let radius = sdf_data[offset];
                let half_height = sdf_data[offset+1];
                dist = sd_cylinder(pos, radius, half_height);
            }
            case EXEC_TORUS: {
                let major = sdf_data[offset];
                let minor = sdf_data[offset+1];
                dist = sd_torus(pos, major, minor);
            }
            case EXEC_CUBOID: {
                let x = sdf_data[offset];
                let y = sdf_data[offset + 1];
                let z = sdf_data[offset + 2];
                dist = sd_cuboid(pos, vec3<f32>(x, y, z));
            }
            case EXEC_INFINITE_PLANE_3D: {
                let x = sdf_data[offset];
                let y = sdf_data[offset + 1];
                let z = sdf_data[offset + 2];
                dist = dot(pos, vec3<f32>(x, y, z));
            }
        }
    }

    return dist;
}

fn sd_sphere(pos: vec3<f32>, radius: f32) -> f32 {
    return length(pos) - radius;
}

fn sd_capsule3d(pos: vec3<f32>, radius: f32, half_length: f32) -> f32 {
    var p = pos;
    p.y -= clamp(p.y, -half_length, half_length);
    return length(p) - radius;
}

fn sd_cylinder(pos: vec3<f32>, radius: f32, half_height: f32) -> f32 {
    let d = abs(vec2<f32>(length(pos.xz), pos.y)) - vec2<f32>(radius, half_height);
    return min(max(d.x, d.y), 0.0) + length(max(d, vec2<f32>(0.)));
}

fn sd_torus(pos: vec3f, major: f32, minor: f32) -> f32 {
    let h = length(pos.xz);
    let from_major = h - major;
    return length(vec2f(from_major, pos.y)) - minor;
}

fn sd_cuboid(pos: vec3<f32>, bounds: vec3<f32>) -> f32 {
    let q = abs(pos) - bounds;
    return length(max(q, vec3<f32>(0.))) + min(max(q.x, max(q.y, q.z)), 0.);
}

fn sd_extrude(dist: f32, pos: vec3<f32>, half_height: f32) -> f32 {
    let w = vec2<f32>(dist, abs(pos.y) - half_height);

    return min(max(w.x, w.y), 0.) + length(max(w, vec2<f32>(0.)));
}

fn sdf2d(pos: vec2<f32>, shape_kind: u32, offset: u32) -> f32 {
    if shape_kind == 3 {
        let radius = bitcast<f32>(sdf_data[offset]);
        let thickness = bitcast<f32>(sdf_data[offset+1]);
        let segment = bitcast<f32>(sdf_data[offset+2]);
        return sd_arc(pos, radius, thickness, segment);
    }

    return 1e9;
}

fn sd_circle(pos: vec2<f32>, radius: f32) -> f32 {
    return length(pos) - radius;
}

fn sd_rectangle(pos: vec2<f32>, bounds: vec2<f32>) -> f32 {
    let d = abs(pos) - bounds;
    return length(max(d, vec2<f32>(0.))) + min(max(d.x, d.y), 0.);
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

fn qtransform(q:vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    return v + 2.0*cross(cross(v, q.xyz) + q.w*v, q.xyz);
}
