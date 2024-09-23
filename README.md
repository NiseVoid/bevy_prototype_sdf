# bevy_prototype_sdf

A crate for defining and using trees of signed distance primitives and operations.

## Goals

- Provide a simple and perfomant way to define SDFs in a serializable and reusable way

## Current limitations

Not many shapes are currently supported, and the data structure is sub-optimal for serialization.
The plan is to switch to a structure that always stores the tree as if it were serialized, and read from it directly while performing operations.

## License

All code in this repository is dual-licensed under either:

* MIT License ([LICENSE-MIT](LICENSE-MIT) or [http://opensource.org/licenses/MIT](http://opensource.org/licenses/MIT))
* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0))

at your option.
