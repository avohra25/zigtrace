//! zigtrace library: a physically based Monte Carlo path tracer.
pub const Vec3 = @import("vec.zig").Vec3;
pub const Rng = @import("rng.zig").Rng;
pub const Ray = @import("ray.zig").Ray;
pub const Aabb = @import("aabb.zig").Aabb;
pub const geometry = @import("geometry.zig");
pub const Material = @import("material.zig").Material;
pub const material = @import("material.zig");
pub const image = @import("image.zig");

test {
    @import("std").testing.refAllDecls(@This());
}
