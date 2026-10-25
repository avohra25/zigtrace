const std = @import("std");
const Vec3 = @import("vec.zig").Vec3;

pub const Ray = struct {
    origin: Vec3,
    dir: Vec3,

    pub fn init(origin: Vec3, dir: Vec3) Ray {
        return .{ .origin = origin, .dir = dir };
    }

    pub fn at(self: Ray, t: f64) Vec3 {
        return self.origin.add(self.dir.scale(t));
    }
};

test "point along ray" {
    const r = Ray.init(Vec3.init(1, 2, 3), Vec3.init(0, 0, -2));
    try std.testing.expectEqual(Vec3.init(1, 2, -1), r.at(2));
}
