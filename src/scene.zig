//! Scene container: materials, shapes, camera, background and the BVH.

const std = @import("std");
const Vec3 = @import("vec.zig").Vec3;
const Ray = @import("ray.zig").Ray;
const geometry = @import("geometry.zig");
const Shape = geometry.Shape;
const Hit = geometry.Hit;
const MaterialId = geometry.MaterialId;
const Material = @import("material.zig").Material;
const Bvh = @import("bvh.zig").Bvh;
const CameraConfig = @import("camera.zig").CameraConfig;

pub const Background = union(enum) {
    solid: Vec3,
    /// Vertical gradient from `horizon` (and below) to `zenith`.
    sky: struct { horizon: Vec3, zenith: Vec3 },

    pub fn radiance(self: Background, dir: Vec3) Vec3 {
        return switch (self) {
            .solid => |c| c,
            .sky => |s| blk: {
                const t = std.math.clamp(dir.normalize().y, 0.0, 1.0);
                break :blk s.horizon.lerp(s.zenith, t);
            },
        };
    }
};

/// Render settings a scene may specify; the CLI can override them.
pub const Settings = struct {
    width: usize = 400,
    height: usize = 225,
    spp: usize = 32,
    max_depth: usize = 16,
};

pub const Scene = struct {
    allocator: std.mem.Allocator,
    materials: std.ArrayListUnmanaged(Material) = .empty,
    /// Bounded shapes, accelerated by the BVH.
    shapes: std.ArrayListUnmanaged(Shape) = .empty,
    /// Unbounded shapes, tested linearly.
    planes: std.ArrayListUnmanaged(Shape) = .empty,
    camera: CameraConfig = .{},
    background: Background = .{ .sky = .{ .horizon = Vec3.one, .zenith = Vec3.init(0.5, 0.7, 1.0) } },
    settings: Settings = .{},
    bvh: ?Bvh = null,

    pub fn init(allocator: std.mem.Allocator) Scene {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *Scene) void {
        if (self.bvh) |*b| b.deinit(self.allocator);
        self.materials.deinit(self.allocator);
        self.shapes.deinit(self.allocator);
        self.planes.deinit(self.allocator);
        self.* = undefined;
    }

    pub fn addMaterial(self: *Scene, m: Material) !MaterialId {
        try self.materials.append(self.allocator, m);
        return @intCast(self.materials.items.len - 1);
    }

    pub fn add(self: *Scene, shape: Shape) !void {
        self.invalidate();
        if (shape.bounds() == null) {
            try self.planes.append(self.allocator, shape);
        } else {
            try self.shapes.append(self.allocator, shape);
        }
    }

    /// Adds the parallelogram `corner + a*u + b*v` (a, b in [0, 1]) as two
    /// triangles. The front face is on the side of `u x v`.
    pub fn addQuad(self: *Scene, corner: Vec3, u: Vec3, v: Vec3, mat: MaterialId) !void {
        const p1 = corner.add(u);
        const p2 = corner.add(u).add(v);
        const p3 = corner.add(v);
        try self.add(.{ .triangle = .{ .v0 = corner, .v1 = p1, .v2 = p2, .material = mat } });
        try self.add(.{ .triangle = .{ .v0 = corner, .v1 = p2, .v2 = p3, .material = mat } });
    }

    /// Builds the acceleration structure. Must be called after the last
    /// `add` and before rendering.
    pub fn build(self: *Scene) !void {
        self.invalidate();
        self.bvh = try Bvh.build(self.allocator, self.shapes.items);
    }

    fn invalidate(self: *Scene) void {
        if (self.bvh) |*b| b.deinit(self.allocator);
        self.bvh = null;
    }

    pub fn primitiveCount(self: Scene) usize {
        return self.shapes.items.len + self.planes.items.len;
    }

    pub fn intersect(self: *const Scene, r: Ray, t_min: f64, t_max: f64) ?Hit {
        var best = if (self.bvh) |b|
            b.intersect(self.shapes.items, r, t_min, t_max)
        else
            geometry.intersectAll(self.shapes.items, r, t_min, t_max);
        const limit = if (best) |h| h.t else t_max;
        if (geometry.intersectAll(self.planes.items, r, t_min, limit)) |h| best = h;
        return best;
    }

    pub fn material(self: *const Scene, id: MaterialId) Material {
        return self.materials.items[id];
    }
};

const testing = std.testing;

test "scene routes planes outside the BVH and finds closest hit" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    const m = try scene.addMaterial(.{ .lambertian = .{ .albedo = Vec3.one } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 0, -5), .radius = 1, .material = m } });
    try scene.add(.{ .plane = .{ .point = Vec3.init(0, 0, -3), .normal = Vec3.init(0, 0, 1), .material = m } });
    try scene.addQuad(Vec3.init(-1, -1, -2), Vec3.init(2, 0, 0), Vec3.init(0, 0.5, 0), m);
    try scene.build();
    try testing.expectEqual(@as(usize, 3), scene.shapes.items.len);
    try testing.expectEqual(@as(usize, 1), scene.planes.items.len);

    const down_axis = Ray.init(Vec3.zero, Vec3.init(0, 0, -1));
    try testing.expectApproxEqAbs(@as(f64, 3), scene.intersect(down_axis, 1e-4, 1e9).?.t, 1e-12);
    const through_quad = Ray.init(Vec3.init(0, -0.75, 0), Vec3.init(0, 0, -1));
    const hq = scene.intersect(through_quad, 1e-4, 1e9).?;
    try testing.expectApproxEqAbs(@as(f64, 2), hq.t, 1e-12);
    try testing.expect(hq.front_face);
}

test "sky background gradient" {
    const bg = Background{ .sky = .{ .horizon = Vec3.one, .zenith = Vec3.zero } };
    try testing.expectEqual(Vec3.one, bg.radiance(Vec3.init(1, 0, 0)));
    try testing.expectEqual(Vec3.zero, bg.radiance(Vec3.init(0, 3, 0)));
    try testing.expectEqual(Vec3.one, bg.radiance(Vec3.init(0, -1, 0)));
}
