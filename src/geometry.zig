//! Ray-intersectable shapes: spheres, infinite planes and triangles.

const std = @import("std");
const Vec3 = @import("vec.zig").Vec3;
const Ray = @import("ray.zig").Ray;
const Aabb = @import("aabb.zig").Aabb;

/// Index into a scene's material table.
pub const MaterialId = u32;

pub const Hit = struct {
    t: f64,
    point: Vec3,
    /// Unit shading normal, always facing against the incoming ray.
    normal: Vec3,
    /// True if the ray struck the outward-facing side of the surface.
    front_face: bool,
    material: MaterialId,

    fn make(r: Ray, t: f64, outward: Vec3, material: MaterialId) Hit {
        const front = r.dir.dot(outward) < 0;
        return .{
            .t = t,
            .point = r.at(t),
            .normal = if (front) outward else outward.neg(),
            .front_face = front,
            .material = material,
        };
    }
};

pub const Sphere = struct {
    center: Vec3,
    radius: f64,
    material: MaterialId,

    pub fn intersect(self: Sphere, r: Ray, t_min: f64, t_max: f64) ?Hit {
        const oc = self.center.sub(r.origin);
        const a = r.dir.lengthSquared();
        const h = r.dir.dot(oc);
        const c = oc.lengthSquared() - self.radius * self.radius;
        const disc = h * h - a * c;
        if (disc < 0) return null;
        const sq = @sqrt(disc);
        var t = (h - sq) / a;
        if (t <= t_min or t >= t_max) {
            t = (h + sq) / a;
            if (t <= t_min or t >= t_max) return null;
        }
        const outward = r.at(t).sub(self.center).div(self.radius);
        return Hit.make(r, t, outward, self.material);
    }

    pub fn bounds(self: Sphere) Aabb {
        const r = Vec3.splat(@abs(self.radius));
        return .{ .min = self.center.sub(r), .max = self.center.add(r) };
    }
};

/// Infinite plane through `point` with unit `normal`. Unbounded, so it is
/// tested separately from the BVH.
pub const Plane = struct {
    point: Vec3,
    normal: Vec3,
    material: MaterialId,

    pub fn intersect(self: Plane, r: Ray, t_min: f64, t_max: f64) ?Hit {
        const denom = self.normal.dot(r.dir);
        if (@abs(denom) < 1e-12) return null;
        const t = self.point.sub(r.origin).dot(self.normal) / denom;
        if (t <= t_min or t >= t_max) return null;
        return Hit.make(r, t, self.normal, self.material);
    }
};

pub const Triangle = struct {
    v0: Vec3,
    v1: Vec3,
    v2: Vec3,
    material: MaterialId,

    /// Möller–Trumbore intersection.
    pub fn intersect(self: Triangle, r: Ray, t_min: f64, t_max: f64) ?Hit {
        const e1 = self.v1.sub(self.v0);
        const e2 = self.v2.sub(self.v0);
        const p = r.dir.cross(e2);
        const det = e1.dot(p);
        if (@abs(det) < 1e-12) return null;
        const inv_det = 1.0 / det;
        const s = r.origin.sub(self.v0);
        const u = s.dot(p) * inv_det;
        if (u < 0 or u > 1) return null;
        const q = s.cross(e1);
        const v = r.dir.dot(q) * inv_det;
        if (v < 0 or u + v > 1) return null;
        const t = e2.dot(q) * inv_det;
        if (t <= t_min or t >= t_max) return null;
        return Hit.make(r, t, e1.cross(e2).normalize(), self.material);
    }

    pub fn bounds(self: Triangle) Aabb {
        return Aabb.fromPoints(self.v0, self.v1).include(self.v2).pad(1e-6);
    }
};

pub const Shape = union(enum) {
    sphere: Sphere,
    triangle: Triangle,
    plane: Plane,

    pub fn intersect(self: Shape, r: Ray, t_min: f64, t_max: f64) ?Hit {
        return switch (self) {
            inline else => |s| s.intersect(r, t_min, t_max),
        };
    }

    /// Bounding box, or null for unbounded shapes.
    pub fn bounds(self: Shape) ?Aabb {
        return switch (self) {
            .sphere => |s| s.bounds(),
            .triangle => |t| t.bounds(),
            .plane => null,
        };
    }
};

/// Closest hit by testing every shape; the reference the BVH is checked
/// against.
pub fn intersectAll(shapes: []const Shape, r: Ray, t_min: f64, t_max: f64) ?Hit {
    var closest = t_max;
    var best: ?Hit = null;
    for (shapes) |s| {
        if (s.intersect(r, t_min, closest)) |h| {
            closest = h.t;
            best = h;
        }
    }
    return best;
}

const testing = std.testing;

test "sphere hit from outside and inside" {
    const s = Sphere{ .center = Vec3.init(0, 0, -5), .radius = 1, .material = 3 };
    const r = Ray.init(Vec3.zero, Vec3.init(0, 0, -1));
    const h = s.intersect(r, 1e-4, 100).?;
    try testing.expectApproxEqAbs(@as(f64, 4), h.t, 1e-12);
    try testing.expect(h.front_face);
    try testing.expect(h.normal.approxEq(Vec3.init(0, 0, 1), 1e-12));
    try testing.expectEqual(@as(MaterialId, 3), h.material);

    const inside = Ray.init(Vec3.init(0, 0, -5), Vec3.init(0, 0, -1));
    const hi = s.intersect(inside, 1e-4, 100).?;
    try testing.expectApproxEqAbs(@as(f64, 1), hi.t, 1e-12);
    try testing.expect(!hi.front_face);
    try testing.expect(hi.normal.approxEq(Vec3.init(0, 0, 1), 1e-12));

    const miss = Ray.init(Vec3.init(0, 2, 0), Vec3.init(0, 0, -1));
    try testing.expect(s.intersect(miss, 1e-4, 100) == null);
}

test "negative radius flips the normal" {
    const s = Sphere{ .center = Vec3.zero, .radius = -1, .material = 0 };
    const h = s.intersect(Ray.init(Vec3.init(0, 0, 5), Vec3.init(0, 0, -1)), 1e-4, 100).?;
    try testing.expectApproxEqAbs(@as(f64, 4), h.t, 1e-12);
    // The outward normal points inward, so the ray is "inside" the surface.
    try testing.expect(!h.front_face);
    try testing.expectEqual(Vec3.init(-1, -1, -1), s.bounds().min);
}

test "plane hit" {
    const p = Plane{ .point = Vec3.zero, .normal = Vec3.init(0, 1, 0), .material = 0 };
    const r = Ray.init(Vec3.init(0, 2, 0), Vec3.init(1, -1, 0));
    const h = p.intersect(r, 1e-4, 100).?;
    try testing.expectApproxEqAbs(@as(f64, 2), h.t, 1e-12);
    try testing.expect(h.point.approxEq(Vec3.init(2, 0, 0), 1e-12));
    const parallel = Ray.init(Vec3.init(0, 2, 0), Vec3.init(1, 0, 0));
    try testing.expect(p.intersect(parallel, 1e-4, 100) == null);
    // Seen from below, the normal flips toward the ray.
    const below = Ray.init(Vec3.init(0, -1, 0), Vec3.init(0, 1, 0));
    const hb = p.intersect(below, 1e-4, 100).?;
    try testing.expect(!hb.front_face);
    try testing.expect(hb.normal.approxEq(Vec3.init(0, -1, 0), 0));
}

test "triangle hit and edges" {
    const tri = Triangle{
        .v0 = Vec3.init(-1, -1, -3),
        .v1 = Vec3.init(1, -1, -3),
        .v2 = Vec3.init(0, 1, -3),
        .material = 0,
    };
    const h = tri.intersect(Ray.init(Vec3.zero, Vec3.init(0, 0, -1)), 1e-4, 100).?;
    try testing.expectApproxEqAbs(@as(f64, 3), h.t, 1e-12);
    try testing.expect(h.normal.approxEq(Vec3.init(0, 0, 1), 1e-12));
    try testing.expect(tri.intersect(Ray.init(Vec3.init(0.9, 0.9, 0), Vec3.init(0, 0, -1)), 1e-4, 100) == null);
    try testing.expect(tri.intersect(Ray.init(Vec3.zero, Vec3.init(0, 0, 1)), 1e-4, 100) == null);
    const b = tri.bounds();
    try testing.expect(b.max.z - b.min.z > 0);
}

test "intersectAll picks the closest" {
    const shapes = [_]Shape{
        .{ .sphere = .{ .center = Vec3.init(0, 0, -10), .radius = 1, .material = 0 } },
        .{ .sphere = .{ .center = Vec3.init(0, 0, -4), .radius = 1, .material = 1 } },
        .{ .plane = .{ .point = Vec3.init(0, 0, -20), .normal = Vec3.init(0, 0, 1), .material = 2 } },
    };
    const h = intersectAll(&shapes, Ray.init(Vec3.zero, Vec3.init(0, 0, -1)), 1e-4, 1e9).?;
    try testing.expectEqual(@as(MaterialId, 1), h.material);
    try testing.expectEqual(@as(?Aabb, null), shapes[2].bounds());
}
