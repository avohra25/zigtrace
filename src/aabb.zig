//! Axis-aligned bounding boxes and the slab ray test used by the BVH.

const std = @import("std");
const Vec3 = @import("vec.zig").Vec3;
const Ray = @import("ray.zig").Ray;

pub const Aabb = struct {
    min: Vec3,
    max: Vec3,

    /// The identity for `merge`: contains nothing.
    pub const empty: Aabb = .{
        .min = Vec3.splat(std.math.inf(f64)),
        .max = Vec3.splat(-std.math.inf(f64)),
    };

    pub fn fromPoints(a: Vec3, b: Vec3) Aabb {
        return .{ .min = a.min(b), .max = a.max(b) };
    }

    pub fn merge(a: Aabb, b: Aabb) Aabb {
        return .{ .min = a.min.min(b.min), .max = a.max.max(b.max) };
    }

    pub fn include(a: Aabb, p: Vec3) Aabb {
        return .{ .min = a.min.min(p), .max = a.max.max(p) };
    }

    /// Grows any axis thinner than `delta` so flat shapes still get hit.
    pub fn pad(a: Aabb, delta: f64) Aabb {
        var r = a;
        inline for (.{ "x", "y", "z" }) |f| {
            if (@field(r.max, f) - @field(r.min, f) < delta) {
                @field(r.min, f) -= delta / 2;
                @field(r.max, f) += delta / 2;
            }
        }
        return r;
    }

    pub fn isEmpty(a: Aabb) bool {
        return a.min.x > a.max.x or a.min.y > a.max.y or a.min.z > a.max.z;
    }

    pub fn centroid(a: Aabb) Vec3 {
        return a.min.add(a.max).scale(0.5);
    }

    pub fn extent(a: Aabb) Vec3 {
        return a.max.sub(a.min);
    }

    pub fn surfaceArea(a: Aabb) f64 {
        if (a.isEmpty()) return 0;
        const e = a.extent();
        return 2.0 * (e.x * e.y + e.y * e.z + e.z * e.x);
    }

    pub fn longestAxis(a: Aabb) usize {
        const e = a.extent();
        if (e.x >= e.y and e.x >= e.z) return 0;
        return if (e.y >= e.z) 1 else 2;
    }

    /// Slab test. `inv_dir` is the component-wise reciprocal of the ray
    /// direction (infinities for zero components are fine). Returns the
    /// entry distance if the box overlaps [t_min, t_max].
    pub fn hit(a: Aabb, r: Ray, inv_dir: Vec3, t_min: f64, t_max: f64) ?f64 {
        var lo = t_min;
        var hi = t_max;
        inline for (.{ "x", "y", "z" }) |f| {
            const inv = @field(inv_dir, f);
            const o = @field(r.origin, f);
            var t0 = (@field(a.min, f) - o) * inv;
            var t1 = (@field(a.max, f) - o) * inv;
            if (inv < 0) std.mem.swap(f64, &t0, &t1);
            // NaN (0 * inf) compares false and leaves the interval unchanged.
            if (t0 > lo) lo = t0;
            if (t1 < hi) hi = t1;
            if (hi < lo) return null;
        }
        return lo;
    }
};

pub fn inverse(d: Vec3) Vec3 {
    return Vec3.init(1.0 / d.x, 1.0 / d.y, 1.0 / d.z);
}

const testing = std.testing;

test "merge and surface area" {
    const a = Aabb.fromPoints(Vec3.init(0, 0, 0), Vec3.init(1, 2, 3));
    try testing.expectEqual(@as(f64, 22), a.surfaceArea());
    const b = Aabb.empty.merge(a);
    try testing.expectEqual(a, b);
    try testing.expect(Aabb.empty.isEmpty());
    try testing.expectEqual(@as(usize, 2), a.longestAxis());
    const c = a.include(Vec3.init(-1, 5, 0));
    try testing.expectEqual(Vec3.init(-1, 0, 0), c.min);
    try testing.expectEqual(Vec3.init(1, 5, 3), c.max);
}

test "slab hit" {
    const box = Aabb.fromPoints(Vec3.init(-1, -1, -1), Vec3.init(1, 1, 1));
    const r = Ray.init(Vec3.init(0, 0, -5), Vec3.init(0, 0, 1));
    const t = box.hit(r, inverse(r.dir), 0, 100) orelse return error.TestExpectedHit;
    try testing.expectApproxEqAbs(@as(f64, 4), t, 1e-12);
    // Interval too short.
    try testing.expect(box.hit(r, inverse(r.dir), 0, 3) == null);
    // Ray missing the box (axis-parallel, outside slab).
    const miss = Ray.init(Vec3.init(2, 0, -5), Vec3.init(0, 0, 1));
    try testing.expect(box.hit(miss, inverse(miss.dir), 0, 100) == null);
    // Ray starting inside.
    const inside = Ray.init(Vec3.zero, Vec3.init(1, 1, 0).normalize());
    try testing.expect(box.hit(inside, inverse(inside.dir), 0, 100) != null);
}

test "pad flat box" {
    const flat = Aabb.fromPoints(Vec3.init(0, 0, 0), Vec3.init(1, 0, 1)).pad(1e-4);
    try testing.expect(flat.max.y - flat.min.y >= 1e-4);
    try testing.expectEqual(@as(f64, 1), flat.max.x);
}
