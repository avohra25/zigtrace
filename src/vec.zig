//! Three-component vector used for points, directions and RGB colours.

const std = @import("std");

pub const Vec3 = struct {
    x: f64,
    y: f64,
    z: f64,

    pub const zero: Vec3 = .{ .x = 0, .y = 0, .z = 0 };
    pub const one: Vec3 = .{ .x = 1, .y = 1, .z = 1 };

    pub fn init(x: f64, y: f64, z: f64) Vec3 {
        return .{ .x = x, .y = y, .z = z };
    }

    pub fn splat(v: f64) Vec3 {
        return .{ .x = v, .y = v, .z = v };
    }

    pub fn add(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };
    }

    pub fn sub(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };
    }

    /// Component-wise (Hadamard) product; used to modulate colours.
    pub fn mul(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = a.x * b.x, .y = a.y * b.y, .z = a.z * b.z };
    }

    pub fn scale(a: Vec3, s: f64) Vec3 {
        return .{ .x = a.x * s, .y = a.y * s, .z = a.z * s };
    }

    pub fn div(a: Vec3, s: f64) Vec3 {
        return a.scale(1.0 / s);
    }

    pub fn neg(a: Vec3) Vec3 {
        return .{ .x = -a.x, .y = -a.y, .z = -a.z };
    }

    pub fn dot(a: Vec3, b: Vec3) f64 {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    pub fn cross(a: Vec3, b: Vec3) Vec3 {
        return .{
            .x = a.y * b.z - a.z * b.y,
            .y = a.z * b.x - a.x * b.z,
            .z = a.x * b.y - a.y * b.x,
        };
    }

    pub fn lengthSquared(a: Vec3) f64 {
        return a.dot(a);
    }

    pub fn length(a: Vec3) f64 {
        return @sqrt(a.lengthSquared());
    }

    pub fn normalize(a: Vec3) Vec3 {
        const len = a.length();
        std.debug.assert(len > 0);
        return a.div(len);
    }

    pub fn lerp(a: Vec3, b: Vec3, t: f64) Vec3 {
        return a.scale(1.0 - t).add(b.scale(t));
    }

    pub fn min(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = @min(a.x, b.x), .y = @min(a.y, b.y), .z = @min(a.z, b.z) };
    }

    pub fn max(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = @max(a.x, b.x), .y = @max(a.y, b.y), .z = @max(a.z, b.z) };
    }

    pub fn maxComponent(a: Vec3) f64 {
        return @max(a.x, @max(a.y, a.z));
    }

    /// Returns the component along `axis` (0 = x, 1 = y, 2 = z).
    pub fn get(a: Vec3, axis: usize) f64 {
        return switch (axis) {
            0 => a.x,
            1 => a.y,
            2 => a.z,
            else => unreachable,
        };
    }

    pub fn nearZero(a: Vec3) bool {
        const eps = 1e-8;
        return @abs(a.x) < eps and @abs(a.y) < eps and @abs(a.z) < eps;
    }

    pub fn isFinite(a: Vec3) bool {
        return std.math.isFinite(a.x) and std.math.isFinite(a.y) and std.math.isFinite(a.z);
    }

    pub fn approxEq(a: Vec3, b: Vec3, tol: f64) bool {
        return @abs(a.x - b.x) <= tol and @abs(a.y - b.y) <= tol and @abs(a.z - b.z) <= tol;
    }

    /// Mirror reflection of `v` about the unit normal `n`.
    pub fn reflect(v: Vec3, n: Vec3) Vec3 {
        return v.sub(n.scale(2.0 * v.dot(n)));
    }

    /// Snell refraction of unit vector `uv` through unit normal `n` (facing
    /// against `uv`) with relative index `eta = n_incident / n_transmitted`.
    /// The caller is responsible for handling total internal reflection.
    pub fn refract(uv: Vec3, n: Vec3, eta: f64) Vec3 {
        const cos_theta = @min(uv.neg().dot(n), 1.0);
        const r_perp = uv.add(n.scale(cos_theta)).scale(eta);
        const r_par = n.scale(-@sqrt(@abs(1.0 - r_perp.lengthSquared())));
        return r_perp.add(r_par);
    }
};

const testing = std.testing;

test "arithmetic" {
    const a = Vec3.init(1, 2, 3);
    const b = Vec3.init(4, -5, 6);
    try testing.expectEqual(Vec3.init(5, -3, 9), a.add(b));
    try testing.expectEqual(Vec3.init(-3, 7, -3), a.sub(b));
    try testing.expectEqual(Vec3.init(4, -10, 18), a.mul(b));
    try testing.expectEqual(Vec3.init(2, 4, 6), a.scale(2));
    try testing.expectEqual(@as(f64, 12), a.dot(b));
    try testing.expectEqual(Vec3.init(27, 6, -13), a.cross(b));
    try testing.expectEqual(@as(f64, 3), a.maxComponent());
    try testing.expectEqual(@as(f64, 2), a.get(1));
}

test "cross product is orthogonal" {
    const a = Vec3.init(0.3, -1.2, 2.5);
    const b = Vec3.init(-4.0, 0.7, 1.1);
    const c = a.cross(b);
    try testing.expectApproxEqAbs(@as(f64, 0), c.dot(a), 1e-12);
    try testing.expectApproxEqAbs(@as(f64, 0), c.dot(b), 1e-12);
}

test "normalize yields unit length" {
    const v = Vec3.init(3, 4, 12).normalize();
    try testing.expectApproxEqAbs(@as(f64, 1), v.length(), 1e-12);
    try testing.expect(v.approxEq(Vec3.init(3.0 / 13.0, 4.0 / 13.0, 12.0 / 13.0), 1e-12));
}

test "reflect" {
    const v = Vec3.init(1, -1, 0);
    const r = v.reflect(Vec3.init(0, 1, 0));
    try testing.expectEqual(Vec3.init(1, 1, 0), r);
}

test "refract obeys Snell's law" {
    const n = Vec3.init(0, 1, 0);
    const theta_i = std.math.degreesToRadians(30.0);
    const uv = Vec3.init(@sin(theta_i), -@cos(theta_i), 0);
    const eta = 1.0 / 1.5;
    const t = uv.refract(n, eta);
    try testing.expectApproxEqAbs(@as(f64, 1), t.length(), 1e-12);
    // sin(theta_t) = eta * sin(theta_i)
    try testing.expectApproxEqAbs(eta * @sin(theta_i), t.x, 1e-12);
    try testing.expect(t.y < 0);
    // Index ratio of one passes straight through.
    try testing.expect(uv.refract(n, 1.0).approxEq(uv, 1e-12));
}
