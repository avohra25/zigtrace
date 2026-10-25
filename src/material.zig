//! Surface materials and their scattering functions.

const std = @import("std");
const Vec3 = @import("vec.zig").Vec3;
const Ray = @import("ray.zig").Ray;
const Rng = @import("rng.zig").Rng;
const Hit = @import("geometry.zig").Hit;

pub const Scatter = struct {
    /// Per-channel throughput multiplier for the scattered path.
    attenuation: Vec3,
    ray: Ray,
};

pub const Material = union(enum) {
    /// Ideal diffuse reflector, sampled with a cosine-weighted lobe so the
    /// BRDF/pdf ratio collapses to the albedo.
    lambertian: struct { albedo: Vec3 },
    /// Specular reflector; `fuzz` in [0, 1] perturbs the mirror direction.
    metal: struct { albedo: Vec3, fuzz: f64 = 0 },
    /// Clear refractive material (glass, water) with index of refraction.
    dielectric: struct { ior: f64 },
    /// Area light. Emits radiance and absorbs everything.
    emissive: struct { color: Vec3 },

    pub fn emitted(self: Material, hit: Hit) Vec3 {
        return switch (self) {
            // Lights are one-sided: only the outward face emits.
            .emissive => |e| if (hit.front_face) e.color else Vec3.zero,
            else => Vec3.zero,
        };
    }

    pub fn scatter(self: Material, r_in: Ray, hit: Hit, rng: *Rng) ?Scatter {
        switch (self) {
            .lambertian => |m| {
                var dir = hit.normal.add(rng.unitVector());
                if (dir.nearZero()) dir = hit.normal;
                return .{ .attenuation = m.albedo, .ray = Ray.init(hit.point, dir.normalize()) };
            },
            .metal => |m| {
                var dir = r_in.dir.normalize().reflect(hit.normal);
                if (m.fuzz > 0) dir = dir.add(rng.unitVector().scale(m.fuzz)).normalize();
                // Fuzz can push the ray below the surface; absorb it.
                if (dir.dot(hit.normal) <= 0) return null;
                return .{ .attenuation = m.albedo, .ray = Ray.init(hit.point, dir) };
            },
            .dielectric => |m| {
                const eta = if (hit.front_face) 1.0 / m.ior else m.ior;
                const unit = r_in.dir.normalize();
                const cos_theta = @min(unit.neg().dot(hit.normal), 1.0);
                const sin_theta = @sqrt(@max(0.0, 1.0 - cos_theta * cos_theta));
                const cannot_refract = eta * sin_theta > 1.0;
                const dir = if (cannot_refract or schlick(cos_theta, eta) > rng.float())
                    unit.reflect(hit.normal)
                else
                    unit.refract(hit.normal, eta);
                return .{ .attenuation = Vec3.one, .ray = Ray.init(hit.point, dir) };
            },
            .emissive => return null,
        }
    }
};

/// Schlick's approximation of Fresnel reflectance.
pub fn schlick(cosine: f64, eta: f64) f64 {
    var r0 = (1.0 - eta) / (1.0 + eta);
    r0 *= r0;
    return r0 + (1.0 - r0) * std.math.pow(f64, 1.0 - cosine, 5);
}

const testing = std.testing;

fn testHit(front: bool) Hit {
    return .{
        .t = 1,
        .point = Vec3.zero,
        .normal = Vec3.init(0, 1, 0),
        .front_face = front,
        .material = 0,
    };
}

test "lambertian scatters into the hemisphere with albedo" {
    var rng = Rng.init(5);
    const m = Material{ .lambertian = .{ .albedo = Vec3.init(0.2, 0.4, 0.6) } };
    const r = Ray.init(Vec3.init(0, 1, 0), Vec3.init(0, -1, 0));
    var mean_cos: f64 = 0;
    const n = 20_000;
    for (0..n) |_| {
        const s = m.scatter(r, testHit(true), &rng).?;
        try testing.expectEqual(Vec3.init(0.2, 0.4, 0.6), s.attenuation);
        const c = s.ray.dir.dot(Vec3.init(0, 1, 0));
        try testing.expect(c >= -1e-9);
        mean_cos += c;
    }
    // A cosine-weighted lobe has E[cos] = 2/3.
    try testing.expectApproxEqAbs(@as(f64, 2.0 / 3.0), mean_cos / n, 0.01);
}

test "perfect metal mirrors" {
    var rng = Rng.init(1);
    const m = Material{ .metal = .{ .albedo = Vec3.one } };
    const r = Ray.init(Vec3.init(-1, 1, 0), Vec3.init(1, -1, 0));
    const s = m.scatter(r, testHit(true), &rng).?;
    try testing.expect(s.ray.dir.approxEq(Vec3.init(1, 1, 0).normalize(), 1e-12));
}

test "dielectric with unit ior passes straight through" {
    var rng = Rng.init(2);
    const m = Material{ .dielectric = .{ .ior = 1.0 } };
    const dir = Vec3.init(0.3, -1, 0.1).normalize();
    for (0..100) |_| {
        const s = m.scatter(Ray.init(Vec3.init(0, 1, 0), dir), testHit(true), &rng).?;
        try testing.expect(s.ray.dir.approxEq(dir, 1e-12));
        try testing.expectEqual(Vec3.one, s.attenuation);
    }
}

test "total internal reflection" {
    var rng = Rng.init(3);
    const m = Material{ .dielectric = .{ .ior = 1.5 } };
    // Inside glass at a grazing angle: exceeds the critical angle.
    const dir = Vec3.init(1, -0.2, 0).normalize();
    const s = m.scatter(Ray.init(Vec3.zero, dir), testHit(false), &rng).?;
    try testing.expect(s.ray.dir.approxEq(dir.reflect(Vec3.init(0, 1, 0)), 1e-12));
}

test "schlick endpoints" {
    try testing.expectApproxEqAbs(@as(f64, 0.04), schlick(1.0, 1.0 / 1.5), 1e-12);
    try testing.expectApproxEqAbs(@as(f64, 1.0), schlick(0.0, 1.0 / 1.5), 1e-12);
}

test "emissive emits only from the front and never scatters" {
    var rng = Rng.init(4);
    const m = Material{ .emissive = .{ .color = Vec3.init(4, 4, 4) } };
    try testing.expectEqual(Vec3.init(4, 4, 4), m.emitted(testHit(true)));
    try testing.expectEqual(Vec3.zero, m.emitted(testHit(false)));
    try testing.expect(m.scatter(Ray.init(Vec3.zero, Vec3.init(0, -1, 0)), testHit(true), &rng) == null);
    const lam = Material{ .lambertian = .{ .albedo = Vec3.one } };
    try testing.expectEqual(Vec3.zero, lam.emitted(testHit(true)));
}
