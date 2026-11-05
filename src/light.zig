//! Emitter sampling for next-event estimation.
//!
//! A `Lights` table holds every bounded emissive triangle and sphere of a
//! scene. `sample` picks a light (proportional to its emitted power) and a
//! point on it as seen from a shading point, and `pdfHit` reports the
//! probability density, per unit solid angle, that `sample` would have
//! produced a given emitter hit. Together they let the integrator weight
//! light sampling against BSDF sampling (multiple importance sampling).

const std = @import("std");
const Vec3 = @import("vec.zig").Vec3;
const Ray = @import("ray.zig").Ray;
const Rng = @import("rng.zig").Rng;
const geometry = @import("geometry.zig");
const Shape = geometry.Shape;
const Hit = geometry.Hit;
const Material = @import("material.zig").Material;

/// An emitter chosen by `Lights.sample`.
pub const LightSample = struct {
    /// Unit direction from the shading point towards the emitter.
    dir: Vec3,
    /// Distance along `dir` to the sampled point.
    distance: f64,
    /// Emitted radiance towards the shading point.
    radiance: Vec3,
    /// Density per unit solid angle, including the chance of choosing this light.
    pdf: f64,
};

pub const Light = struct {
    shape: Shape,
    radiance: Vec3,
    /// Relative chance of being picked: luminance times surface area.
    weight: f64,
};

fn luminance(c: Vec3) f64 {
    return 0.2126 * c.x + 0.7152 * c.y + 0.0722 * c.z;
}

fn triangleArea(t: geometry.Triangle) f64 {
    return 0.5 * t.v1.sub(t.v0).cross(t.v2.sub(t.v0)).length();
}

/// True when `origin` is far enough outside the sphere for cone sampling.
fn outside(s: geometry.Sphere, origin: Vec3) bool {
    const d = s.center.sub(origin).length();
    return d > @abs(s.radius) * (1 + 1e-9);
}

/// Orthonormal basis (u, v) perpendicular to the unit vector `w`.
fn basis(w: Vec3) struct { u: Vec3, v: Vec3 } {
    const a = if (@abs(w.x) > 0.9) Vec3.init(0, 1, 0) else Vec3.init(1, 0, 0);
    const v = w.cross(a).normalize();
    return .{ .u = v.cross(w), .v = v };
}

pub const Lights = struct {
    items: []Light = &.{},
    /// Running sum of `items[..i+1].weight`, for picking by binary search.
    cdf: []f64 = &.{},
    total: f64 = 0,

    /// Collects the emissive triangles and spheres among `shapes`. Planes are
    /// unbounded and spheres with a negative radius are hollow bubbles, so
    /// neither is sampled; they still emit when a path happens to hit them.
    pub fn build(allocator: std.mem.Allocator, shapes: []const Shape, materials: []const Material) !Lights {
        var list: std.ArrayListUnmanaged(Light) = .empty;
        errdefer list.deinit(allocator);
        for (shapes) |shape| {
            const mat_id, const area = switch (shape) {
                .triangle => |t| .{ t.material, triangleArea(t) },
                .sphere => |s| .{ s.material, if (s.radius > 0) 4 * std.math.pi * s.radius * s.radius else 0 },
                .plane => continue,
            };
            const radiance = switch (materials[mat_id]) {
                .emissive => |e| e.color,
                else => continue,
            };
            const weight = luminance(radiance) * area;
            if (!(weight > 0) or !std.math.isFinite(weight)) continue;
            try list.append(allocator, .{ .shape = shape, .radiance = radiance, .weight = weight });
        }
        const items = try list.toOwnedSlice(allocator);
        errdefer allocator.free(items);
        const cdf = try allocator.alloc(f64, items.len);
        var sum: f64 = 0;
        for (items, cdf) |l, *c| {
            sum += l.weight;
            c.* = sum;
        }
        return .{ .items = items, .cdf = cdf, .total = sum };
    }

    pub fn deinit(self: *Lights, allocator: std.mem.Allocator) void {
        allocator.free(self.items);
        allocator.free(self.cdf);
        self.* = .{};
    }

    pub fn isEmpty(self: Lights) bool {
        return self.items.len == 0;
    }

    fn pick(self: Lights, rng: *Rng) *const Light {
        const x = rng.float() * self.total;
        var lo: usize = 0;
        var hi: usize = self.cdf.len - 1;
        while (lo < hi) {
            const mid = (lo + hi) / 2;
            if (self.cdf[mid] > x) hi = mid else lo = mid + 1;
        }
        return &self.items[lo];
    }

    /// Samples a point on a randomly chosen light as seen from `origin`.
    /// Returns null when the pick cannot light `origin` (a back-facing
    /// emitter, or an origin inside a spherical light).
    pub fn sample(self: Lights, origin: Vec3, rng: *Rng) ?LightSample {
        if (self.items.len == 0) return null;
        const light = self.pick(rng);
        const select = light.weight / self.total;
        switch (light.shape) {
            .triangle => |t| {
                const su = @sqrt(rng.float());
                const b2 = rng.float();
                const p = t.v0.scale(1 - su).add(t.v1.scale(su * (1 - b2))).add(t.v2.scale(su * b2));
                const to = p.sub(origin);
                const dist = to.length();
                if (dist < 1e-9) return null;
                const dir = to.scale(1 / dist);
                const n = t.v1.sub(t.v0).cross(t.v2.sub(t.v0)).normalize();
                const cos_l = -dir.dot(n);
                if (cos_l <= 1e-9) return null;
                return .{
                    .dir = dir,
                    .distance = dist,
                    .radiance = light.radiance,
                    .pdf = select * dist * dist / (cos_l * triangleArea(t)),
                };
            },
            .sphere => |s| {
                if (!outside(s, origin)) return null;
                const axis = s.center.sub(origin);
                const d2 = axis.lengthSquared();
                const w = axis.scale(1 / @sqrt(d2));
                const cos_max = @sqrt(@max(0.0, 1 - s.radius * s.radius / d2));
                const cos_t = 1 - rng.float() * (1 - cos_max);
                const sin_t = @sqrt(@max(0.0, 1 - cos_t * cos_t));
                const phi = 2 * std.math.pi * rng.float();
                const b = basis(w);
                const dir = b.u.scale(sin_t * @cos(phi)).add(b.v.scale(sin_t * @sin(phi))).add(w.scale(cos_t)).normalize();
                // The cone is sampled, not the surface, so find where the ray lands.
                const hit = s.intersect(Ray.init(origin, dir), 0, std.math.inf(f64)) orelse return null;
                if (!hit.front_face) return null;
                return .{
                    .dir = dir,
                    .distance = hit.t,
                    .radiance = light.radiance,
                    .pdf = select / (2 * std.math.pi * (1 - cos_max)),
                };
            },
            .plane => unreachable,
        }
    }

    /// Density, per unit solid angle, with which `sample` from `origin` would
    /// have produced the direction towards `hit`. Zero when `hit` is not on
    /// a sampled emitter or could never have been reached by `sample`.
    pub fn pdfHit(self: Lights, origin: Vec3, hit: Hit) f64 {
        const to = hit.point.sub(origin);
        const dist = to.length();
        if (dist < 1e-9) return 0;
        const dir = to.scale(1 / dist);
        const ray = Ray.init(origin, dir);
        var pdf: f64 = 0;
        for (self.items) |light| {
            const select = light.weight / self.total;
            switch (light.shape) {
                .triangle => |t| {
                    const h = t.intersect(ray, 1e-9, std.math.inf(f64)) orelse continue;
                    if (@abs(h.t - dist) > 1e-6 * dist or !h.front_face) continue;
                    pdf += select * dist * dist / (-dir.dot(h.normal) * triangleArea(t));
                },
                .sphere => |s| {
                    if (!outside(s, origin)) continue;
                    const h = s.intersect(ray, 1e-9, std.math.inf(f64)) orelse continue;
                    if (@abs(h.t - dist) > 1e-6 * dist or !h.front_face) continue;
                    const d2 = s.center.sub(origin).lengthSquared();
                    const cos_max = @sqrt(@max(0.0, 1 - s.radius * s.radius / d2));
                    pdf += select / (2 * std.math.pi * (1 - cos_max));
                },
                .plane => {},
            }
        }
        return pdf;
    }
};

const testing = std.testing;
const Scene = @import("scene.zig").Scene;

fn lampScene(scene: *Scene) !void {
    const white = try scene.addMaterial(.{ .lambertian = .{ .albedo = Vec3.one } });
    const dim = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.init(1, 1, 1) } });
    const bright = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.init(4, 4, 4) } });
    try scene.add(.{ .plane = .{ .point = Vec3.zero, .normal = Vec3.init(0, 1, 0), .material = white } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 3, 0), .radius = 0.5, .material = dim } });
    // A quad light facing down (u x v points along -y) and a decoy that is not emissive.
    try scene.addQuad(Vec3.init(2, 2.5, -1), Vec3.init(1, 0, 0), Vec3.init(0, 0, 2), bright);
    try scene.add(.{ .sphere = .{ .center = Vec3.init(-3, 1, 0), .radius = 1, .material = white } });
    try scene.build();
}

test "only bounded emissive shapes become lights" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    try lampScene(&scene);
    // One sphere plus the two halves of the quad.
    try testing.expectEqual(@as(usize, 3), scene.lights.items.len);
    try testing.expect(!scene.lights.isEmpty());
    const sphere_weight = 4 * std.math.pi * 0.25;
    try testing.expectApproxEqAbs(sphere_weight, scene.lights.items[0].weight, 1e-12);
    // The quad is 2 x 1 with radiance 4, split into two triangles.
    try testing.expectApproxEqAbs(sphere_weight + 4 * 2.0, scene.lights.total, 1e-12);
}

test "scene without emitters has no lights and never samples" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    const m = try scene.addMaterial(.{ .lambertian = .{ .albedo = Vec3.one } });
    try scene.add(.{ .sphere = .{ .center = Vec3.zero, .radius = 1, .material = m } });
    try scene.build();
    var rng = Rng.init(1);
    try testing.expect(scene.lights.isEmpty());
    try testing.expect(scene.lights.sample(Vec3.init(0, 5, 0), &rng) == null);
}

test "emissive planes, bubbles and dark emitters are not sampled" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    const lamp = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.one } });
    const dark = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.zero } });
    try scene.add(.{ .plane = .{ .point = Vec3.zero, .normal = Vec3.init(0, 1, 0), .material = lamp } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 2, 0), .radius = -1, .material = lamp } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 5, 0), .radius = 1, .material = dark } });
    try scene.build();
    try testing.expect(scene.lights.isEmpty());
}

test "samples land on the emitters with consistent pdfs" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    try lampScene(&scene);
    var rng = Rng.init(7);
    const origin = Vec3.init(0.5, 0.2, 0.3);
    var got: usize = 0;
    for (0..2000) |_| {
        const s = scene.lights.sample(origin, &rng) orelse continue;
        got += 1;
        try testing.expectApproxEqAbs(@as(f64, 1), s.dir.length(), 1e-9);
        // Shoot the sampled ray: the first thing it meets must be an emitter
        // at the sampled distance, and pdfHit must agree with the sample pdf.
        const hit = scene.intersect(Ray.init(origin, s.dir), 1e-6, std.math.inf(f64)).?;
        try testing.expectApproxEqRel(s.distance, hit.t, 1e-6);
        try testing.expect(scene.material(hit.material) == .emissive);
        try testing.expect(hit.front_face);
        try testing.expectApproxEqRel(s.pdf, scene.lights.pdfHit(origin, hit), 1e-6);
    }
    try testing.expect(got > 1500);
}

test "reciprocal pdf averages to the solid angle of a sphere light" {
    // E[1/pdf] over samples equals the measure of the sampled directions,
    // here the cone's solid angle 2*pi*(1 - cos(theta_max)).
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    const lamp = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.one } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 0, -4), .radius = 1, .material = lamp } });
    try scene.build();
    var rng = Rng.init(11);
    const n = 20000;
    var sum: f64 = 0;
    for (0..n) |_| sum += 1 / scene.lights.sample(Vec3.zero, &rng).?.pdf;
    const cos_max = @sqrt(1 - 1.0 / 16.0);
    try testing.expectApproxEqRel(2 * std.math.pi * (1 - cos_max), sum / n, 1e-9);
}

test "reciprocal pdf averages to the solid angle of a triangle light" {
    // Compare against brute force: the fraction of uniformly random
    // directions that hit the triangle times 4*pi.
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    const lamp = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.one } });
    const tri = geometry.Triangle{
        .v0 = Vec3.init(-1, 2, -1),
        .v1 = Vec3.init(1.5, 2, -1),
        .v2 = Vec3.init(0, 2, 1.5),
        .material = lamp,
    };
    // e1 x e2 points along -y, so the front face looks down at the origin.
    try scene.add(.{ .triangle = tri });
    try scene.build();
    var rng = Rng.init(21);
    const origin = Vec3.zero;
    const n = 40000;
    var sum: f64 = 0;
    for (0..n) |_| sum += 1 / scene.lights.sample(origin, &rng).?.pdf;
    var hits: usize = 0;
    const trials = 400000;
    for (0..trials) |_| {
        if (tri.intersect(Ray.init(origin, rng.unitVector()), 1e-9, 1e9) != null) hits += 1;
    }
    const brute = 4 * std.math.pi * @as(f64, @floatFromInt(hits)) / trials;
    try testing.expectApproxEqRel(brute, sum / n, 0.03);
}

test "back-facing triangle and enclosing sphere light nothing" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    const lamp = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.one } });
    // Front face is +y (e1 x e2), so a point above sees it and one below does not.
    try scene.add(.{ .triangle = .{ .v0 = Vec3.init(-1, 0, -1), .v1 = Vec3.init(-1, 0, 1), .v2 = Vec3.init(1, 0, 0), .material = lamp } });
    try scene.build();
    var rng = Rng.init(3);
    try testing.expect(scene.lights.sample(Vec3.init(0, -2, 0), &rng) == null);
    try testing.expect(scene.lights.sample(Vec3.init(0, 2, 0), &rng) != null);

    var inside = Scene.init(testing.allocator);
    defer inside.deinit();
    const l2 = try inside.addMaterial(.{ .emissive = .{ .color = Vec3.one } });
    try inside.add(.{ .sphere = .{ .center = Vec3.zero, .radius = 2, .material = l2 } });
    try inside.build();
    try testing.expect(inside.lights.sample(Vec3.zero, &rng) == null);
}

test "lights are picked in proportion to their power" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    const weak = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.splat(1) } });
    const strong = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.splat(3) } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(-5, 0, 0), .radius = 1, .material = weak } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(5, 0, 0), .radius = 1, .material = strong } });
    try scene.build();
    var rng = Rng.init(5);
    var right: usize = 0;
    const n = 20000;
    for (0..n) |_| {
        if (scene.lights.sample(Vec3.zero, &rng).?.dir.x > 0) right += 1;
    }
    try testing.expectApproxEqAbs(@as(f64, 0.75), @as(f64, @floatFromInt(right)) / n, 0.015);
}
