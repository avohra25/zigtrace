//! Monte Carlo path tracing integrator and multithreaded tile renderer.

const std = @import("std");
const Vec3 = @import("vec.zig").Vec3;
const Ray = @import("ray.zig").Ray;
const Rng = @import("rng.zig").Rng;
const Image = @import("image.zig").Image;
const Scene = @import("scene.zig").Scene;
const Hit = @import("geometry.zig").Hit;
const Camera = @import("camera.zig").Camera;

pub const Options = struct {
    width: usize,
    height: usize,
    spp: usize,
    max_depth: usize,
    /// Worker threads including the calling thread; must be at least 1.
    threads: usize = 1,
    seed: u64 = 0,
    tile_size: usize = 16,
    /// Sample emitters directly at diffuse surfaces (next-event estimation).
    light_sampling: bool = true,
};

/// Offset that keeps secondary rays from re-hitting their own surface.
const ray_epsilon = 1e-4;
/// Paths shorter than this are never terminated by Russian roulette.
const roulette_depth = 4;

/// Estimates incoming radiance along `ray` with one random path, using
/// next-event estimation when the scene has lights.
pub fn trace(scene: *const Scene, initial: Ray, rng: *Rng, max_depth: usize) Vec3 {
    return traceWith(scene, initial, rng, max_depth, true);
}

/// Power heuristic (beta = 2) weight for a sample drawn with density `a`
/// when the other strategy has density `b`.
fn powerWeight(a: f64, b: f64) f64 {
    const a2 = a * a;
    return a2 / (a2 + b * b);
}

/// Light arriving at a diffuse point straight from one sampled emitter,
/// weighted against the chance that BSDF sampling would have found the same
/// direction. Occluded or back-facing samples contribute nothing.
fn directLight(scene: *const Scene, albedo: Vec3, hit: Hit, rng: *Rng) Vec3 {
    const ls = scene.lights.sample(hit.point, rng) orelse return Vec3.zero;
    const cos_s = hit.normal.dot(ls.dir);
    if (cos_s <= 0 or !(ls.pdf > 0)) return Vec3.zero;
    const shadow = Ray.init(hit.point, ls.dir);
    if (scene.intersect(shadow, ray_epsilon, ls.distance * (1 - 1e-6)) != null) return Vec3.zero;
    const bsdf_pdf = cos_s / std.math.pi;
    // albedo / pi * cos / pdf, times the MIS weight.
    const scale = cos_s / (std.math.pi * ls.pdf) * powerWeight(ls.pdf, bsdf_pdf);
    return albedo.mul(ls.radiance).scale(scale);
}

/// Like `trace`, with light sampling switched on or off. With it off, only
/// BSDF sampling finds emitters, which converges far more slowly for small
/// lights but is a handy reference.
pub fn traceWith(scene: *const Scene, initial: Ray, rng: *Rng, max_depth: usize, light_sampling: bool) Vec3 {
    const use_nee = light_sampling and !scene.lights.isEmpty();
    var radiance = Vec3.zero;
    var throughput = Vec3.one;
    var r = initial;
    // Where the last diffuse bounce started and the density it used, so an
    // emitter found by that bounce can be weighted against light sampling.
    // A density of 0 means the previous bounce was not light-sampled.
    var prev_origin = Vec3.zero;
    var prev_pdf: f64 = 0;
    var depth: usize = 0;
    while (depth < max_depth) : (depth += 1) {
        const hit = scene.intersect(r, ray_epsilon, std.math.inf(f64)) orelse {
            return radiance.add(throughput.mul(scene.background.radiance(r.dir)));
        };
        const mat = scene.material(hit.material);
        const le = mat.emitted(hit);
        if (le.maxComponent() > 0) {
            const w = if (prev_pdf > 0) powerWeight(prev_pdf, scene.lights.pdfHit(prev_origin, hit)) else 1.0;
            radiance = radiance.add(throughput.mul(le).scale(w));
        }
        switch (mat) {
            .lambertian => |m| if (use_nee) {
                radiance = radiance.add(throughput.mul(directLight(scene, m.albedo, hit, rng)));
            },
            else => {},
        }
        const s = mat.scatter(r, hit, rng) orelse break;
        throughput = throughput.mul(s.attenuation);
        prev_pdf = if (use_nee and mat == .lambertian) @max(s.ray.dir.dot(hit.normal), 0) / std.math.pi else 0;
        prev_origin = hit.point;
        r = s.ray;

        if (depth + 1 >= roulette_depth) {
            // Russian roulette: terminate dim paths early, reweight survivors.
            const p = @min(throughput.maxComponent(), 0.95);
            if (p <= 0 or rng.float() >= p) break;
            throughput = throughput.div(p);
        }
    }
    return radiance;
}

const Context = struct {
    scene: *const Scene,
    camera: Camera,
    opts: Options,
    image: *Image,
    tiles_x: usize,
    tile_count: usize,
    next_tile: std.atomic.Value(usize) = .init(0),

    fn work(ctx: *Context) void {
        while (true) {
            const tile = ctx.next_tile.fetchAdd(1, .monotonic);
            if (tile >= ctx.tile_count) return;
            ctx.renderTile(tile);
        }
    }

    fn renderTile(ctx: *Context, tile: usize) void {
        const ts = ctx.opts.tile_size;
        const x0 = (tile % ctx.tiles_x) * ts;
        const y0 = (tile / ctx.tiles_x) * ts;
        const x1 = @min(x0 + ts, ctx.opts.width);
        const y1 = @min(y0 + ts, ctx.opts.height);
        const w: f64 = @floatFromInt(ctx.opts.width);
        const h: f64 = @floatFromInt(ctx.opts.height);
        const inv_spp = 1.0 / @as(f64, @floatFromInt(ctx.opts.spp));

        for (y0..y1) |y| {
            for (x0..x1) |x| {
                // One RNG stream per pixel: output is independent of tiling
                // and thread scheduling.
                var rng = Rng.forStream(ctx.opts.seed, y * ctx.opts.width + x);
                var sum = Vec3.zero;
                for (0..ctx.opts.spp) |_| {
                    const s = (@as(f64, @floatFromInt(x)) + rng.float()) / w;
                    const t = (@as(f64, @floatFromInt(y)) + rng.float()) / h;
                    const c = traceWith(ctx.scene, ctx.camera.ray(s, t, &rng), &rng, ctx.opts.max_depth, ctx.opts.light_sampling);
                    // Drop numerically broken samples rather than poison the pixel.
                    if (c.isFinite()) sum = sum.add(c);
                }
                ctx.image.set(x, y, sum.scale(inv_spp));
            }
        }
    }
};

/// Renders `scene` (whose BVH should already be built) into a new linear
/// HDR image owned by the caller.
pub fn render(allocator: std.mem.Allocator, scene: *const Scene, opts: Options) !Image {
    if (opts.width == 0 or opts.height == 0 or opts.spp == 0 or opts.tile_size == 0) return error.InvalidOptions;
    var image = try Image.init(allocator, opts.width, opts.height);
    errdefer image.deinit(allocator);

    const aspect = @as(f64, @floatFromInt(opts.width)) / @as(f64, @floatFromInt(opts.height));
    const tiles_x = std.math.divCeil(usize, opts.width, opts.tile_size) catch unreachable;
    const tiles_y = std.math.divCeil(usize, opts.height, opts.tile_size) catch unreachable;
    var ctx = Context{
        .scene = scene,
        .camera = Camera.init(scene.camera, aspect),
        .opts = opts,
        .image = &image,
        .tiles_x = tiles_x,
        .tile_count = tiles_x * tiles_y,
    };

    const extra = @min(@max(opts.threads, 1), ctx.tile_count) - 1;
    const threads = try allocator.alloc(std.Thread, extra);
    defer allocator.free(threads);
    var spawned: usize = 0;
    defer for (threads[0..spawned]) |t| t.join();
    for (threads) |*t| {
        t.* = try std.Thread.spawn(.{}, Context.work, .{&ctx});
        spawned += 1;
    }
    ctx.work();
    return image;
}

const testing = std.testing;

fn testOptions() Options {
    return .{ .width = 24, .height = 16, .spp = 8, .max_depth = 8, .tile_size = 5 };
}

test "empty scene renders the background exactly" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    scene.background = .{ .solid = Vec3.init(0.25, 0.5, 0.75) };
    try scene.build();
    var img = try render(testing.allocator, &scene, testOptions());
    defer img.deinit(testing.allocator);
    for (img.pixels) |p| try testing.expect(p.approxEq(Vec3.init(0.25, 0.5, 0.75), 1e-12));
}

fn demoScene(scene: *Scene) !void {
    const ground = try scene.addMaterial(.{ .lambertian = .{ .albedo = Vec3.init(0.5, 0.5, 0.5) } });
    const glass = try scene.addMaterial(.{ .dielectric = .{ .ior = 1.5 } });
    const gold = try scene.addMaterial(.{ .metal = .{ .albedo = Vec3.init(0.8, 0.6, 0.2), .fuzz = 0.2 } });
    const lamp = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.init(3, 3, 3) } });
    try scene.add(.{ .plane = .{ .point = Vec3.init(0, -1, 0), .normal = Vec3.init(0, 1, 0), .material = ground } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(-1, 0, -3), .radius = 1, .material = glass } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(1, 0, -3), .radius = 1, .material = gold } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 2, -4), .radius = 0.5, .material = lamp } });
    scene.camera = .{ .look_from = Vec3.init(0, 0.5, 1), .look_at = Vec3.init(0, 0, -3), .aperture = 0.1 };
    try scene.build();
}

test "output is identical for any thread count and depends on seed" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    try demoScene(&scene);

    var single = try render(testing.allocator, &scene, testOptions());
    defer single.deinit(testing.allocator);
    var opts = testOptions();
    opts.threads = 4;
    var multi = try render(testing.allocator, &scene, opts);
    defer multi.deinit(testing.allocator);
    try testing.expectEqualSlices(Vec3, single.pixels, multi.pixels);

    opts.seed = 99;
    var reseeded = try render(testing.allocator, &scene, opts);
    defer reseeded.deinit(testing.allocator);
    try testing.expect(!std.mem.eql(u8, std.mem.sliceAsBytes(single.pixels), std.mem.sliceAsBytes(reseeded.pixels)));
}

test "diffuse sphere under a uniform sky reflects exactly its albedo" {
    // A convex Lambertian surface lit by a constant environment: every path
    // hits the sphere once and escapes, so radiance == albedo * sky.
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    scene.background = .{ .solid = Vec3.one };
    const m = try scene.addMaterial(.{ .lambertian = .{ .albedo = Vec3.init(0.5, 0.25, 0.125) } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 0, -3), .radius = 1, .material = m } });
    scene.camera = .{ .vfov = 10 };
    try scene.build();
    var img = try render(testing.allocator, &scene, testOptions());
    defer img.deinit(testing.allocator);
    for (img.pixels) |p| try testing.expect(p.approxEq(Vec3.init(0.5, 0.25, 0.125), 1e-12));
}

test "white furnace: a non-absorbing enclosure conserves energy" {
    // A glass shell around a white diffuse core, in a uniform white
    // environment. Nothing absorbs, so every pixel must converge to 1; Russian
    // roulette keeps this unbiased and only max_depth truncation (rare at 64
    // bounces) can pull it down.
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    scene.background = .{ .solid = Vec3.one };
    const glass = try scene.addMaterial(.{ .dielectric = .{ .ior = 1.5 } });
    const white = try scene.addMaterial(.{ .lambertian = .{ .albedo = Vec3.one } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 0, -3), .radius = 1, .material = glass } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 0, -3), .radius = 0.5, .material = white } });
    try scene.build();
    var opts = testOptions();
    opts.max_depth = 64;
    opts.spp = 32;
    var img = try render(testing.allocator, &scene, opts);
    defer img.deinit(testing.allocator);
    var mean = Vec3.zero;
    for (img.pixels) |p| mean = mean.add(p);
    mean = mean.div(@floatFromInt(img.pixels.len));
    try testing.expect(mean.approxEq(Vec3.one, 0.03));
}

test "emitters are seen directly" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    scene.background = .{ .solid = Vec3.zero };
    const lamp = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.init(2, 1, 0.5) } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 0, -2), .radius = 1, .material = lamp } });
    scene.camera = .{ .vfov = 20 };
    try scene.build();
    var img = try render(testing.allocator, &scene, testOptions());
    defer img.deinit(testing.allocator);
    for (img.pixels) |p| try testing.expectEqual(Vec3.init(2, 1, 0.5), p);
}

test "rejects degenerate options" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    var opts = testOptions();
    opts.spp = 0;
    try testing.expectError(error.InvalidOptions, render(testing.allocator, &scene, opts));
}

fn meanRadiance(scene: *const Scene, ray: Ray, seed: u64, n: usize, max_depth: usize, nee: bool) struct { mean: f64, variance: f64 } {
    var rng = Rng.init(seed);
    var sum: f64 = 0;
    var sum2: f64 = 0;
    for (0..n) |_| {
        const v = traceWith(scene, ray, &rng, max_depth, nee).x;
        sum += v;
        sum2 += v * v;
    }
    const nf: f64 = @floatFromInt(n);
    const mean = sum / nf;
    return .{ .mean = mean, .variance = sum2 / nf - mean * mean };
}

fn lampOverFloor(scene: *Scene, lamp_radius: f64) !void {
    scene.background = .{ .solid = Vec3.zero };
    const floor = try scene.addMaterial(.{ .lambertian = .{ .albedo = Vec3.splat(0.5) } });
    const lamp = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.splat(2) } });
    try scene.add(.{ .plane = .{ .point = Vec3.zero, .normal = Vec3.init(0, 1, 0), .material = floor } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 3, 0), .radius = lamp_radius, .material = lamp } });
    try scene.build();
}

test "irradiance from a sphere light matches the closed form with and without light sampling" {
    // A diffuse point directly below a sphere of radius R at distance d has
    // radiance albedo * Le * (R / d)^2.
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    try lampOverFloor(&scene, 1);
    const down = Ray.init(Vec3.init(0, 1, 0), Vec3.init(0, -1, 0));
    const expected = 0.5 * 2.0 / 9.0;
    const with = meanRadiance(&scene, down, 1, 20000, 2, true);
    const without = meanRadiance(&scene, down, 2, 40000, 2, false);
    try testing.expectApproxEqRel(expected, with.mean, 0.02);
    try testing.expectApproxEqRel(expected, without.mean, 0.06);
}

test "light sampling slashes variance for a small light" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    try lampOverFloor(&scene, 0.2);
    const down = Ray.init(Vec3.init(0, 1, 0), Vec3.init(0, -1, 0));
    const with = meanRadiance(&scene, down, 3, 5000, 2, true);
    const without = meanRadiance(&scene, down, 3, 5000, 2, false);
    try testing.expect(with.variance * 10 < without.variance);
    const expected = 0.5 * 2.0 * 0.04 / 9.0;
    try testing.expectApproxEqRel(expected, with.mean, 0.02);
}

test "emitters seen through a mirror are not weighted" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    scene.background = .{ .solid = Vec3.zero };
    const mirror = try scene.addMaterial(.{ .metal = .{ .albedo = Vec3.one, .fuzz = 0 } });
    const lamp = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.init(3, 2, 1) } });
    try scene.add(.{ .plane = .{ .point = Vec3.zero, .normal = Vec3.init(0, 1, 0), .material = mirror } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(8, 4, 0), .radius = 1, .material = lamp } });
    try scene.build();
    // This 45 degree ray reflects off the mirror straight into the lamp.
    const ray = Ray.init(Vec3.init(0, 4, 0), Vec3.init(1, -1, 0).normalize());
    var rng = Rng.init(9);
    for ([_]bool{ true, false }) |nee| {
        const c = traceWith(&scene, ray, &rng, 4, nee);
        try testing.expect(c.approxEq(Vec3.init(3, 2, 1), 1e-9));
    }
}

test "shadowed lights add nothing" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    try lampOverFloor(&scene, 1);
    const blocker = try scene.addMaterial(.{ .lambertian = .{ .albedo = Vec3.zero } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 1.8, 0), .radius = 0.6, .material = blocker } });
    try scene.build();
    // Start beneath the blocker: it hides the lamp for every sampled direction
    // the cosine lobe reaches at depth 1, and black albedo absorbs the rest.
    const down = Ray.init(Vec3.init(0, 1, 0), Vec3.init(0, -1, 0));
    const r = meanRadiance(&scene, down, 4, 2000, 2, true);
    try testing.expect(r.mean < 0.5 * 2.0 / 9.0 * 0.6);
}

test "light sampling and plain path tracing agree on a lit room" {
    var scene = Scene.init(testing.allocator);
    defer scene.deinit();
    scene.background = .{ .solid = Vec3.zero };
    const wall = try scene.addMaterial(.{ .lambertian = .{ .albedo = Vec3.init(0.7, 0.6, 0.5) } });
    const lamp = try scene.addMaterial(.{ .emissive = .{ .color = Vec3.splat(6) } });
    const ball = try scene.addMaterial(.{ .lambertian = .{ .albedo = Vec3.init(0.2, 0.5, 0.8) } });
    try scene.add(.{ .plane = .{ .point = Vec3.zero, .normal = Vec3.init(0, 1, 0), .material = wall } });
    try scene.addQuad(Vec3.init(-5, 0, -8), Vec3.init(10, 0, 0), Vec3.init(0, 6, 0), wall);
    // The light faces down (u x v = -y).
    try scene.addQuad(Vec3.init(-1, 4, -4), Vec3.init(2, 0, 0), Vec3.init(0, 0, 2), lamp);
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 1, -3), .radius = 1, .material = ball } });
    scene.camera = .{ .look_from = Vec3.init(0, 2, 3), .look_at = Vec3.init(0, 1.5, -3), .vfov = 60 };
    try scene.build();
    var opts = testOptions();
    opts.spp = 256;
    opts.max_depth = 6;
    opts.threads = 2;
    var with = try render(testing.allocator, &scene, opts);
    defer with.deinit(testing.allocator);
    opts.light_sampling = false;
    opts.spp = 1024;
    var without = try render(testing.allocator, &scene, opts);
    defer without.deinit(testing.allocator);
    var mw = Vec3.zero;
    var mo = Vec3.zero;
    for (with.pixels, without.pixels) |a, b| {
        mw = mw.add(a);
        mo = mo.add(b);
    }
    try testing.expect(mw.maxComponent() > 0);
    try testing.expect(mw.approxEq(mo, 0.05 * mo.maxComponent()));
}
