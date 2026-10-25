//! Monte Carlo path tracing integrator and multithreaded tile renderer.

const std = @import("std");
const Vec3 = @import("vec.zig").Vec3;
const Ray = @import("ray.zig").Ray;
const Rng = @import("rng.zig").Rng;
const Image = @import("image.zig").Image;
const Scene = @import("scene.zig").Scene;
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
};

/// Offset that keeps secondary rays from re-hitting their own surface.
const ray_epsilon = 1e-4;
/// Paths shorter than this are never terminated by Russian roulette.
const roulette_depth = 4;

/// Estimates incoming radiance along `ray` with one random path.
pub fn trace(scene: *const Scene, initial: Ray, rng: *Rng, max_depth: usize) Vec3 {
    var radiance = Vec3.zero;
    var throughput = Vec3.one;
    var r = initial;
    var depth: usize = 0;
    while (depth < max_depth) : (depth += 1) {
        const hit = scene.intersect(r, ray_epsilon, std.math.inf(f64)) orelse {
            return radiance.add(throughput.mul(scene.background.radiance(r.dir)));
        };
        const mat = scene.material(hit.material);
        radiance = radiance.add(throughput.mul(mat.emitted(hit)));
        const s = mat.scatter(r, hit, rng) orelse break;
        throughput = throughput.mul(s.attenuation);
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
                    const c = trace(ctx.scene, ctx.camera.ray(s, t, &rng), &rng, ctx.opts.max_depth);
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
