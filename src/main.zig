//! Command-line front end: `zigtrace render scene.txt -o out.png --spp 64`.

const std = @import("std");
const zt = @import("zigtrace");

const version = "0.1.0";

const usage =
    \\zigtrace - a physically based Monte Carlo path tracer
    \\
    \\Usage:
    \\  zigtrace render <scene.txt> [options]   render a scene to PPM or PNG
    \\  zigtrace info <scene.txt>               parse a scene and print statistics
    \\  zigtrace help | --help                  show this message
    \\  zigtrace version | --version            print the version
    \\
    \\Render options (override values from the scene file):
    \\  -o, --output <path>      output file, .ppm or .png (default: out.ppm)
    \\  --spp <n>                samples per pixel
    \\  --width <n>              image width in pixels
    \\  --height <n>             image height in pixels
    \\  --depth <n>              maximum path length (bounces)
    \\  --threads <n>            worker threads (default: all CPUs)
    \\  --seed <n>               RNG seed; output is deterministic per seed (default: 0)
    \\  --tile <n>               tile size in pixels (default: 16)
    \\  --tonemap <op>           none | reinhard | aces (default: aces)
    \\  --exposure <x>           linear exposure multiplier (default: 1)
    \\  --gamma <g>              display gamma (default: 2.2)
    \\  -q, --quiet              do not print render statistics
    \\
;

pub const Format = enum { ppm, png };

pub const RenderArgs = struct {
    scene_path: []const u8,
    output: []const u8 = "out.ppm",
    format: Format = .ppm,
    spp: ?usize = null,
    width: ?usize = null,
    height: ?usize = null,
    depth: ?usize = null,
    threads: ?usize = null,
    seed: u64 = 0,
    tile: usize = 16,
    output_opts: zt.image.Output = .{ .tonemap = .aces },
    quiet: bool = false,
};

pub const Command = union(enum) {
    help,
    version,
    info: []const u8,
    render: RenderArgs,
};

pub const ArgError = struct {
    message: []const u8 = "",
    arg: []const u8 = "",
};

pub fn parseArgs(args: []const []const u8, err: *ArgError) error{InvalidArgs}!Command {
    if (args.len == 0) return .help;
    const cmd = args[0];
    if (eql(cmd, "help") or eql(cmd, "--help") or eql(cmd, "-h")) return .help;
    if (eql(cmd, "version") or eql(cmd, "--version")) return .version;
    if (eql(cmd, "info")) {
        if (args.len != 2) return fail(err, "info expects exactly one scene file", cmd);
        return .{ .info = args[1] };
    }
    if (!eql(cmd, "render")) return fail(err, "unknown command", cmd);

    var out: RenderArgs = .{ .scene_path = "" };
    var i: usize = 1;
    while (i < args.len) : (i += 1) {
        const a = args[i];
        if (eql(a, "-q") or eql(a, "--quiet")) {
            out.quiet = true;
            continue;
        }
        if (a.len > 0 and a[0] == '-') {
            if (i + 1 >= args.len) return fail(err, "missing value for option", a);
            i += 1;
            const v = args[i];
            if (eql(a, "-o") or eql(a, "--output")) {
                out.output = v;
            } else if (eql(a, "--spp")) {
                out.spp = try positive(v, err, a);
            } else if (eql(a, "--width")) {
                out.width = try positive(v, err, a);
            } else if (eql(a, "--height")) {
                out.height = try positive(v, err, a);
            } else if (eql(a, "--depth")) {
                out.depth = try positive(v, err, a);
            } else if (eql(a, "--threads")) {
                out.threads = try positive(v, err, a);
            } else if (eql(a, "--tile")) {
                out.tile = try positive(v, err, a);
            } else if (eql(a, "--seed")) {
                out.seed = std.fmt.parseInt(u64, v, 10) catch return fail(err, "expected a non-negative integer", a);
            } else if (eql(a, "--tonemap")) {
                out.output_opts.tonemap = std.meta.stringToEnum(zt.image.Tonemap, v) orelse
                    return fail(err, "tonemap must be none, reinhard or aces", v);
            } else if (eql(a, "--gamma")) {
                out.output_opts.gamma = try positiveFloat(v, err, a);
            } else if (eql(a, "--exposure")) {
                out.output_opts.exposure = try positiveFloat(v, err, a);
            } else return fail(err, "unknown option", a);
            continue;
        }
        if (out.scene_path.len != 0) return fail(err, "more than one scene file given", a);
        out.scene_path = a;
    }
    if (out.scene_path.len == 0) return fail(err, "render needs a scene file", cmd);
    out.format = formatFor(out.output) orelse return fail(err, "output must end in .ppm or .png", out.output);
    return .{ .render = out };
}

fn formatFor(path: []const u8) ?Format {
    const ext = std.fs.path.extension(path);
    if (std.ascii.eqlIgnoreCase(ext, ".ppm")) return .ppm;
    if (std.ascii.eqlIgnoreCase(ext, ".png")) return .png;
    return null;
}

fn positive(v: []const u8, err: *ArgError, opt: []const u8) error{InvalidArgs}!usize {
    const n = std.fmt.parseInt(usize, v, 10) catch return fail(err, "expected a positive integer", opt);
    if (n == 0) return fail(err, "expected a positive integer", opt);
    return n;
}

fn positiveFloat(v: []const u8, err: *ArgError, opt: []const u8) error{InvalidArgs}!f64 {
    const x = std.fmt.parseFloat(f64, v) catch return fail(err, "expected a number", opt);
    if (!(x > 0) or !std.math.isFinite(x)) return fail(err, "expected a positive number", opt);
    return x;
}

fn fail(err: *ArgError, message: []const u8, arg: []const u8) error{InvalidArgs} {
    err.* = .{ .message = message, .arg = arg };
    return error.InvalidArgs;
}

fn eql(a: []const u8, b: []const u8) bool {
    return std.mem.eql(u8, a, b);
}

pub fn main() !void {
    var gpa: std.heap.GeneralPurposeAllocator(.{}) = .init;
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const argv = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, argv);

    const stdout = std.io.getStdOut().writer();
    const stderr = std.io.getStdErr().writer();

    var arg_err: ArgError = .{};
    const cmd = parseArgs(argv[1..], &arg_err) catch {
        try stderr.print("zigtrace: {s}: '{s}'\n\n{s}", .{ arg_err.message, arg_err.arg, usage });
        std.process.exit(2);
    };

    switch (cmd) {
        .help => try stdout.writeAll(usage),
        .version => try stdout.print("zigtrace {s}\n", .{version}),
        .info => |path| {
            var scene = loadScene(allocator, path, stderr) orelse std.process.exit(1);
            defer scene.deinit();
            try printInfo(stdout, path, &scene);
        },
        .render => |args| {
            var scene = loadScene(allocator, args.scene_path, stderr) orelse std.process.exit(1);
            defer scene.deinit();
            runRender(allocator, &scene, args, stderr) catch |e| {
                try stderr.print("zigtrace: render failed: {s}\n", .{@errorName(e)});
                std.process.exit(1);
            };
        },
    }
}

fn loadScene(allocator: std.mem.Allocator, path: []const u8, stderr: anytype) ?zt.Scene {
    const source = std.fs.cwd().readFileAlloc(allocator, path, 64 << 20) catch |e| {
        stderr.print("zigtrace: cannot read '{s}': {s}\n", .{ path, @errorName(e) }) catch {};
        return null;
    };
    defer allocator.free(source);
    var diag: zt.scenefile.Diagnostic = .{};
    return zt.scenefile.parse(allocator, source, &diag) catch |e| {
        switch (e) {
            error.SyntaxError => stderr.print("{s}:{d}: error: {s}\n", .{ path, diag.line, diag.message }) catch {},
            else => stderr.print("zigtrace: failed to load '{s}': {s}\n", .{ path, @errorName(e) }) catch {},
        }
        return null;
    };
}

fn printInfo(out: anytype, path: []const u8, scene: *const zt.Scene) !void {
    const bvh = scene.bvh.?;
    try out.print(
        \\scene:       {s}
        \\image:       {d}x{d}, {d} spp, depth {d}
        \\materials:   {d}
        \\primitives:  {d} bounded + {d} planes
        \\bvh:         {d} nodes, depth {d}
        \\
    , .{
        path,
        scene.settings.width,
        scene.settings.height,
        scene.settings.spp,
        scene.settings.max_depth,
        scene.materials.items.len,
        scene.shapes.items.len,
        scene.planes.items.len,
        bvh.nodes.len,
        bvh.depth(),
    });
}

fn runRender(allocator: std.mem.Allocator, scene: *const zt.Scene, args: RenderArgs, stderr: anytype) !void {
    const opts = zt.render.Options{
        .width = args.width orelse scene.settings.width,
        .height = args.height orelse scene.settings.height,
        .spp = args.spp orelse scene.settings.spp,
        .max_depth = args.depth orelse scene.settings.max_depth,
        .threads = args.threads orelse (std.Thread.getCpuCount() catch 1),
        .seed = args.seed,
        .tile_size = args.tile,
    };

    var timer = try std.time.Timer.start();
    var img = try zt.render.render(allocator, scene, opts);
    defer img.deinit(allocator);
    const elapsed_ns = timer.read();

    const rgb = try img.toRgb8(allocator, args.output_opts);
    defer allocator.free(rgb);

    var file = try std.fs.cwd().createFile(args.output, .{});
    defer file.close();
    var buffered = std.io.bufferedWriter(file.writer());
    switch (args.format) {
        .ppm => try zt.image.writePpm(buffered.writer(), img.width, img.height, rgb),
        .png => try zt.png.writePng(buffered.writer(), img.width, img.height, rgb),
    }
    try buffered.flush();

    if (!args.quiet) {
        const secs = @as(f64, @floatFromInt(elapsed_ns)) / std.time.ns_per_s;
        const samples = @as(f64, @floatFromInt(opts.width * opts.height * opts.spp));
        try stderr.print("rendered {d}x{d} @ {d} spp on {d} thread(s) in {d:.2}s ({d:.2} Msamples/s) -> {s}\n", .{
            opts.width, opts.height, opts.spp, opts.threads, secs, samples / secs / 1e6, args.output,
        });
    }
}

const testing = std.testing;

fn parseOk(args: []const []const u8) !Command {
    var err: ArgError = .{};
    return parseArgs(args, &err);
}

fn expectArgError(args: []const []const u8, message: []const u8) !void {
    var err: ArgError = .{};
    try testing.expectError(error.InvalidArgs, parseArgs(args, &err));
    try testing.expectEqualStrings(message, err.message);
}

test "parse render command" {
    const cmd = try parseOk(&.{ "render", "scene.txt", "-o", "img.PNG", "--spp", "64", "--threads", "3", "--seed", "7", "--tonemap", "reinhard", "--gamma", "2.4", "-q" });
    const r = cmd.render;
    try testing.expectEqualStrings("scene.txt", r.scene_path);
    try testing.expectEqualStrings("img.PNG", r.output);
    try testing.expectEqual(Format.png, r.format);
    try testing.expectEqual(@as(?usize, 64), r.spp);
    try testing.expectEqual(@as(?usize, 3), r.threads);
    try testing.expectEqual(@as(u64, 7), r.seed);
    try testing.expectEqual(zt.image.Tonemap.reinhard, r.output_opts.tonemap);
    try testing.expectEqual(@as(f64, 2.4), r.output_opts.gamma);
    try testing.expect(r.quiet);
    try testing.expectEqual(@as(?usize, null), r.width);
}

test "defaults and simple commands" {
    const r = (try parseOk(&.{ "render", "s.txt" })).render;
    try testing.expectEqualStrings("out.ppm", r.output);
    try testing.expectEqual(Format.ppm, r.format);
    try testing.expectEqual(zt.image.Tonemap.aces, r.output_opts.tonemap);
    try testing.expectEqual(Command.help, try parseOk(&.{}));
    try testing.expectEqual(Command.version, try parseOk(&.{"--version"}));
    try testing.expectEqualStrings("a.txt", (try parseOk(&.{ "info", "a.txt" })).info);
}

test "argument errors" {
    try expectArgError(&.{"paint"}, "unknown command");
    try expectArgError(&.{"render"}, "render needs a scene file");
    try expectArgError(&.{ "render", "a.txt", "b.txt" }, "more than one scene file given");
    try expectArgError(&.{ "render", "a.txt", "--spp" }, "missing value for option");
    try expectArgError(&.{ "render", "a.txt", "--spp", "0" }, "expected a positive integer");
    try expectArgError(&.{ "render", "a.txt", "--bogus", "1" }, "unknown option");
    try expectArgError(&.{ "render", "a.txt", "-o", "x.jpg" }, "output must end in .ppm or .png");
    try expectArgError(&.{ "render", "a.txt", "--tonemap", "filmic" }, "tonemap must be none, reinhard or aces");
    try expectArgError(&.{ "render", "a.txt", "--gamma", "-1" }, "expected a positive number");
}
