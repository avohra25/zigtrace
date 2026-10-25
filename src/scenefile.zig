//! Parser for the line-oriented `.txt` scene format.
//!
//! Each non-blank line is a directive; `#` starts a comment. Materials are
//! named and must be declared before shapes refer to them.
//!
//!     image 640 360
//!     samples 64
//!     depth 16
//!     camera from 0 1 5 to 0 0 0 up 0 1 0 fov 40 aperture 0.1 focus 5
//!     background sky 1 1 1  0.5 0.7 1.0      # or: background color r g b
//!     material ground lambertian 0.5 0.5 0.5
//!     material steel  metal 0.8 0.8 0.8 0.05  # optional fuzz
//!     material glass  dielectric 1.5
//!     material lamp   emissive 4 4 4
//!     sphere   0 1 0  1  glass               # negative radius = inward normals
//!     plane    0 0 0  0 1 0  ground
//!     triangle x0 y0 z0  x1 y1 z1  x2 y2 z2  steel
//!     quad     cx cy cz  ux uy uz  vx vy vz  lamp

const std = @import("std");
const Vec3 = @import("vec.zig").Vec3;
const Scene = @import("scene.zig").Scene;
const Material = @import("material.zig").Material;
const MaterialId = @import("geometry.zig").MaterialId;

pub const Diagnostic = struct {
    line: usize = 0,
    message: []const u8 = "",
};

pub const Error = error{SyntaxError} || std.mem.Allocator.Error || error{ TooManyPrimitives, UnboundedShape };

/// Parses `source` into a scene with its BVH built. On `error.SyntaxError`,
/// `diag` (if given) holds the offending line number and a message.
pub fn parse(allocator: std.mem.Allocator, source: []const u8, diag: ?*Diagnostic) Error!Scene {
    var scene = Scene.init(allocator);
    errdefer scene.deinit();
    var names = std.StringHashMap(MaterialId).init(allocator);
    defer names.deinit();

    var dummy: Diagnostic = .{};
    const d = diag orelse &dummy;

    var lines = std.mem.splitScalar(u8, source, '\n');
    var line_no: usize = 0;
    while (lines.next()) |raw| {
        line_no += 1;
        d.line = line_no;
        const line = if (std.mem.indexOfScalar(u8, raw, '#')) |i| raw[0..i] else raw;
        var p = LineParser{ .tokens = std.mem.tokenizeAny(u8, line, " \t\r"), .diag = d };
        const keyword = p.tokens.next() orelse continue;

        if (eql(keyword, "image")) {
            scene.settings.width = try p.positiveInt();
            scene.settings.height = try p.positiveInt();
        } else if (eql(keyword, "samples")) {
            scene.settings.spp = try p.positiveInt();
        } else if (eql(keyword, "depth")) {
            scene.settings.max_depth = try p.positiveInt();
        } else if (eql(keyword, "camera")) {
            try parseCamera(&p, &scene);
        } else if (eql(keyword, "background")) {
            const kind = try p.word();
            if (eql(kind, "color")) {
                scene.background = .{ .solid = try p.vec() };
            } else if (eql(kind, "sky")) {
                scene.background = .{ .sky = .{ .horizon = try p.vec(), .zenith = try p.vec() } };
            } else return p.fail("background must be 'color' or 'sky'");
        } else if (eql(keyword, "material")) {
            const name = try p.word();
            if (names.contains(name)) return p.fail("material already defined");
            const m = try parseMaterial(&p);
            try names.put(name, try scene.addMaterial(m));
        } else if (eql(keyword, "sphere")) {
            const center = try p.vec();
            const radius = try p.number();
            // A negative radius flips the normals, modelling a hollow bubble.
            if (radius == 0) return p.fail("sphere radius must be non-zero");
            try scene.add(.{ .sphere = .{ .center = center, .radius = radius, .material = try p.materialRef(names) } });
        } else if (eql(keyword, "plane")) {
            const point = try p.vec();
            const normal = try p.vec();
            if (normal.nearZero()) return p.fail("plane normal must be non-zero");
            try scene.add(.{ .plane = .{ .point = point, .normal = normal.normalize(), .material = try p.materialRef(names) } });
        } else if (eql(keyword, "triangle")) {
            const v0 = try p.vec();
            const v1 = try p.vec();
            const v2 = try p.vec();
            if (v1.sub(v0).cross(v2.sub(v0)).nearZero()) return p.fail("degenerate triangle");
            try scene.add(.{ .triangle = .{ .v0 = v0, .v1 = v1, .v2 = v2, .material = try p.materialRef(names) } });
        } else if (eql(keyword, "quad")) {
            const corner = try p.vec();
            const u = try p.vec();
            const v = try p.vec();
            if (u.cross(v).nearZero()) return p.fail("degenerate quad");
            try scene.addQuad(corner, u, v, try p.materialRef(names));
        } else {
            return p.fail("unknown directive");
        }
        try p.end();
    }
    d.* = .{};
    try scene.build();
    return scene;
}

fn parseCamera(p: *LineParser, scene: *Scene) Error!void {
    var cfg = scene.camera;
    var saw_from = false;
    var saw_to = false;
    while (p.tokens.peek() != null) {
        const key = try p.word();
        if (eql(key, "from")) {
            cfg.look_from = try p.vec();
            saw_from = true;
        } else if (eql(key, "to")) {
            cfg.look_at = try p.vec();
            saw_to = true;
        } else if (eql(key, "up")) {
            cfg.up = try p.vec();
        } else if (eql(key, "fov")) {
            cfg.vfov = try p.number();
            if (!(cfg.vfov > 0 and cfg.vfov < 180)) return p.fail("fov must be between 0 and 180 degrees");
        } else if (eql(key, "aperture")) {
            cfg.aperture = try p.number();
            if (cfg.aperture < 0) return p.fail("aperture must be non-negative");
        } else if (eql(key, "focus")) {
            const f = try p.number();
            if (!(f > 0)) return p.fail("focus distance must be positive");
            cfg.focus_dist = f;
        } else return p.fail("unknown camera parameter");
    }
    if (!saw_from or !saw_to) return p.fail("camera needs 'from' and 'to'");
    const view = cfg.look_from.sub(cfg.look_at);
    if (view.nearZero()) return p.fail("camera 'from' and 'to' must differ");
    if (cfg.up.cross(view).nearZero()) return p.fail("camera 'up' is parallel to the view direction");
    scene.camera = cfg;
}

fn parseMaterial(p: *LineParser) Error!Material {
    const kind = try p.word();
    if (eql(kind, "lambertian")) return .{ .lambertian = .{ .albedo = try p.color() } };
    if (eql(kind, "metal")) {
        const albedo = try p.color();
        const fuzz = if (p.tokens.peek() != null) try p.number() else 0;
        if (fuzz < 0 or fuzz > 1) return p.fail("metal fuzz must be in [0, 1]");
        return .{ .metal = .{ .albedo = albedo, .fuzz = fuzz } };
    }
    if (eql(kind, "dielectric")) {
        const ior = try p.number();
        if (!(ior > 0)) return p.fail("index of refraction must be positive");
        return .{ .dielectric = .{ .ior = ior } };
    }
    if (eql(kind, "emissive")) return .{ .emissive = .{ .color = try p.color() } };
    return p.fail("unknown material type");
}

const LineParser = struct {
    tokens: std.mem.TokenIterator(u8, .any),
    diag: *Diagnostic,

    fn fail(self: *LineParser, message: []const u8) error{SyntaxError} {
        self.diag.message = message;
        return error.SyntaxError;
    }

    fn word(self: *LineParser) error{SyntaxError}![]const u8 {
        return self.tokens.next() orelse self.fail("unexpected end of line");
    }

    fn number(self: *LineParser) error{SyntaxError}!f64 {
        const tok = try self.word();
        const v = std.fmt.parseFloat(f64, tok) catch return self.fail("expected a number");
        if (!std.math.isFinite(v)) return self.fail("number must be finite");
        return v;
    }

    fn positiveInt(self: *LineParser) error{SyntaxError}!usize {
        const tok = try self.word();
        const v = std.fmt.parseInt(usize, tok, 10) catch return self.fail("expected a positive integer");
        if (v == 0) return self.fail("expected a positive integer");
        return v;
    }

    fn vec(self: *LineParser) error{SyntaxError}!Vec3 {
        return Vec3.init(try self.number(), try self.number(), try self.number());
    }

    fn color(self: *LineParser) error{SyntaxError}!Vec3 {
        const c = try self.vec();
        if (c.x < 0 or c.y < 0 or c.z < 0) return self.fail("colour components must be non-negative");
        return c;
    }

    fn materialRef(self: *LineParser, names: std.StringHashMap(MaterialId)) error{SyntaxError}!MaterialId {
        const name = try self.word();
        return names.get(name) orelse self.fail("undefined material");
    }

    fn end(self: *LineParser) error{SyntaxError}!void {
        if (self.tokens.next() != null) return self.fail("unexpected extra tokens");
    }
};

fn eql(a: []const u8, b: []const u8) bool {
    return std.mem.eql(u8, a, b);
}

const testing = std.testing;

test "parses a complete scene" {
    const src =
        \\# a test scene
        \\image 320 200
        \\samples 8
        \\depth 5
        \\camera from 0 1 5 to 0 0 0 fov 35 aperture 0.2 focus 4.5
        \\background color 0.1 0.2 0.3
        \\material ground lambertian 0.5 0.5 0.5
        \\material steel metal 0.8 0.8 0.8 0.1
        \\material glass dielectric 1.5   # trailing comment
        \\material lamp emissive 4 4 4
        \\
        \\sphere 0 1 0 1 glass
        \\plane 0 0 0  0 2 0 ground
        \\triangle -1 0 -1  1 0 -1  0 2 -1 steel
        \\quad -1 3 -1  2 0 0  0 0 2 lamp
    ;
    var scene = try parse(testing.allocator, src, null);
    defer scene.deinit();
    try testing.expectEqual(@as(usize, 320), scene.settings.width);
    try testing.expectEqual(@as(usize, 200), scene.settings.height);
    try testing.expectEqual(@as(usize, 8), scene.settings.spp);
    try testing.expectEqual(@as(usize, 5), scene.settings.max_depth);
    try testing.expectEqual(@as(f64, 35), scene.camera.vfov);
    try testing.expectEqual(@as(?f64, 4.5), scene.camera.focus_dist);
    try testing.expectEqual(Vec3.init(0.1, 0.2, 0.3), scene.background.solid);
    try testing.expectEqual(@as(usize, 4), scene.materials.items.len);
    try testing.expectEqual(@as(f64, 0.1), scene.materials.items[1].metal.fuzz);
    try testing.expectEqual(@as(usize, 4), scene.shapes.items.len); // sphere, triangle, 2 quad halves
    try testing.expectEqual(@as(usize, 1), scene.planes.items.len);
    try testing.expectEqual(Vec3.init(0, 1, 0), scene.planes.items[0].plane.normal);
    try testing.expect(scene.bvh != null);
}

test "reports errors with line numbers" {
    const cases = [_]struct { src: []const u8, line: usize, msg: []const u8 }{
        .{ .src = "image 10", .line = 1, .msg = "unexpected end of line" },
        .{ .src = "\n\nsphere 0 0 0 1 nope", .line = 3, .msg = "undefined material" },
        .{ .src = "material a lambertian 1 1 1\nmaterial a dielectric 1.5", .line = 2, .msg = "material already defined" },
        .{ .src = "frobnicate", .line = 1, .msg = "unknown directive" },
        .{ .src = "samples 0", .line = 1, .msg = "expected a positive integer" },
        .{ .src = "camera from 0 0 0 to 0 0 0", .line = 1, .msg = "camera 'from' and 'to' must differ" },
        .{ .src = "camera from 0 0 0", .line = 1, .msg = "camera needs 'from' and 'to'" },
        .{ .src = "material m metal 1 1 1 2", .line = 1, .msg = "metal fuzz must be in [0, 1]" },
        .{ .src = "material m lambertian 1 x 1", .line = 1, .msg = "expected a number" },
        .{ .src = "material m lambertian 1 1 1\ntriangle 0 0 0 1 1 1 2 2 2 m", .line = 2, .msg = "degenerate triangle" },
        .{ .src = "depth 3 4", .line = 1, .msg = "unexpected extra tokens" },
        .{ .src = "background gradient", .line = 1, .msg = "background must be 'color' or 'sky'" },
    };
    for (cases) |c| {
        var diag: Diagnostic = .{};
        try testing.expectError(error.SyntaxError, parse(testing.allocator, c.src, &diag));
        try testing.expectEqual(c.line, diag.line);
        try testing.expectEqualStrings(c.msg, diag.message);
    }
}

test "example scenes parse" {
    inline for (.{ "examples/spheres.txt", "examples/cornell.txt", "examples/glass.txt" }) |path| {
        var scene = try parse(testing.allocator, @embedFile(path), null);
        defer scene.deinit();
        try testing.expect(scene.primitiveCount() > 0);
    }
}
