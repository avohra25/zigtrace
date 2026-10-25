//! Floating-point HDR framebuffer, tone mapping, gamma and PPM output.

const std = @import("std");
const Vec3 = @import("vec.zig").Vec3;

pub const Tonemap = enum {
    /// Clamp linear radiance to [0, 1].
    none,
    /// Reinhard operator applied per channel: c / (1 + c).
    reinhard,
    /// Krzysztof Narkowicz's fitted ACES filmic curve.
    aces,

    pub fn apply(self: Tonemap, c: f64) f64 {
        const x = @max(c, 0.0);
        return switch (self) {
            .none => x,
            .reinhard => x / (1.0 + x),
            .aces => (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14),
        };
    }
};

pub const Output = struct {
    tonemap: Tonemap = .none,
    /// Display gamma; 1.0 leaves values linear.
    gamma: f64 = 2.2,
    /// Exposure multiplier applied before tone mapping.
    exposure: f64 = 1.0,

    /// Maps a linear radiance value to an 8-bit display value.
    pub fn encode(self: Output, linear: f64) u8 {
        const v = if (std.math.isNan(linear)) 0.0 else linear;
        const mapped = std.math.clamp(self.tonemap.apply(v * self.exposure), 0.0, 1.0);
        const corrected = std.math.pow(f64, mapped, 1.0 / self.gamma);
        return @intFromFloat(@round(std.math.clamp(corrected, 0.0, 1.0) * 255.0));
    }
};

pub const Image = struct {
    width: usize,
    height: usize,
    pixels: []Vec3,

    pub fn init(allocator: std.mem.Allocator, width: usize, height: usize) !Image {
        const pixels = try allocator.alloc(Vec3, width * height);
        @memset(pixels, Vec3.zero);
        return .{ .width = width, .height = height, .pixels = pixels };
    }

    pub fn deinit(self: *Image, allocator: std.mem.Allocator) void {
        allocator.free(self.pixels);
        self.* = undefined;
    }

    pub fn get(self: Image, x: usize, y: usize) Vec3 {
        return self.pixels[y * self.width + x];
    }

    pub fn set(self: *Image, x: usize, y: usize, c: Vec3) void {
        self.pixels[y * self.width + x] = c;
    }

    /// Converts the image to packed 8-bit RGB, row-major, top row first.
    pub fn toRgb8(self: Image, allocator: std.mem.Allocator, out: Output) ![]u8 {
        const buf = try allocator.alloc(u8, self.pixels.len * 3);
        for (self.pixels, 0..) |p, i| {
            buf[3 * i + 0] = out.encode(p.x);
            buf[3 * i + 1] = out.encode(p.y);
            buf[3 * i + 2] = out.encode(p.z);
        }
        return buf;
    }
};

/// Writes a binary (P6) PPM of packed RGB8 data.
pub fn writePpm(writer: anytype, width: usize, height: usize, rgb: []const u8) !void {
    std.debug.assert(rgb.len == width * height * 3);
    try writer.print("P6\n{d} {d}\n255\n", .{ width, height });
    try writer.writeAll(rgb);
}

const testing = std.testing;

test "tone mapping curves" {
    try testing.expectEqual(@as(f64, 0.5), Tonemap.reinhard.apply(1.0));
    try testing.expectEqual(@as(f64, 0), Tonemap.none.apply(-3.0));
    // ACES maps zero to zero and saturates near one.
    try testing.expectApproxEqAbs(@as(f64, 0), Tonemap.aces.apply(0), 1e-12);
    try testing.expect(Tonemap.aces.apply(100) > 0.99);
    // All curves are monotonic.
    inline for (.{ Tonemap.none, Tonemap.reinhard, Tonemap.aces }) |tm| {
        var prev: f64 = -1;
        var x: f64 = 0;
        while (x < 4) : (x += 0.05) {
            const y = tm.apply(x);
            try testing.expect(y >= prev);
            prev = y;
        }
    }
}

test "encode applies gamma and clamps" {
    const lin = Output{ .gamma = 1.0 };
    try testing.expectEqual(@as(u8, 0), lin.encode(-1));
    try testing.expectEqual(@as(u8, 255), lin.encode(7));
    try testing.expectEqual(@as(u8, 128), lin.encode(0.5));
    try testing.expectEqual(@as(u8, 0), lin.encode(std.math.nan(f64)));
    const srgbish = Output{ .gamma = 2.2 };
    // 0.5^(1/2.2) ~= 0.7297 -> 186
    try testing.expectEqual(@as(u8, 186), srgbish.encode(0.5));
}

test "ppm output" {
    var img = try Image.init(testing.allocator, 2, 1);
    defer img.deinit(testing.allocator);
    img.set(0, 0, Vec3.init(1, 0, 0));
    img.set(1, 0, Vec3.init(0, 0, 1));
    const rgb = try img.toRgb8(testing.allocator, .{});
    defer testing.allocator.free(rgb);

    var buf = std.ArrayList(u8).init(testing.allocator);
    defer buf.deinit();
    try writePpm(buf.writer(), img.width, img.height, rgb);
    try testing.expectEqualSlices(u8, "P6\n2 1\n255\n\xff\x00\x00\x00\x00\xff", buf.items);
}
