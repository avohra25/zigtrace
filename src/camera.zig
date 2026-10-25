//! Thin-lens perspective camera with depth of field.

const std = @import("std");
const Vec3 = @import("vec.zig").Vec3;
const Ray = @import("ray.zig").Ray;
const Rng = @import("rng.zig").Rng;

pub const CameraConfig = struct {
    look_from: Vec3 = Vec3.init(0, 0, 0),
    look_at: Vec3 = Vec3.init(0, 0, -1),
    up: Vec3 = Vec3.init(0, 1, 0),
    /// Vertical field of view in degrees.
    vfov: f64 = 60,
    /// Lens diameter; zero gives a pinhole camera with everything in focus.
    aperture: f64 = 0,
    /// Distance to the plane of perfect focus; defaults to |look_at - look_from|.
    focus_dist: ?f64 = null,
};

pub const Camera = struct {
    origin: Vec3,
    upper_left: Vec3,
    horizontal: Vec3,
    vertical: Vec3,
    u: Vec3,
    v: Vec3,
    lens_radius: f64,

    pub fn init(cfg: CameraConfig, aspect: f64) Camera {
        const theta = std.math.degreesToRadians(cfg.vfov);
        const half_h = @tan(theta / 2);
        const viewport_h = 2.0 * half_h;
        const viewport_w = aspect * viewport_h;

        const w = cfg.look_from.sub(cfg.look_at).normalize();
        const u = cfg.up.cross(w).normalize();
        const v = w.cross(u);
        const focus = cfg.focus_dist orelse cfg.look_from.sub(cfg.look_at).length();

        const horizontal = u.scale(viewport_w * focus);
        const vertical = v.scale(-viewport_h * focus); // image rows go downward
        const upper_left = cfg.look_from
            .sub(w.scale(focus))
            .sub(horizontal.scale(0.5))
            .sub(vertical.scale(0.5));
        return .{
            .origin = cfg.look_from,
            .upper_left = upper_left,
            .horizontal = horizontal,
            .vertical = vertical,
            .u = u,
            .v = v,
            .lens_radius = cfg.aperture / 2,
        };
    }

    /// Primary ray through normalised image coordinates (`s` left to right,
    /// `t` top to bottom, both in [0, 1]), with a random lens sample.
    pub fn ray(self: Camera, s: f64, t: f64, rng: *Rng) Ray {
        var origin = self.origin;
        if (self.lens_radius > 0) {
            const d = rng.inUnitDisk().scale(self.lens_radius);
            origin = origin.add(self.u.scale(d.x)).add(self.v.scale(d.y));
        }
        const target = self.upper_left.add(self.horizontal.scale(s)).add(self.vertical.scale(t));
        return Ray.init(origin, target.sub(origin).normalize());
    }
};

const testing = std.testing;

test "centre ray points at look_at" {
    var rng = Rng.init(0);
    const cam = Camera.init(.{ .look_from = Vec3.init(1, 2, 3), .look_at = Vec3.init(4, 2, -1) }, 16.0 / 9.0);
    const r = cam.ray(0.5, 0.5, &rng);
    try testing.expect(r.dir.approxEq(Vec3.init(3, 0, -4).normalize(), 1e-12));
    try testing.expectEqual(Vec3.init(1, 2, 3), r.origin);
}

test "image orientation and field of view" {
    var rng = Rng.init(0);
    const cam = Camera.init(.{ .vfov = 90 }, 1.0);
    const top_left = cam.ray(0, 0, &rng).dir;
    try testing.expect(top_left.x < 0 and top_left.y > 0);
    const top = cam.ray(0.5, 0, &rng).dir;
    // 90 degree vfov: the top edge is 45 degrees above the view axis.
    try testing.expectApproxEqAbs(@as(f64, 45), std.math.radiansToDegrees(std.math.atan2(top.y, -top.z)), 1e-9);
}

test "depth of field rays converge on the focus plane" {
    var rng = Rng.init(9);
    const cam = Camera.init(.{
        .look_from = Vec3.init(0, 0, 5),
        .look_at = Vec3.zero,
        .aperture = 0.8,
        .focus_dist = 5,
    }, 1.5);
    var spread: f64 = 0;
    for (0..200) |_| {
        const r = cam.ray(0.5, 0.5, &rng);
        spread += @abs(r.origin.x) + @abs(r.origin.y);
        // Every lens sample passes through the focus point.
        const hit = r.at((0 - r.origin.z) / r.dir.z);
        try testing.expect(hit.approxEq(Vec3.zero, 1e-9));
        try testing.expect(r.origin.sub(Vec3.init(0, 0, 5)).length() <= 0.4 + 1e-12);
    }
    try testing.expect(spread > 0);
}
