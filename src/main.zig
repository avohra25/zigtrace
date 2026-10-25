//! Command-line front end (work in progress): renders a built-in demo scene.

const std = @import("std");
const zt = @import("zigtrace");
const Vec3 = zt.Vec3;

pub fn main() !void {
    var gpa: std.heap.GeneralPurposeAllocator(.{}) = .init;
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var scene = zt.Scene.init(allocator);
    defer scene.deinit();
    try buildDemo(&scene);

    const threads = std.Thread.getCpuCount() catch 1;
    var img = try zt.render.render(allocator, &scene, .{
        .width = 480,
        .height = 270,
        .spp = 32,
        .max_depth = 16,
        .threads = threads,
    });
    defer img.deinit(allocator);

    const rgb = try img.toRgb8(allocator, .{ .tonemap = .aces });
    defer allocator.free(rgb);
    var file = try std.fs.cwd().createFile("demo.ppm", .{});
    defer file.close();
    var buffered = std.io.bufferedWriter(file.writer());
    try zt.image.writePpm(buffered.writer(), img.width, img.height, rgb);
    try buffered.flush();
    try std.io.getStdErr().writer().print("wrote demo.ppm using {d} thread(s)\n", .{threads});
}

pub fn buildDemo(scene: *zt.Scene) !void {
    const ground = try scene.addMaterial(.{ .lambertian = .{ .albedo = Vec3.init(0.55, 0.55, 0.5) } });
    const glass = try scene.addMaterial(.{ .dielectric = .{ .ior = 1.5 } });
    const copper = try scene.addMaterial(.{ .metal = .{ .albedo = Vec3.init(0.95, 0.64, 0.54), .fuzz = 0.25 } });
    const blue = try scene.addMaterial(.{ .lambertian = .{ .albedo = Vec3.init(0.1, 0.2, 0.6) } });
    try scene.add(.{ .plane = .{ .point = Vec3.zero, .normal = Vec3.init(0, 1, 0), .material = ground } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(0, 0.6, 0), .radius = 0.6, .material = glass } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(-1.35, 0.45, -0.4), .radius = 0.45, .material = copper } });
    try scene.add(.{ .sphere = .{ .center = Vec3.init(1.35, 0.45, -0.4), .radius = 0.45, .material = blue } });
    scene.camera = .{
        .look_from = Vec3.init(0, 1.2, 4.5),
        .look_at = Vec3.init(0, 0.6, 0),
        .vfov = 35,
        .aperture = 0.1,
        .focus_dist = 4.4,
    };
    try scene.build();
}

test "demo scene builds and renders" {
    var scene = zt.Scene.init(std.testing.allocator);
    defer scene.deinit();
    try buildDemo(&scene);
    var img = try zt.render.render(std.testing.allocator, &scene, .{ .width = 8, .height = 6, .spp = 2, .max_depth = 4, .threads = 2 });
    defer img.deinit(std.testing.allocator);
    for (img.pixels) |p| try std.testing.expect(p.isFinite());
}
