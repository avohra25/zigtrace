//! Command-line front end (work in progress).

const std = @import("std");
const zt = @import("zigtrace");

pub fn main() !void {
    try std.io.getStdOut().writer().writeAll("zigtrace 0.1.0-dev: the CLI is not wired up yet; run `zig build test`.\n");
}

test "library module is importable" {
    _ = zt.Vec3.init(0, 0, 0);
}
