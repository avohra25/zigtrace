//! Bounding volume hierarchy built with a binned surface-area heuristic and
//! traversed iteratively, front-to-back.
//!
//! Nodes are stored in a flat array; the two children of an interior node are
//! always adjacent, so a node only needs the index of its left child.

const std = @import("std");
const Vec3 = @import("vec.zig").Vec3;
const Ray = @import("ray.zig").Ray;
const aabb = @import("aabb.zig");
const Aabb = aabb.Aabb;
const geometry = @import("geometry.zig");
const Shape = geometry.Shape;
const Hit = geometry.Hit;

const bin_count = 12;
const max_leaf_size = 4;
const max_depth = 64;
/// Relative cost of a traversal step versus a primitive intersection.
const traversal_cost = 1.0;

pub const Node = struct {
    bounds: Aabb,
    /// Leaf: first index into `Bvh.indices`. Interior: index of left child
    /// (the right child is `first + 1`).
    first: u32,
    /// Number of primitives for a leaf, zero for an interior node.
    count: u32,

    pub fn isLeaf(self: Node) bool {
        return self.count > 0;
    }
};

pub const Bvh = struct {
    nodes: []Node,
    /// Permutation of shape indices; leaves reference contiguous ranges.
    indices: []u32,

    /// Builds a hierarchy over `shapes`, all of which must be bounded.
    pub fn build(allocator: std.mem.Allocator, shapes: []const Shape) !Bvh {
        const n = shapes.len;
        if (n == 0) return .{ .nodes = &.{}, .indices = &.{} };
        if (n > std.math.maxInt(u32) / 2) return error.TooManyPrimitives;

        const indices = try allocator.alloc(u32, n);
        errdefer allocator.free(indices);
        const boxes = try allocator.alloc(Aabb, n);
        defer allocator.free(boxes);
        const centroids = try allocator.alloc(Vec3, n);
        defer allocator.free(centroids);

        for (shapes, 0..) |s, i| {
            indices[i] = @intCast(i);
            boxes[i] = s.bounds() orelse return error.UnboundedShape;
            centroids[i] = boxes[i].centroid();
        }

        var nodes = try std.ArrayList(Node).initCapacity(allocator, 2 * n - 1);
        errdefer nodes.deinit();
        nodes.appendAssumeCapacity(.{ .bounds = Aabb.empty, .first = 0, .count = @intCast(n) });

        var builder = Builder{ .nodes = &nodes, .indices = indices, .boxes = boxes, .centroids = centroids };
        builder.subdivide(0, 0);

        return .{ .nodes = try nodes.toOwnedSlice(), .indices = indices };
    }

    pub fn deinit(self: *Bvh, allocator: std.mem.Allocator) void {
        allocator.free(self.nodes);
        allocator.free(self.indices);
        self.* = undefined;
    }

    pub fn bounds(self: Bvh) Aabb {
        return if (self.nodes.len == 0) Aabb.empty else self.nodes[0].bounds;
    }

    /// Closest hit among `shapes` (the same slice the BVH was built from).
    pub fn intersect(self: Bvh, shapes: []const Shape, r: Ray, t_min: f64, t_max: f64) ?Hit {
        if (self.nodes.len == 0) return null;
        const inv = aabb.inverse(r.dir);
        var closest = t_max;
        var best: ?Hit = null;

        const Entry = struct { node: u32, t: f64 };
        var stack: [max_depth + 2]Entry = undefined;
        var sp: usize = 0;
        if (self.nodes[0].bounds.hit(r, inv, t_min, closest)) |t| {
            stack[0] = .{ .node = 0, .t = t };
            sp = 1;
        }

        while (sp > 0) {
            sp -= 1;
            const entry = stack[sp];
            if (entry.t > closest) continue;
            const node = self.nodes[entry.node];
            if (node.isLeaf()) {
                for (self.indices[node.first..][0..node.count]) |idx| {
                    if (shapes[idx].intersect(r, t_min, closest)) |h| {
                        closest = h.t;
                        best = h;
                    }
                }
                continue;
            }
            const l = node.first;
            const tl = self.nodes[l].bounds.hit(r, inv, t_min, closest);
            const tr = self.nodes[l + 1].bounds.hit(r, inv, t_min, closest);
            // Push the farther child first so the nearer one is popped next.
            if (tl != null and tr != null) {
                const near_left = tl.? <= tr.?;
                stack[sp] = if (near_left) .{ .node = l + 1, .t = tr.? } else .{ .node = l, .t = tl.? };
                stack[sp + 1] = if (near_left) .{ .node = l, .t = tl.? } else .{ .node = l + 1, .t = tr.? };
                sp += 2;
            } else if (tl) |t| {
                stack[sp] = .{ .node = l, .t = t };
                sp += 1;
            } else if (tr) |t| {
                stack[sp] = .{ .node = l + 1, .t = t };
                sp += 1;
            }
        }
        return best;
    }

    pub fn depth(self: Bvh) usize {
        if (self.nodes.len == 0) return 0;
        return self.depthFrom(0);
    }

    fn depthFrom(self: Bvh, i: u32) usize {
        const node = self.nodes[i];
        if (node.isLeaf()) return 1;
        return 1 + @max(self.depthFrom(node.first), self.depthFrom(node.first + 1));
    }
};

const Builder = struct {
    nodes: *std.ArrayList(Node),
    indices: []u32,
    boxes: []const Aabb,
    centroids: []const Vec3,

    const Split = struct { axis: usize, bin: usize, cost: f64 };

    fn subdivide(self: *Builder, node_idx: usize, level: usize) void {
        const node = &self.nodes.items[node_idx];
        const first = node.first;
        const count = node.count;
        const range = self.indices[first..][0..count];

        var bounds = Aabb.empty;
        var cbounds = Aabb.empty;
        for (range) |i| {
            bounds = bounds.merge(self.boxes[i]);
            cbounds = cbounds.include(self.centroids[i]);
        }
        node.bounds = bounds;

        if (count <= 1 or level >= max_depth) return;

        const leaf_cost: f64 = @floatFromInt(count);
        const split = self.findSplit(range, bounds, cbounds) orelse {
            // All centroids coincide: binning cannot separate them.
            if (count <= max_leaf_size) return;
            return self.splitMedian(node_idx, range, cbounds, level);
        };
        if (count <= max_leaf_size and split.cost >= leaf_cost) return;

        // Partition `range` in place by bin.
        const axis = split.axis;
        const lo = cbounds.min.get(axis);
        const extent = cbounds.max.get(axis) - lo;
        var mid: usize = 0;
        for (range, 0..) |idx, j| {
            if (binOf(self.centroids[idx].get(axis), lo, extent) <= split.bin) {
                std.mem.swap(u32, &range[mid], &range[j]);
                mid += 1;
            }
        }
        std.debug.assert(mid > 0 and mid < count);
        self.makeChildren(node_idx, first, @intCast(mid), count, level);
    }

    fn makeChildren(self: *Builder, node_idx: usize, first: u32, left_count: u32, count: u32, level: usize) void {
        const left: u32 = @intCast(self.nodes.items.len);
        self.nodes.appendAssumeCapacity(.{ .bounds = Aabb.empty, .first = first, .count = left_count });
        self.nodes.appendAssumeCapacity(.{ .bounds = Aabb.empty, .first = first + left_count, .count = count - left_count });
        self.nodes.items[node_idx].first = left;
        self.nodes.items[node_idx].count = 0;
        self.subdivide(left, level + 1);
        self.subdivide(left + 1, level + 1);
    }

    fn splitMedian(self: *Builder, node_idx: usize, range: []u32, cbounds: Aabb, level: usize) void {
        const axis = cbounds.longestAxis();
        const Ctx = struct {
            centroids: []const Vec3,
            axis: usize,
            fn less(ctx: @This(), a: u32, b: u32) bool {
                return ctx.centroids[a].get(ctx.axis) < ctx.centroids[b].get(ctx.axis);
            }
        };
        std.mem.sort(u32, range, Ctx{ .centroids = self.centroids, .axis = axis }, Ctx.less);
        const node = self.nodes.items[node_idx];
        self.makeChildren(node_idx, node.first, node.count / 2, node.count, level);
    }

    fn findSplit(self: *Builder, range: []const u32, bounds: Aabb, cbounds: Aabb) ?Split {
        var best: ?Split = null;
        const parent_area = bounds.surfaceArea();
        if (parent_area <= 0) return null;

        for (0..3) |axis| {
            const lo = cbounds.min.get(axis);
            const extent = cbounds.max.get(axis) - lo;
            if (!(extent > 0)) continue;

            var bin_bounds = [_]Aabb{Aabb.empty} ** bin_count;
            var bin_counts = [_]usize{0} ** bin_count;
            for (range) |idx| {
                const b = binOf(self.centroids[idx].get(axis), lo, extent);
                bin_counts[b] += 1;
                bin_bounds[b] = bin_bounds[b].merge(self.boxes[idx]);
            }

            // Sweep from the right to get suffix areas/counts.
            var right_area: [bin_count]f64 = undefined;
            var right_count: [bin_count]usize = undefined;
            var acc = Aabb.empty;
            var acc_n: usize = 0;
            var i: usize = bin_count;
            while (i > 0) {
                i -= 1;
                acc = acc.merge(bin_bounds[i]);
                acc_n += bin_counts[i];
                right_area[i] = acc.surfaceArea();
                right_count[i] = acc_n;
            }

            acc = Aabb.empty;
            acc_n = 0;
            for (0..bin_count - 1) |b| {
                acc = acc.merge(bin_bounds[b]);
                acc_n += bin_counts[b];
                const rn = right_count[b + 1];
                if (acc_n == 0 or rn == 0) continue;
                const cost = traversal_cost + (acc.surfaceArea() * @as(f64, @floatFromInt(acc_n)) +
                    right_area[b + 1] * @as(f64, @floatFromInt(rn))) / parent_area;
                if (best == null or cost < best.?.cost) best = .{ .axis = axis, .bin = b, .cost = cost };
            }
        }
        return best;
    }

    fn binOf(c: f64, lo: f64, extent: f64) usize {
        const f = (c - lo) / extent * @as(f64, bin_count);
        const b: usize = @intFromFloat(std.math.clamp(f, 0, bin_count - 1));
        return b;
    }
};

const testing = std.testing;
const Rng = @import("rng.zig").Rng;

fn randomVec(rng: *Rng, lo: f64, hi: f64) Vec3 {
    return Vec3.init(rng.range(lo, hi), rng.range(lo, hi), rng.range(lo, hi));
}

fn randomScene(allocator: std.mem.Allocator, rng: *Rng, n: usize) ![]Shape {
    const shapes = try allocator.alloc(Shape, n);
    for (shapes, 0..) |*s, i| {
        const mat: u32 = @intCast(i);
        if (rng.float() < 0.5) {
            s.* = .{ .sphere = .{ .center = randomVec(rng, -10, 10), .radius = rng.range(0.05, 1.5), .material = mat } };
        } else {
            const c = randomVec(rng, -10, 10);
            s.* = .{ .triangle = .{
                .v0 = c.add(randomVec(rng, -2, 2)),
                .v1 = c.add(randomVec(rng, -2, 2)),
                .v2 = c.add(randomVec(rng, -2, 2)),
                .material = mat,
            } };
        }
    }
    return shapes;
}

fn expectMatchesBruteForce(bvh: Bvh, shapes: []const Shape, rng: *Rng, rays: usize) !void {
    var hits: usize = 0;
    for (0..rays) |_| {
        // Aim most rays near a random primitive so plenty of them hit.
        const origin = randomVec(rng, -15, 15);
        const dir = if (rng.float() < 0.8) blk: {
            const target = shapes[rng.below(shapes.len)].bounds().?.centroid().add(randomVec(rng, -0.5, 0.5));
            const d = target.sub(origin);
            break :blk if (d.nearZero()) rng.unitVector() else d.normalize();
        } else rng.unitVector();
        const r = Ray.init(origin, dir);
        const want = geometry.intersectAll(shapes, r, 1e-6, 1e9);
        const got = bvh.intersect(shapes, r, 1e-6, 1e9);
        try testing.expectEqual(want == null, got == null);
        if (want) |w| {
            hits += 1;
            try testing.expectEqual(w.t, got.?.t);
            try testing.expectEqual(w.material, got.?.material);
        }
    }
    // Make sure the comparison is not vacuous.
    try testing.expect(hits > 0);
    if (shapes.len >= 7) try testing.expect(hits > rays / 4);
}

test "BVH agrees with brute force on random scenes" {
    var rng = Rng.init(2026);
    for ([_]usize{ 1, 2, 3, 7, 50, 400 }) |n| {
        const shapes = try randomScene(testing.allocator, &rng, n);
        defer testing.allocator.free(shapes);
        var bvh = try Bvh.build(testing.allocator, shapes);
        defer bvh.deinit(testing.allocator);
        try expectMatchesBruteForce(bvh, shapes, &rng, 3000);
    }
}

test "BVH structure is a valid permutation with enclosing bounds" {
    var rng = Rng.init(11);
    const shapes = try randomScene(testing.allocator, &rng, 257);
    defer testing.allocator.free(shapes);
    var bvh = try Bvh.build(testing.allocator, shapes);
    defer bvh.deinit(testing.allocator);

    const seen = try testing.allocator.alloc(bool, shapes.len);
    defer testing.allocator.free(seen);
    @memset(seen, false);
    for (bvh.nodes) |node| {
        if (!node.isLeaf()) {
            for ([_]u32{ node.first, node.first + 1 }) |c| {
                const cb = bvh.nodes[c].bounds;
                try testing.expect(cb.merge(node.bounds).min.approxEq(node.bounds.min, 0));
                try testing.expect(cb.merge(node.bounds).max.approxEq(node.bounds.max, 0));
            }
            continue;
        }
        for (bvh.indices[node.first..][0..node.count]) |i| {
            try testing.expect(!seen[i]);
            seen[i] = true;
        }
    }
    for (seen) |s| try testing.expect(s);
    try testing.expect(bvh.depth() < 30);
    try testing.expect(bvh.nodes.len <= 2 * shapes.len - 1);
}

test "BVH handles coincident primitives and empty input" {
    var rng = Rng.init(5);
    var shapes: [40]Shape = undefined;
    for (&shapes, 0..) |*s, i| s.* = .{ .sphere = .{ .center = Vec3.zero, .radius = 1 + @as(f64, @floatFromInt(i)) * 0.01, .material = @intCast(i) } };
    var bvh = try Bvh.build(testing.allocator, &shapes);
    defer bvh.deinit(testing.allocator);
    try expectMatchesBruteForce(bvh, &shapes, &rng, 500);

    var empty = try Bvh.build(testing.allocator, &.{});
    try testing.expect(empty.intersect(&.{}, Ray.init(Vec3.zero, Vec3.init(1, 0, 0)), 0, 1e9) == null);
    empty.deinit(testing.allocator);
}

test "BVH rejects unbounded shapes" {
    const shapes = [_]Shape{.{ .plane = .{ .point = Vec3.zero, .normal = Vec3.init(0, 1, 0), .material = 0 } }};
    try testing.expectError(error.UnboundedShape, Bvh.build(testing.allocator, &shapes));
}
