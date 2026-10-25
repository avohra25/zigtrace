//! Small, fast, deterministic random number generator (xoshiro256++ seeded
//! via SplitMix64) plus the sampling routines the renderer needs.
//!
//! The renderer derives an independent stream per pixel from a global seed,
//! so images are bit-for-bit reproducible regardless of thread count.

const std = @import("std");
const Vec3 = @import("vec.zig").Vec3;

pub const Rng = struct {
    s: [4]u64,

    pub fn init(seed: u64) Rng {
        var sm = seed;
        var r: Rng = undefined;
        for (&r.s) |*word| word.* = splitMix64(&sm);
        return r;
    }

    /// Derives a well-mixed seed from a base seed and a stream index (for
    /// example a pixel index), so neighbouring streams are uncorrelated.
    pub fn forStream(seed: u64, stream: u64) Rng {
        var sm = seed ^ (stream *% 0x9E3779B97F4A7C15);
        return init(splitMix64(&sm));
    }

    pub fn next(self: *Rng) u64 {
        const s = &self.s;
        const result = std.math.rotl(u64, s[0] +% s[3], 23) +% s[0];
        const t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = std.math.rotl(u64, s[3], 45);
        return result;
    }

    /// Uniform float in [0, 1) with 53 bits of precision.
    pub fn float(self: *Rng) f64 {
        return @as(f64, @floatFromInt(self.next() >> 11)) * 0x1.0p-53;
    }

    /// Uniform float in [lo, hi).
    pub fn range(self: *Rng, lo: f64, hi: f64) f64 {
        return lo + (hi - lo) * self.float();
    }

    /// Uniform integer in [0, n).
    pub fn below(self: *Rng, n: u64) u64 {
        std.debug.assert(n > 0);
        // Lemire's multiply-shift; the tiny bias is irrelevant here.
        return @intCast((@as(u128, self.next()) * n) >> 64);
    }

    /// Uniformly distributed unit vector (point on the unit sphere).
    pub fn unitVector(self: *Rng) Vec3 {
        const z = self.range(-1.0, 1.0);
        const phi = self.range(0.0, 2.0 * std.math.pi);
        const r = @sqrt(@max(0.0, 1.0 - z * z));
        return Vec3.init(r * @cos(phi), r * @sin(phi), z);
    }

    /// Uniform point inside the unit sphere.
    pub fn inUnitSphere(self: *Rng) Vec3 {
        while (true) {
            const p = Vec3.init(self.range(-1, 1), self.range(-1, 1), self.range(-1, 1));
            if (p.lengthSquared() < 1.0) return p;
        }
    }

    /// Uniform point inside the unit disk in the xy plane.
    pub fn inUnitDisk(self: *Rng) Vec3 {
        while (true) {
            const p = Vec3.init(self.range(-1, 1), self.range(-1, 1), 0);
            if (p.lengthSquared() < 1.0) return p;
        }
    }
};

fn splitMix64(state: *u64) u64 {
    state.* +%= 0x9E3779B97F4A7C15;
    var z = state.*;
    z = (z ^ (z >> 30)) *% 0xBF58476D1CE4E5B9;
    z = (z ^ (z >> 27)) *% 0x94D049BB133111EB;
    return z ^ (z >> 31);
}

const testing = std.testing;

test "same seed gives same sequence" {
    var a = Rng.init(42);
    var b = Rng.init(42);
    for (0..100) |_| try testing.expectEqual(a.next(), b.next());
    var c = Rng.init(43);
    try testing.expect(a.next() != c.next());
}

test "streams differ" {
    var a = Rng.forStream(7, 0);
    var b = Rng.forStream(7, 1);
    try testing.expect(a.next() != b.next());
}

test "float range and mean" {
    var r = Rng.init(1);
    var sum: f64 = 0;
    const n = 100_000;
    for (0..n) |_| {
        const f = r.float();
        try testing.expect(f >= 0 and f < 1);
        sum += f;
    }
    try testing.expectApproxEqAbs(@as(f64, 0.5), sum / n, 0.01);
}

test "below stays in range" {
    var r = Rng.init(9);
    var counts = [_]u32{0} ** 5;
    for (0..10_000) |_| counts[r.below(5)] += 1;
    for (counts) |c| try testing.expect(c > 1700 and c < 2300);
}

test "sampling domains" {
    var r = Rng.init(3);
    var mean = Vec3.zero;
    for (0..20_000) |_| {
        const u = r.unitVector();
        try testing.expectApproxEqAbs(@as(f64, 1), u.length(), 1e-9);
        mean = mean.add(u);
        try testing.expect(r.inUnitSphere().lengthSquared() < 1);
        const d = r.inUnitDisk();
        try testing.expect(d.lengthSquared() < 1 and d.z == 0);
    }
    // Uniform directions average out to roughly the origin.
    try testing.expect(mean.div(20_000).length() < 0.03);
}
