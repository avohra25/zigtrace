//! Minimal PNG encoder: 8-bit RGB, no filtering, zlib stream made of stored
//! (uncompressed) DEFLATE blocks. Files are larger than a real compressor
//! would produce but are valid everywhere and need no dependencies.

const std = @import("std");

const signature = "\x89PNG\r\n\x1a\n";
const max_stored_block = 65535;

pub fn writePng(writer: anytype, width: usize, height: usize, rgb: []const u8) !void {
    std.debug.assert(rgb.len == width * height * 3);
    if (width == 0 or height == 0 or width > std.math.maxInt(u31) or height > std.math.maxInt(u31))
        return error.InvalidDimensions;

    try writer.writeAll(signature);

    var ihdr: [13]u8 = undefined;
    std.mem.writeInt(u32, ihdr[0..4], @intCast(width), .big);
    std.mem.writeInt(u32, ihdr[4..8], @intCast(height), .big);
    ihdr[8] = 8; // bit depth
    ihdr[9] = 2; // colour type: truecolour
    ihdr[10] = 0; // compression: deflate
    ihdr[11] = 0; // filter method
    ihdr[12] = 0; // no interlace
    try writeChunk(writer, "IHDR", &ihdr);

    // The zlib stream is written as one IDAT chunk; its length is known up
    // front because stored blocks have a fixed 5-byte overhead.
    const raw_len = (width * 3 + 1) * height;
    const blocks = std.math.divCeil(usize, raw_len, max_stored_block) catch unreachable;
    const zlib_len = 2 + raw_len + 5 * blocks + 4;
    if (zlib_len > std.math.maxInt(u31)) return error.ImageTooLarge;

    var crc = std.hash.Crc32.init();
    var len_buf: [4]u8 = undefined;
    std.mem.writeInt(u32, &len_buf, @intCast(zlib_len), .big);
    try writer.writeAll(&len_buf);
    const cw = ChunkWriter(@TypeOf(writer)){ .inner = writer, .crc = &crc };
    try cw.write("IDAT");

    var deflate = StoredDeflate(@TypeOf(cw)){ .out = cw, .remaining = raw_len };
    try cw.write(&.{ 0x78, 0x01 }); // zlib header: deflate, 32K window, no dict
    const row_bytes = width * 3;
    for (0..height) |y| {
        try deflate.write(&.{0}); // filter type: none
        try deflate.write(rgb[y * row_bytes ..][0..row_bytes]);
    }
    std.debug.assert(deflate.remaining == 0 and deflate.block_left == 0);
    var adler_buf: [4]u8 = undefined;
    std.mem.writeInt(u32, &adler_buf, deflate.adler.final(), .big);
    try cw.write(&adler_buf);

    var crc_buf: [4]u8 = undefined;
    std.mem.writeInt(u32, &crc_buf, crc.final(), .big);
    try writer.writeAll(&crc_buf);

    try writeChunk(writer, "IEND", "");
}

fn writeChunk(writer: anytype, kind: *const [4]u8, data: []const u8) !void {
    var buf: [4]u8 = undefined;
    std.mem.writeInt(u32, &buf, @intCast(data.len), .big);
    try writer.writeAll(&buf);
    var crc = std.hash.Crc32.init();
    crc.update(kind);
    crc.update(data);
    try writer.writeAll(kind);
    try writer.writeAll(data);
    std.mem.writeInt(u32, &buf, crc.final(), .big);
    try writer.writeAll(&buf);
}

fn ChunkWriter(comptime W: type) type {
    return struct {
        inner: W,
        crc: *std.hash.Crc32,

        fn write(self: @This(), bytes: []const u8) !void {
            self.crc.update(bytes);
            try self.inner.writeAll(bytes);
        }
    };
}

/// Splits a byte stream of known total length into stored DEFLATE blocks.
fn StoredDeflate(comptime W: type) type {
    return struct {
        out: W,
        remaining: usize,
        block_left: usize = 0,
        adler: std.hash.Adler32 = std.hash.Adler32.init(),

        fn write(self: *@This(), bytes: []const u8) !void {
            var rest = bytes;
            while (rest.len > 0) {
                if (self.block_left == 0) try self.startBlock();
                const n = @min(rest.len, self.block_left);
                try self.out.write(rest[0..n]);
                self.adler.update(rest[0..n]);
                self.block_left -= n;
                self.remaining -= n;
                rest = rest[n..];
            }
        }

        fn startBlock(self: *@This()) !void {
            const len: u16 = @intCast(@min(self.remaining, max_stored_block));
            const final: u8 = if (len == self.remaining) 1 else 0;
            var hdr: [5]u8 = undefined;
            hdr[0] = final; // BFINAL + BTYPE=00 (stored)
            std.mem.writeInt(u16, hdr[1..3], len, .little);
            std.mem.writeInt(u16, hdr[3..5], ~len, .little);
            try self.out.write(&hdr);
            self.block_left = len;
        }
    };
}

const testing = std.testing;

const Chunk = struct { kind: []const u8, data: []const u8 };

fn readChunk(bytes: []const u8, pos: *usize) !Chunk {
    const len = std.mem.readInt(u32, bytes[pos.*..][0..4], .big);
    const kind = bytes[pos.* + 4 ..][0..4];
    const data = bytes[pos.* + 8 ..][0..len];
    const stored_crc = std.mem.readInt(u32, bytes[pos.* + 8 + len ..][0..4], .big);
    var crc = std.hash.Crc32.init();
    crc.update(kind);
    crc.update(data);
    try testing.expectEqual(crc.final(), stored_crc);
    pos.* += 12 + len;
    return .{ .kind = kind, .data = data };
}

fn roundTrip(width: usize, height: usize) !void {
    const rgb = try testing.allocator.alloc(u8, width * height * 3);
    defer testing.allocator.free(rgb);
    for (rgb, 0..) |*b, i| b.* = @truncate(i *% 31 +% (i >> 7));

    var out = std.ArrayList(u8).init(testing.allocator);
    defer out.deinit();
    try writePng(out.writer(), width, height, rgb);
    const bytes = out.items;
    try testing.expectEqualSlices(u8, signature, bytes[0..8]);

    var pos: usize = 8;
    const ihdr = try readChunk(bytes, &pos);
    try testing.expectEqualStrings("IHDR", ihdr.kind);
    try testing.expectEqual(@as(u32, @intCast(width)), std.mem.readInt(u32, ihdr.data[0..4], .big));
    try testing.expectEqual(@as(u32, @intCast(height)), std.mem.readInt(u32, ihdr.data[4..8], .big));
    const idat = try readChunk(bytes, &pos);
    try testing.expectEqualStrings("IDAT", idat.kind);
    const iend = try readChunk(bytes, &pos);
    try testing.expectEqualStrings("IEND", iend.kind);
    try testing.expectEqual(bytes.len, pos);

    // Inflate with the standard library and compare scanlines.
    var stream = std.io.fixedBufferStream(idat.data);
    var raw = std.ArrayList(u8).init(testing.allocator);
    defer raw.deinit();
    try std.compress.zlib.decompress(stream.reader(), raw.writer());
    try testing.expectEqual((width * 3 + 1) * height, raw.items.len);
    for (0..height) |y| {
        const line = raw.items[y * (width * 3 + 1) ..][0 .. width * 3 + 1];
        try testing.expectEqual(@as(u8, 0), line[0]);
        try testing.expectEqualSlices(u8, rgb[y * width * 3 ..][0 .. width * 3], line[1..]);
    }
}

test "small png round-trips through std inflate" {
    try roundTrip(3, 2);
}

test "png spanning several stored blocks" {
    // 200x150 RGB is ~90 KB of scanlines: two stored blocks.
    try roundTrip(200, 150);
    // One scanline just over a single stored block (65536 bytes).
    try roundTrip(21845, 1);
}
