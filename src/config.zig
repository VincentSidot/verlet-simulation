const std = @import("std");
const math = std.math;

const Allocator = std.mem.Allocator;
const File = std.fs.File;

const r = @import("raylib.zig").c;
const engineSrc = @import("engine.zig");

const Constraints = engineSrc.Constraints;

const Vec2 = r.Vector2;
const Color = r.Color;

pub const ObjectSpawner = struct {
    /// Position of the spawner
    pos: Vec2,

    /// Velocity of the spawned objects
    velocity: f32,

    /// Angle range (in radians) for the spawned objects
    angle: struct {
        /// Start angle
        start: f32,
        /// End angle
        end: f32,
    },

    /// Delay between spawns
    delay: f32,

    fn getAngle(self: *const ObjectSpawner, time: f32) f32 {
        const sin = math.sin(time);
        const sin2 = sin * sin;

        return self.angle.start + (self.angle.end - self.angle.start) * sin2;
    }

    pub fn getVelocityAtTime(self: *const ObjectSpawner, time: f32) Vec2 {
        const angle = self.getAngle(time);
        return r.Vector2Scale(
            .{
                .x = math.cos(angle),
                .y = math.sin(angle),
            },
            self.velocity,
        );
    }
};

const ConfigVersion = u16; // Configuration file version
const CURRENT_CONFIG_VERSION: ConfigVersion = 1;
const MAGIC_NUMBER = "VSCF"; // "Verlet Simulation Config File"

fn writeInFile(file: File, value: anytype) !void {
    const T = @TypeOf(value);
    const T_size = @sizeOf(T);

    const bytes = std.mem.asBytes(&value);

    const written = try file.write(bytes);
    if (written != T_size) {
        return error.WriteFailed;
    }
}

fn readFromFile(file: File, value: anytype) !void {
    const P = @TypeOf(value);
    const P_Info = @typeInfo(P);

    const T: type = comptime blk: {
        switch (P_Info) {
            .pointer => |ptr_info| break :blk ptr_info.child,
            else => return error.InvalidType,
        }
    };

    const T_size = @sizeOf(T);
    const slices = std.mem.asBytes(value);

    const read = try file.read(slices);
    if (read != T_size) {
        return error.ReadFailed;
    }
}

pub const ObjectCountStrategy = union(enum) {
    /// Fixed number of objects
    fixed: usize,
    /// Compute needed objects based on target constraints fill percentage
    fill: f32,

    fn getCount(self: ObjectCountStrategy, constraints: *const Constraints, objectRadius: f32) usize {
        return switch (self) {
            .fixed => |count| count,
            .fill => |percentage| {
                const objectArea = math.pi * objectRadius * objectRadius;
                const constraintsArea = constraints.getArea();

                const targetArea = constraintsArea * percentage;
                return @intFromFloat(targetArea / objectArea);
            },
        };
    }
};

pub fn SimulationConfig(allocator: Allocator) type {
    return struct {
        const Self = @This();

        version: ConfigVersion = CURRENT_CONFIG_VERSION,

        frameRate: u16,
        objectCount: usize,
        objectRadius: f32,

        width: u16,
        height: u16,

        substep: u8,

        constraints: Constraints,

        spawners: []ObjectSpawner,

        balls: ?[]Color,

        pub fn init(
            frameRate: u16,
            objectRadius: f32,
            objectCountStrategy: ObjectCountStrategy,
            substep: u8,
            width: u16,
            height: u16,
            constraints: Constraints,
            spawners: []const ObjectSpawner,
        ) Self {
            const spawnersOwned = allocator.alloc(ObjectSpawner, spawners.len) catch unreachable;
            @memcpy(spawnersOwned, spawners);

            return .{
                .frameRate = frameRate,
                .width = width,
                .height = height,
                .objectRadius = objectRadius,
                .objectCount = objectCountStrategy.getCount(&constraints, objectRadius),
                .substep = substep,
                .constraints = constraints,
                .spawners = spawnersOwned,
                .balls = null,
            };
        }

        pub fn deinit(self: *const Self) void {
            allocator.free(self.spawners);

            if (self.balls) |balls| {
                allocator.free(balls);
            }
        }

        /// Saves the configuration into a file
        pub fn saveToFile(self: *const Self, path: []const u8) !void {
            const balls = self.balls orelse return error.InvalidState;

            var file = try std.fs.cwd().createFile(
                path,
                .{
                    .read = false,
                    .exclusive = false,
                    .truncate = true,
                },
            );
            defer file.close();

            // Write configuration data
            // For simplicity, we write binary data directly

            _ = try file.write(MAGIC_NUMBER);
            try writeInFile(file, self.version);

            try writeInFile(file, self.frameRate);
            try writeInFile(file, self.objectCount);
            try writeInFile(file, self.objectRadius);

            try writeInFile(file, self.width);
            try writeInFile(file, self.height);

            try writeInFile(file, self.substep);

            try writeInFile(file, self.constraints);

            try writeInFile(file, self.spawners.len);
            for (self.spawners) |spawner| {
                try writeInFile(file, spawner); // Spawner is a self-contained struct
            }

            try writeInFile(file, balls.len);
            for (balls) |color| {
                try writeInFile(file, color); // Color is a self-contained struct
            }
        }

        pub fn loadFromFile(path: []const u8) !Self {
            const file = try std.fs.cwd().openFile(
                path,
                .{ .mode = .read_only },
            );
            defer file.close();

            var magic: [4]u8 = undefined;
            _ = try file.read(&magic);
            if (!std.mem.eql(u8, &magic, MAGIC_NUMBER)) {
                return error.InvalidFileFormat;
            }

            var self: Self = undefined; // Placeholder for the struct to be filled

            // Read version
            try readFromFile(file, &self.version);

            if (self.version != CURRENT_CONFIG_VERSION) {
                return error.UnmatchedConfigVersion;
            }

            // Read configuration data
            try readFromFile(file, &self.frameRate);
            try readFromFile(file, &self.objectCount);
            try readFromFile(file, &self.objectRadius);

            try readFromFile(file, &self.width);
            try readFromFile(file, &self.height);

            try readFromFile(file, &self.substep);

            try readFromFile(file, &self.constraints);

            var spawnerCount: usize = 0;
            try readFromFile(file, &spawnerCount);
            const spawners = try allocator.alloc(ObjectSpawner, spawnerCount);
            for (spawners) |*spawner| {
                try readFromFile(file, spawner);
            }
            self.spawners = spawners;

            var ballCount: usize = 0;
            try readFromFile(file, &ballCount);
            const balls = try allocator.alloc(Color, ballCount);
            for (balls) |*color| {
                try readFromFile(file, color);
            }
            self.balls = balls;

            return self;
        }
    };
}

test "write / read config" {
    const allocator = std.testing.allocator;
    const t = std.testing;

    var configOut = SimulationConfig(allocator).init(
        60,
        5.0,
        .{ .fixed = 10 },
        4,
        800,
        600,
        .{ .box = .{
            .min = .{ .x = 0.0, .y = 0.0 },
            .max = .{ .x = 800.0, .y = 600.0 },
        } },
        &.{
            .{ .pos = .{ .x = 400.0, .y = 300.0 }, .velocity = 100.0, .angle = .{ .start = 0.0, .end = math.pi * 2.0 }, .delay = 0.5 },
            .{ .pos = .{ .x = 200.0, .y = 300.0 }, .velocity = 1000.0, .angle = .{ .start = 1.0, .end = math.pi * 2.0 }, .delay = 0.25 },
        },
    );
    defer configOut.deinit();

    const colorMap: [10]Color = .{
        r.RED,
        r.GREEN,
        r.BLUE,
        r.YELLOW,
        r.ORANGE,
        r.PURPLE,
        r.RAYWHITE,
        r.SKYBLUE,
        r.LIME,
        r.MAROON,
    };

    const balls = try allocator.alloc(Color, configOut.objectCount);
    for (balls, 0..) |*ball, idx| {
        ball.* = colorMap[idx % colorMap.len];
    }
    configOut.balls = balls;

    const filePath = "test_config.vscf";
    try configOut.saveToFile(filePath);
    defer std.fs.cwd().deleteFile(filePath) catch {
        std.debug.print("Failed to delete test config file: {s}\n", .{filePath});
    };

    const configIn = try SimulationConfig(allocator).loadFromFile(filePath);
    defer configIn.deinit();

    // Validate

    try t.expectEqual(configOut.version, configIn.version);

    try t.expectEqual(configOut.frameRate, configIn.frameRate);
    try t.expectEqual(configOut.objectCount, configIn.objectCount);
    try t.expectEqual(configOut.objectRadius, configIn.objectRadius);

    try t.expectEqual(configOut.width, configIn.width);
    try t.expectEqual(configOut.height, configIn.height);

    try t.expectEqual(configOut.substep, configIn.substep);

    try t.expectEqualDeep(configOut.constraints, configIn.constraints);

    try t.expectEqual(configOut.spawners.len, configIn.spawners.len);
    for (configOut.spawners, configIn.spawners, 0..) |in, out, idx| {
        t.expectEqualDeep(in, out) catch |err| {
            std.debug.print("Spawner mismatch at index {d}\n", .{idx});
            return err;
        };
    }

    const ballsOut = configOut.balls orelse return error.InvalidStateOut;
    const ballsIn = configIn.balls orelse return error.InvalidStateIn;

    try t.expectEqual(ballsOut.len, ballsIn.len);
    for (ballsOut, ballsIn, 0..) |in, out, idx| {
        t.expectEqualDeep(in, out) catch |err| {
            std.debug.print("Ball color mismatch at index {d}\n", .{idx});
            return err;
        };
    }
}
