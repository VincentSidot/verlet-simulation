const std = @import("std");
const math = std.math;

const r = @import("raylib.zig").c;

const engineSrc = @import("engine.zig");
const configSrc = @import("config.zig");

const renderer = @import("renderer.zig");

const allocator = std.heap.smp_allocator;

const Particle = engineSrc.Particle;
const Engine = engineSrc.Solver(allocator);
const Constraints = engineSrc.Constraints;

const Color = r.Color;
const Vec2 = r.Vector2;

fn getColor(t: f32) Color {
    const red = math.sin(t);
    const green = math.sin(t + 0.33 * 2.0 * math.pi);
    const blue = math.sin(t + 0.66 * 2.0 * math.pi);

    return .{
        .r = @intFromFloat((red * red) * 255.0),
        .g = @intFromFloat((green * green) * 255.0),
        .b = @intFromFloat((blue * blue) * 255.0),
        .a = 0xFF,
    };
}

const WIDTH: comptime_float = 800;
const HEIGHT: comptime_float = 600;

const FRAME_RATE = 144;

fn objectsAngle(t: f32) f32 {
    const startAngle = math.pi / 8.0;
    const endAngle = 3.0 * math.pi / 8.0;

    const speed = 0.5; // cycles per second

    const sin = math.sin(t * speed);
    const sin2 = sin * sin;

    return startAngle + (endAngle - startAngle) * sin2;
}

// const RAYLIB_LOG_TARGET = r.LOG_ALL;
const RAYLIB_LOG_TARGET = r.LOG_WARNING;

pub const CreateConfigMode = struct {
    imgPath: [:0]const u8,
    configPath: [:0]const u8,
};

pub const ReadConfigMode = struct {
    configPath: [:0]const u8,
};

pub const RunMode = struct {
    mode: union(enum) {
        createConfig: CreateConfigMode,
        readConfig: ReadConfigMode,
    },

    // Do not cap the frame rate to simulation target rate
    realTime: bool = true,
};

const Config = configSrc.SimulationConfig(allocator);
const ObjectSpawner = configSrc.ObjectSpawner;

const OBJECT_SIZE = 5.0;
const SUBSTEP = 8;

const CONSTRAINTS_CIRCLE: Constraints = .{ .circle = .{
    .center = .{
        .x = 0.5 * WIDTH,
        .y = 0.5 * HEIGHT,
    },
    .radius = @min(WIDTH, HEIGHT) * 0.48,
} };

const CONSTRAINTS_BOX: Constraints = .{ .box = .{
    .min = .{
        .x = 0.01 * WIDTH,
        .y = 0.01 * HEIGHT,
    },
    .max = .{
        .x = 0.99 * WIDTH,
        .y = 0.99 * HEIGHT,
    },
} };

const SPAWNERS_CIRCLE: [2]ObjectSpawner = .{
    .{
        .pos = .{
            .x = 0.5 * WIDTH,
            .y = 0.5 * HEIGHT,
        },
        .velocity = 2000.0,
        .delay = 0.015,
        .angle = .{
            .start = 0.0,
            .end = 8.0 * math.pi, // 4 full rotations
        },
    },
    .{
        .pos = .{
            .x = 0.5 * WIDTH,
            .y = 0.5 * HEIGHT - CONSTRAINTS_CIRCLE.circle.radius + 2.0 * OBJECT_SIZE,
        },
        .velocity = 1999.0,
        .delay = 0.019,
        .angle = .{
            .start = 2.0 * math.pi / 8.0,
            .end = 6.0 * math.pi / 8.0,
        },
    },
};

const SPAWNERS_BOX: [3]ObjectSpawner = .{
    .{
        // Central spawner
        .pos = .{
            .x = 0.5 * WIDTH,
            .y = 0.1 * HEIGHT + 2.0 * OBJECT_SIZE,
        },
        .velocity = 900.0,
        .delay = 0.015,
        .angle = .{
            .start = math.pi / 6.0,
            .end = 5.0 * math.pi / 6.0,
        },
    },
    .{
        .pos = .{
            .x = 0.3 * WIDTH,
            .y = 0.1 * HEIGHT + 2.0 * OBJECT_SIZE,
        },
        .velocity = 900.0,
        .delay = 0.015,
        .angle = .{
            .start = (5.0 * math.pi / 6.0) - (math.pi / 24.0),
            .end = (5.0 * math.pi / 6.0) + (math.pi / 24.0),
        },
    },
    .{
        .pos = .{
            .x = 0.7 * WIDTH,
            .y = 0.1 * HEIGHT + 2.0 * OBJECT_SIZE,
        },
        .velocity = 900.0,
        .delay = 0.015,
        .angle = .{
            .start = (1.0 * math.pi / 6.0) - (math.pi / 24.0),
            .end = (1.0 * math.pi / 6.0) + (math.pi / 24.0),
        },
    },
};

fn defaultConfig() Config {
    return Config.init(
        FRAME_RATE,
        OBJECT_SIZE,
        .{ .fill = 0.95 },
        SUBSTEP,
        WIDTH,
        HEIGHT,
        CONSTRAINTS_CIRCLE,
        &SPAWNERS_CIRCLE,
    );
}

fn saveConfig(config: *Config, opts: CreateConfigMode, engine: *Engine) !void {
    // Opening the image
    std.debug.print("Opening image at path: {s}", .{opts.imgPath});

    const screenWidth: f32 = @floatFromInt(config.width);
    const screenHeight: f32 = @floatFromInt(config.height);

    const targetWidth: i32 = @intFromFloat(screenWidth / config.objectRadius);
    const targetHeight: i32 = @intFromFloat(screenHeight / config.objectRadius);

    var img = r.LoadImage(opts.imgPath);
    defer r.UnloadImage(img);

    r.ImageResize(&img, targetWidth, targetHeight);

    // Apply colors
    std.debug.print("Applying image to simulation\n", .{});

    const ballsColor = try allocator.alloc(Color, config.objectCount);

    for (engine.objects.items, 0..) |*obj, i| {
        const pos = obj.position;
        const imgX: i32 = @intFromFloat(pos.x / config.objectRadius);
        const imgY: i32 = @intFromFloat(pos.y / config.objectRadius);

        const color = r.GetImageColor(img, imgX, imgY);
        obj.setColor(color);
        ballsColor[i] = color;
    }

    config.balls = ballsColor; // Set the colors in the config

    // Saving the config file
    std.debug.print("Saving configuration to path: {s}\n", .{opts.configPath});
    try config.saveToFile(opts.configPath);
}

pub fn run(args: RunMode) !void {
    var config: Config = undefined;

    switch (args.mode) {
        .createConfig => {
            config = defaultConfig();
        },
        .readConfig => |cfg| {
            config = try Config.loadFromFile(cfg.configPath);
        },
    }

    r.SetTraceLogLevel(RAYLIB_LOG_TARGET);

    r.InitWindow(
        @intCast(config.width),
        @intCast(config.height),
        "Verlet Simulation",
    );
    defer r.CloseWindow();

    if (args.realTime) {
        r.SetTargetFPS(@intCast(config.frameRate));
    }

    // Compute grid size
    const gridRows: usize = @intFromFloat(@as(f32, @floatFromInt(config.height)) / (2.0 * config.objectRadius));
    const gridCols: usize = @intFromFloat(@as(f32, @floatFromInt(config.width)) / (2.0 * config.objectRadius));

    var engine = Engine.init(config.constraints, .{
        .rows = gridRows,
        .cols = gridCols,
    });
    defer engine.deinit();

    engine.setSubSteps(config.substep);
    engine.setRate(config.frameRate);

    const lastTimes: []f32 = allocator.alloc(f32, config.spawners.len) catch unreachable;
    defer allocator.free(lastTimes);

    while (!r.WindowShouldClose()) {
        const time = engine.getTime();

        if (engine.getObjectsCount() < config.objectCount) {
            for (config.spawners, lastTimes) |*spawner, *lastTime| {
                if (time - lastTime.* > spawner.delay) {
                    lastTime.* = time; // Save last spawn time

                    const objectIndex = engine.getObjectsCount();
                    const object = engine.addObject(spawner.pos, config.objectRadius);

                    const velocity = spawner.getVelocityAtTime(time);
                    engine.setObjectVelocity(object, velocity);

                    if (config.balls) |balls| {
                        object.setColor(balls[objectIndex]);
                    } else {
                        object.setColor(getColor(time));
                    }
                }
            }
        } else if (config.balls == null) {
            try saveConfig(&config, args.mode.createConfig, &engine);
        }

        const startEngineUpdate = r.GetTime();
        engine.update();
        const engineUpdate = r.GetTime() - startEngineUpdate;
        renderer.render(&engine, &config, engineUpdate);
    }
}
