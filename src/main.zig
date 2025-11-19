const std = @import("std");
const math = std.math;

const r = @import("raylib.zig").c;

const engineSrc = @import("engine.zig");
const renderer = @import("renderer.zig");

const Particle = engineSrc.Particle;
const Engine = engineSrc.Solver;
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

const WIDTH = 1024;
const HEIGHT = 1024;

const FRAME_RATE = 144;

fn objectsAngle(t: f32) f32 {
    const startAngle = math.pi / 8.0;
    const endAngle = 3.0 * math.pi / 8.0;

    const speed = 0.5; // cycles per second

    const sin = math.sin(t * speed);
    const sin2 = sin * sin;

    return startAngle + (endAngle - startAngle) * sin2;
}

const RAYLIB_LOG_TARGET = r.LOG_ALL;

pub fn main() !void {
    r.SetTraceLogLevel(RAYLIB_LOG_TARGET);

    r.InitWindow(WIDTH, HEIGHT, "Verlet Engine with Raylib");
    defer r.CloseWindow();

    r.SetTargetFPS(FRAME_RATE);

    var engine = Engine.init(.{ .circle = .{
        .center = .{
            .x = 0.5 * WIDTH,
            .y = 0.5 * HEIGHT,
        },
        .radius = @min(WIDTH, HEIGHT) * 0.45,
    } });
    defer engine.deinit();

    engine.setSubSteps(8);
    engine.setRate(FRAME_RATE);

    const objectSpawnDelay = 0.025;
    const objectSpawnSpeed = 2400.0;
    const objectRadius = 6.0;

    // const objectsAngle = math.pi / 6.0; // 30 degrees

    const objectSpawnPosition: Vec2 = .{
        .x = 0.3 * WIDTH,
        .y = 0.3 * HEIGHT,
    };
    const maxObjectsCount = 10_000;

    var lastTime = r.GetTime();
    mainLoop: while (!r.WindowShouldClose()) {
        const getTime = r.GetTime();
        const deltaTime = getTime - lastTime;

        if (r.IsKeyPressed(r.KEY_ESCAPE)) {
            break :mainLoop;
        }

        if (engine.getObjectsCount() < maxObjectsCount and deltaTime >= objectSpawnDelay) {
            lastTime = getTime;

            var obj = engine.addObject(objectSpawnPosition, objectRadius);
            const time = engine.getTime();
            const angle = objectsAngle(time);

            engine.setObjectVelocity(obj, r.Vector2Scale(
                .{
                    .x = math.cos(angle),
                    .y = math.sin(angle),
                },
                objectSpawnSpeed,
            ));
            obj.setColor(getColor(time));
        }

        engine.update();

        renderer.render(&engine);
    }
}
