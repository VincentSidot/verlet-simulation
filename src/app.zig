const std = @import("std");
const math = std.math;

const r = @import("raylib.zig").c;

const engineSrc = @import("engine.zig");
const imageSrc = @import("image.zig");
const renderer = @import("renderer.zig");

const Particle = engineSrc.Particle;
const Engine = engineSrc.Solver;
const Constraints = engineSrc.Constraints;
const Image = imageSrc.Image;

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

const WIDTH: comptime_float = 1024;
const HEIGHT: comptime_float = 1024;

const FRAME_RATE = 60;

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

pub fn run() !void {
    r.SetTraceLogLevel(RAYLIB_LOG_TARGET);

    r.InitWindow(WIDTH, HEIGHT, "Verlet Engine with Raylib");
    defer r.CloseWindow();

    r.SetTargetFPS(FRAME_RATE);

    const circleBoundaryCenterX: comptime_float = 0.5 * WIDTH;
    const circleBoundaryCenterY: comptime_float = 0.5 * HEIGHT;
    const circleBoundaryRadius: comptime_float = @min(WIDTH, HEIGHT) * 0.45;

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

    const objectSpawnDelay = 0.015;
    // const objectSpawnSpeed = 800.0;
    const objectRadius = 6.0;

    // Compute boundary area
    const boundaryArea = math.pi * circleBoundaryRadius * circleBoundaryRadius; // pi * r^2
    const objectArea = math.pi * objectRadius * objectRadius; // pi * r^
    const targetObjectsCount: comptime_int = @intFromFloat((boundaryArea / objectArea) * 0.95);

    std.debug.print("Target objects count: {}\n", .{targetObjectsCount});

    // Compute the highest point of the circle boundary
    const consideredObjectRadius = 4 * objectRadius;
    const circleTopY = circleBoundaryCenterY - circleBoundaryRadius + 2 * consideredObjectRadius;
    const centerX = circleBoundaryCenterX;

    const objectSpawnPositionA: Vec2 = .{
        .x = centerX - consideredObjectRadius,
        .y = circleTopY,
    };
    const objectSpawnPositionB: Vec2 = .{
        .x = centerX + consideredObjectRadius,
        .y = circleTopY,
    };
    // const maxObjectsCount = 10_000;
    const objectsVelocity = 1200.0;

    var lastTime = r.GetTime();
    mainLoop: while (!r.WindowShouldClose()) {
        const getTime = r.GetTime();
        const deltaTime = getTime - lastTime;

        if (r.IsKeyPressed(r.KEY_ESCAPE)) {
            break :mainLoop;
        }

        if (engine.getObjectsCount() < targetObjectsCount and deltaTime >= objectSpawnDelay) {
            lastTime = getTime;

            const time = engine.getTime();
            const angle = objectsAngle(time);
            const angle2 = angle + (math.pi / 2.0);
            const speed = objectsVelocity;

            var objA = engine.addObject(objectSpawnPositionA, objectRadius);
            engine.setObjectVelocity(objA, r.Vector2Scale(
                .{
                    .x = math.cos(angle2),
                    .y = math.sin(angle2),
                },
                speed,
            ));
            objA.setColor(getColor(time));

            var objB = engine.addObject(objectSpawnPositionB, objectRadius);
            engine.setObjectVelocity(objB, r.Vector2Scale(
                .{
                    .x = math.cos(angle),
                    .y = math.sin(angle),
                },
                speed,
            ));
            objB.setColor(getColor(time));
        } else if (engine.getObjectsCount() == targetObjectsCount) {
            std.debug.print("Reached target objects count: {}\n", .{targetObjectsCount});
            // Done spawning objects for this simulation
            // Let's create from the image
            const targetWidth: i32 = @intFromFloat(WIDTH / objectRadius);
            const targetHeight: i32 = @intFromFloat(HEIGHT / objectRadius);
            const img = try Image.init("images/sky.jpg", targetWidth, targetHeight);
            defer img.deinit();

            const file = try std.fs.cwd().createFile("config/sky.bin", .{ .truncate = true });
            defer file.close();

            for (engine.objects.items) |*obj| {
                const pos = obj.position;
                const imgX: i32 = @intFromFloat(pos.x / objectRadius);
                const imgY: i32 = @intFromFloat(pos.y / objectRadius);

                const color = img.getColorAt(imgX, imgY);
                obj.setColor(color); // To se the rendered image
                const colorSlice: [4]u8 = @bitCast(color);
                _ = try file.write(&colorSlice);
            }

            std.debug.print("Saved object colors to config/sky.bin\n", .{});
        }

        engine.update();

        renderer.render(&engine);
    }
}
