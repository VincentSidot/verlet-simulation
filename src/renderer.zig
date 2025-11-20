const std = @import("std");
const math = std.math;

const r = @import("raylib.zig").c;

const configSrc = @import("config.zig");
const engineSrc = @import("engine.zig");

const Vec2 = r.Vector2;
const Color = r.Color;

const Solver = engineSrc.Solver;
const Constraints = engineSrc.Constraints;
const Particle = engineSrc.Particle;

const SimulationConfig = configSrc.SimulationConfig;

const CONSTRAINTS_BORDER_COLOR: Color = r.WHITE;
const BACKGROUND_COLOR: Color = .{
    .r = 0x15,
    .g = 0x15,
    .b = 0x15,
    .a = 0xFF,
};
const TEXT_COLOR: Color = r.WHITE;

fn writeText(buf: []u8, comptime fmt: []const u8, args: anytype, base_y: *c_int) void {
    const base_x = 10;
    const font_size = 20;

    if (std.fmt.bufPrintZ(buf, fmt, args)) |_| {
        r.DrawText(@ptrCast(buf), base_x, base_y.*, font_size, TEXT_COLOR);
        base_y.* += font_size + 5;
    } else |_| {
        // Ignore formatting errors
    }
}

fn renderConstraints(constraints: *const Constraints) void {
    switch (constraints.*) {
        .circle => |circle| {
            r.DrawCircleLines(
                @intFromFloat(circle.center.x),
                @intFromFloat(circle.center.y),
                circle.radius,
                CONSTRAINTS_BORDER_COLOR,
            );
        },
        .box => |box| {
            r.DrawRectangleLines(
                @intFromFloat(box.min.x),
                @intFromFloat(box.min.y),
                @intFromFloat(box.max.x - box.min.x),
                @intFromFloat(box.max.y - box.min.y),
                CONSTRAINTS_BORDER_COLOR,
            );
        },
    }
}

fn renderParticle(particle: *const Particle) void {
    r.DrawCircleV(
        particle.position,
        particle.radius,
        particle.color,
    );
}

fn renderEngineInfo(
    engine: anytype,
    config: anytype,
) void {
    var buf: [255]u8 = undefined;
    var base_y: c_int = 10;

    const frameTime: f32 = r.GetFrameTime();
    const frameTimeMs: u32 = @intFromFloat(frameTime * 1000.0);

    const objectCount: u32 = @intCast(engine.objects.items.len);
    const maxObjects: u32 = @intCast(config.objectCount);

    const fillPercentage: i32 = @intFromFloat(100.0 * @as(f32, @floatFromInt(objectCount)) / @as(f32, @floatFromInt(maxObjects)));

    const fps = r.GetFPS();

    writeText(&buf, "Object count: {d} ({d}%)", .{ objectCount, fillPercentage }, &base_y);
    writeText(&buf, "Frame time: {d} ms ({d} FPS)", .{ frameTimeMs, fps }, &base_y);
}

pub fn render(engine: anytype, config: anytype) void {
    r.BeginDrawing();
    defer r.EndDrawing();

    r.ClearBackground(BACKGROUND_COLOR);

    renderConstraints(&engine.constraints);
    for (engine.objects.items) |*obj| {
        renderParticle(obj);
    }
    renderEngineInfo(engine, config);
}
