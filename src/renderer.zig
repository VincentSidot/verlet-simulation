const std = @import("std");
const math = std.math;

const r = @import("raylib.zig").c;

const engineSrc = @import("engine.zig");

const Vec2 = r.Vector2;
const Color = r.Color;

const Solver = engineSrc.Solver;
const Constraints = engineSrc.Constraints;
const Particle = engineSrc.Particle;

const BACKGROUND_COLOR = r.BLACK;

fn writeText(buf: []u8, comptime fmt: []const u8, args: anytype, base_y: *c_int) void {
    const base_x = 10;
    const font_size = 20;

    if (std.fmt.bufPrintZ(buf, fmt, args)) |_| {
        r.DrawText(@ptrCast(buf), base_x, base_y.*, font_size, r.RAYWHITE);
        base_y.* += font_size + 5;
    } else |_| {
        // Ignore formatting errors
    }
}

fn renderConstraints(constraints: Constraints) void {
    switch (constraints) {
        .circle => |circle| {
            r.DrawCircleLines(
                @intFromFloat(circle.center.x),
                @intFromFloat(circle.center.y),
                circle.radius,
                BACKGROUND_COLOR,
            );
        },
        .box => |box| {
            r.DrawRectangleLines(
                @intFromFloat(box.min.x),
                @intFromFloat(box.min.y),
                @intFromFloat(box.max.x - box.min.x),
                @intFromFloat(box.max.y - box.min.y),
                BACKGROUND_COLOR,
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
    engine: *const Solver,
) void {
    var buf: [255]u8 = undefined;
    var base_y: c_int = 10;

    const frameTime: f32 = r.GetFrameTime();
    const frameTimeMs: u32 = @intFromFloat(frameTime * 1000.0);

    writeText(&buf, "Object count: {d}", .{engine.objects.items.len}, &base_y);
    writeText(&buf, "Frame time: {d} ms", .{frameTimeMs}, &base_y);
}

pub fn render(engine: *const Solver) void {
    renderConstraints(engine.constraints);
    for (engine.objects.items) |*obj| {
        renderParticle(obj);
    }
    renderEngineInfo(engine);
}
