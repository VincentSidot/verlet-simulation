// raylib.zig - Zig bindings for Raylib library

pub const c = @cImport({
    @cInclude("../../raylib-5.5/include/raylib.h");
    @cInclude("../../raylib-5.5/include/raymath.h");
});
