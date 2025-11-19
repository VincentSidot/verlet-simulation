const std = @import("std");

const stb = @import("stb.zig");
const app = @import("app.zig");

// pub fn main() !void {
//     try app.run();
// }

const Image = stb.image;

pub fn main() !void {
    const imagePath = "images/sky.jpg";

    std.debug.print("Loading image from path: {s}\n", .{imagePath});

    var width: c_int = undefined;
    var height: c_int = undefined;
    var channels: c_int = undefined;

    const data = Image.stbi_load(
        imagePath,
        &width,
        &height,
        &channels,
        0,
    );
    defer Image.stbi_image_free(data);

    std.debug.print("Image data: {*}\nWidth: {d}\nHeight: {d}\nChannels: {d}\n", .{
        data,
        width,
        height,
        channels,
    });

    const resizedWidth: c_int = 512;
    const resizedHeight: c_int = 512;
    const layout: Image.stbir_pixel_layout = 4;

    const resizedData = Image.stbir_resize_uint8_linear(
        data,
        width,
        height,
        0,
        null,
        resizedWidth,
        resizedHeight,
        0,
        layout,
    );

    std.debug.print("Image data: {*}\nWidth: {d}\nHeight: {d}\nChannels: {d}\n", .{
        resizedData,
        resizedWidth,
        resizedHeight,
        layout,
    });
}
