const cImage = @import("stb.zig").image;
const cRaylib = @import("raylib.zig").c;

const Color = cRaylib.Color;

pub const Image = struct {
    width: i32,
    height: i32,
    channels: i32,
    raw: [*]u8,

    pub fn init(path: [:0]const u8, width: i32, height: i32) !Image {
        var w: c_int = 0;
        var h: c_int = 0;
        var ch: c_int = 0;

        const imgData = cImage.stbi_load(
            path,
            &w,
            &h,
            &ch,
            0,
        );

        if (imgData == null) {
            return error.FailedToLoadImage;
        }
        defer cImage.stbi_image_free(imgData);

        const resizedData = cImage.stbir_resize_uint8_linear(
            imgData,
            w,
            h,
            0,
            null,
            width,
            height,
            0,
            4, // Assume RGBA (4 channels)
        );

        if (resizedData == null) {
            return error.FailedToResizeImage;
        }

        return Image{
            .width = width,
            .height = height,
            .channels = 3,
            .raw = resizedData,
        };
    }

    pub fn deinit(self: *const Image) void {
        cImage.stbi_image_free(self.raw);
    }

    pub fn data(self: *const Image) [*]u8 {
        return self.raw;
    }

    pub fn getColorAt(self: *const Image, x: i32, y: i32) Color {
        const index: usize = @intCast((y * self.width + x) * self.channels);

        // Channel order is RGBA
        return Color{
            .r = self.raw[index + 0],
            .g = self.raw[index + 1],
            .b = self.raw[index + 2],
            .a = self.raw[index + 3],
        };
    }
};
