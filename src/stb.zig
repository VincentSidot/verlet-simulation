// stb - Zig bindings for stb libraries

pub const image = @cImport({
    @cInclude("stb_image/stb_image.h");
    @cInclude("stb_image/stb_image_resize2.h");
});
