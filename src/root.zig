//! By convention, root.zig is the root source file when making a library.
const std = @import("std");

test "lib tests" {
    const configSrc = @import("config.zig");

    std.testing.refAllDecls(configSrc);
}
