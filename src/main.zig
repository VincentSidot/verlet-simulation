const std = @import("std");

const a = std.heap.smp_allocator;

const app = @import("app.zig");

const RunMode = app.RunMode;
const createConfigMode = app.CreateConfigMode;
const readConfigMode = app.ReadConfigMode;

fn printUsage(programName: []const u8) void {
    std.debug.print("Usage '{s} [opts] <mode>'\n", .{programName});
    std.debug.print("Options:\n", .{});
    std.debug.print("  -h, --help        Show this help message\n", .{});
    std.debug.print("  --no-rta               Disable FPS caps\n", .{});
    std.debug.print("Modes:\n", .{});
    std.debug.print("  create <image_path> <config_path>  Create a default configuration file\n", .{});
    std.debug.print("  read   <config_path>               Read and use the specified configuration file\n", .{});
}

fn checkArg(comptime long: []const u8, comptime short: ?[]const u8, arg: []const u8) bool {
    const longForm = "--" ++ long;
    if (std.mem.eql(u8, arg, longForm)) {
        return true;
    }

    if (short) |s| {
        const shortForm = "-" ++ s;
        if (std.mem.eql(u8, arg, shortForm)) {
            return true;
        }
    }

    return false;
}

fn parseArgs() !RunMode {
    var output: RunMode = .{
        .mode = undefined,
    };

    var args = std.process.args();

    const programName = args.next(); // Skip executable name

    var genericArgumentsParsing: bool = true;

    var rawMode = args.next();
    while (rawMode) |mode| : (rawMode = args.next()) {
        if (mode[0] != '-') {
            genericArgumentsParsing = false;
            break;
        }

        // Here you can parse generic arguments like --help, --version, etc.
        if (checkArg("help", "h", mode)) {
            printUsage(programName orelse "app");
            return error.HelpRequested;
        }

        if (checkArg("no-rta", null, mode)) {
            output.realTime = false;
            continue;
        }
    }

    if (rawMode == null) {
        printUsage(programName orelse "app");
        return error.InvalidArguments;
    }
    const mode = rawMode.?;

    if (std.mem.eql(u8, mode, "create")) {
        const imagePath = args.next() orelse {
            printUsage(programName orelse "app");
            return error.InvalidArguments;
        };
        const configPath = args.next() orelse {
            printUsage(programName orelse "app");
            return error.InvalidArguments;
        };
        output.mode = .{ .createConfig = .{ .imgPath = imagePath, .configPath = configPath } };
    } else if (std.mem.eql(u8, mode, "read")) {
        const configPath = args.next() orelse {
            printUsage(programName orelse "app");
            return error.InvalidArguments;
        };
        output.mode = .{ .readConfig = .{ .configPath = configPath } };
    } else {
        printUsage(programName orelse "app");
        return error.InvalidArguments;
    }

    return output;
}

pub fn main() !void {
    const mode = try parseArgs();

    std.debug.print("Running with args: {any}\n", .{mode});

    try app.run(mode);
}

// pub fn main() !void {
//     const r = @import("raylib.zig").c;

//     r.InitWindow(800, 600, "Verlet Simulation");
//     defer r.CloseWindow();

//     while (!r.WindowShouldClose()) {
//         r.BeginDrawing();
//         r.ClearBackground(r.RAYWHITE);
//         r.DrawText("Hello, Raylib with Zig!", 190, 200, 20, r.DARKGRAY);
//         r.EndDrawing();
//     }
// }
