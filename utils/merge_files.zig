// merge_files.zig - Merges all source files into a single file for easier distribution
const std = @import("std");

const a = std.heap.smp_allocator;

const str = []const u8;
const StringArray = std.ArrayList(str);
const String = std.ArrayList(u8);

const Args = struct {
    excludedDirs: StringArray,
    outputPath: str,
};

const DEFAULT_PATH = "output.zig.merged";

fn strContains(targetStr: str, subStr: str) bool {
    return std.mem.indexOf(u8, targetStr, subStr) != null;
}

fn checkExcluded(path: str, args: *const Args) bool {
    for (args.excludedDirs.items) |excludeDir| {
        if (strContains(path, excludeDir)) {
            return true;
        }
    }
    return false;
}

fn listDirectory(path: str, args: *const Args, arrayList: *StringArray) !void {
    if (checkExcluded(path, args)) {
        return;
    }

    var dir = try std.fs.cwd().openDir(path, .{ .iterate = true, .access_sub_paths = false });
    defer dir.close();

    var dirIterator = dir.iterate();

    while (try dirIterator.next()) |entry| {
        if (entry.kind == .file) {
            const pathLen = path.len + 1 + entry.name.len; // +1 for "/"

            var buffer = try a.alloc(u8, pathLen);
            errdefer a.free(buffer);

            @memcpy(buffer[0..path.len], path);
            buffer[path.len] = '/';
            @memcpy(buffer[path.len + 1 ..], entry.name);

            try arrayList.append(a, buffer);
        } else if (entry.kind == .directory) {
            // Recursively list subdirectory
            const subDirPathLen = path.len + 1 + entry.name.len; // +1 for "/"
            var subDirBuffer = try a.alloc(u8, subDirPathLen);
            defer a.free(subDirBuffer);

            @memcpy(subDirBuffer[0..path.len], path);
            subDirBuffer[path.len] = '/';
            @memcpy(subDirBuffer[path.len + 1 ..], entry.name);

            try listDirectory(subDirBuffer, args, arrayList);
        }
    }
}

fn appendFileContent(path: []const u8, output: *String) !void {
    var file = try std.fs.cwd().openFile(
        path,
        .{
            .mode = .read_only,
        },
    );
    defer file.close();

    // Append simple header
    try output.appendSlice(a, "// Begin file: ");
    try output.appendSlice(a, path);
    try output.appendSlice(a, "\n");

    // Read file content
    const fileSize = try file.getEndPos();
    var buffer = try a.alloc(u8, fileSize);
    defer a.free(buffer);

    const bytesRead = try file.readAll(buffer);
    try output.appendSlice(a, buffer[0..bytesRead]);
}

fn writeContent(args: *const Args, content: []const u8) !void {
    const path = args.outputPath;

    var isExclusive = true;
    if (std.mem.eql(u8, path, DEFAULT_PATH)) {
        isExclusive = false; // Allow overwriting default path
    }

    var file = try std.fs.cwd().createFile(
        path,
        .{
            .read = false,
            .exclusive = isExclusive,
        },
    );
    defer file.close();

    try file.writeAll(content);
}

fn parseArgs(args: *Args) !void {
    var getArgs = std.process.args();
    _ = getArgs.next(); // Skip program name

    while (getArgs.next()) |arg| {
        if (std.mem.eql(u8, arg, "--output") or std.mem.eql(u8, arg, "-o")) {
            const outputArg = getArgs.next() orelse {
                return error.MissingOutputPath;
            };
            args.outputPath = outputArg;
        } else if (std.mem.eql(u8, arg, "--exclude-dir") or std.mem.eql(u8, arg, "-e")) {
            const excludeDir = getArgs.next() orelse {
                return error.MissingExcludeDir;
            };
            try args.excludedDirs.append(a, excludeDir);
        } else {
            return error.UnknownArgument;
        }
    }
}

pub fn main() !void {
    // var args = std.process.args();
    // _ = args.next(); // Skip program name
    // const outputPath = args.next() orelse DEFAULT_PATH;

    var args = Args{
        .excludedDirs = StringArray.empty,
        .outputPath = DEFAULT_PATH,
    };
    defer args.excludedDirs.deinit(a);

    try parseArgs(&args);

    std.debug.print("Merging source files into: {s}\n", .{args.outputPath});

    var fileList = StringArray.empty;

    defer {
        for (fileList.items) |file| {
            a.free(file); // Free each allocated file path
        }

        fileList.deinit(a);
    }

    try listDirectory("src", &args, &fileList);

    var outputContent = String.empty;
    defer outputContent.deinit(a);

    for (fileList.items) |file| {
        std.debug.print("Reading file: {s}\n", .{file});
        try appendFileContent(file, &outputContent);
        try outputContent.appendSlice(a, "\n\n");
    }

    try writeContent(&args, outputContent.items);
}
