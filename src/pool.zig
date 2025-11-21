const std = @import("std");

pub fn Pool(comptime size: usize, comptime T: type, comptime R: type, comptime callback: fn (T) R) type {
    comptime if (size < 1) {
        @compileError("Thread pool size must be at least 1");
    };

    const Thread = std.Thread;
    const Semaphore = std.Thread.Semaphore;
    const POOL_SIZE = size - 1;

    return struct {
        const Self = @This();

        pool: [POOL_SIZE]Thread = undefined,
        args: [POOL_SIZE + 1]T = undefined,
        results: [POOL_SIZE + 1]R = undefined,

        start: Semaphore = .{},
        end: Semaphore = .{},
        isAlive: bool = false,

        pub fn init(self: *Self) void {
            self.isAlive = true;

            for (0..POOL_SIZE) |i| {
                self.pool[i] = Thread.spawn(.{}, Self.run, .{ self, i }) catch unreachable;
            }
        }

        pub fn deinit(self: *Self) void {
            self.isAlive = false;

            for (0..POOL_SIZE) |_| {
                self.start.post();
            }

            for (0..POOL_SIZE) |i| {
                self.pool[i].join();
            }
        }

        pub fn execute(self: *Self, args: anytype) void {
            const ArgsType = @TypeOf(args);
            const args_type_info = @typeInfo(ArgsType);

            if (ArgsType != void and @typeInfo(T) == .@"struct") {
                switch (args_type_info) {
                    .@"struct" => |structInfo| {
                        inline for (structInfo.fields) |field| {
                            const field_name = field.name;
                            const field_type = field.type;

                            comptime {
                                if (!@hasField(T, field_name)) {
                                    @compileError("T does not have " ++ field_name ++ " field");
                                }
                                if (@FieldType(T, field_name) != field_type) {
                                    @compileError("Field " ++ field_name ++
                                        " type mismatch expected " ++ @typeName(@FieldType(T, field_name)) ++
                                        " but got " ++ @typeName(field_type));
                                }
                            }

                            // Set the field value for each thread argument
                            for (&self.args) |*arg| {
                                @field(arg, field_name) = @field(args, field_name);
                            }
                        }
                    },
                    else => {
                        @compileError("Expecting struct type or void as argument");
                    },
                }
            }

            for (0..POOL_SIZE) |_| {
                self.start.post();
            }

            // Run last task in the current thread
            const res = callback(self.args[POOL_SIZE]);
            if (R != void) {
                self.results[POOL_SIZE] = res;
            }

            for (0..POOL_SIZE) |_| {
                self.end.wait();
            }
        }

        fn run(self: *Self, idx: usize) void {
            while (true) {
                self.start.wait();

                if (!self.isAlive) {
                    break;
                }

                const res = callback(self.args[idx]);
                if (R != void) {
                    self.results[idx] = res;
                }

                self.end.post();
            }
        }
    };
}
