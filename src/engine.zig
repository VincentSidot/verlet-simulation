const std = @import("std");
const math = std.math;

const r = @import("raylib.zig").c;

const a = std.heap.smp_allocator;

const Vec2 = r.Vector2;
const Color = r.Color;

pub const Particle = struct {
    const Self = @This();

    position: Vec2,
    position_old: Vec2,

    acceleration: Vec2,
    radius: f32,

    color: Color,

    pub fn init(position: Vec2, radius: f32) Self {
        return Self{
            .position = position,
            .position_old = position,
            .acceleration = .{ .x = 0.0, .y = 0.0 },
            .radius = radius,
            .color = r.WHITE,
        };
    }

    pub fn update(self: *Self, dt: f32) void {
        const delta = r.Vector2Subtract(self.position, self.position_old);

        const dt2 = dt * dt;

        self.position_old = self.position;
        self.position = r.Vector2Add(
            self.position,
            r.Vector2Add(
                delta,
                r.Vector2Scale(
                    self.acceleration,
                    dt2,
                ),
            ),
        );

        // Reset acceleration
        self.acceleration = .{ .x = 0.0, .y = 0.0 };
    }

    pub fn accelerate(self: *Self, acceleration: Vec2) void {
        self.acceleration = r.Vector2Add(self.acceleration, acceleration);
    }

    pub fn setVelocity(self: *Self, velocity: Vec2, dt: f32) void {
        self.position_old = r.Vector2Subtract(
            self.position,
            r.Vector2Scale(
                velocity,
                dt,
            ),
        );
    }

    pub fn addVelocity(self: *Self, velocity: Vec2, dt: f32) void {
        self.position_old = r.Vector2Subtract(
            self.position_old,
            r.Vector2Scale(
                velocity,
                dt,
            ),
        );
    }

    pub inline fn getVelocity(self: *const Self, dt: f32) Vec2 {
        return r.Vector2Scale(
            r.Vector2Subtract(self.position, self.position_old),
            1.0 / dt,
        );
    }

    pub fn setColor(self: *Self, color: Color) void {
        self.color = color;
    }
};

const ParticleArray = std.array_list.AlignedManaged(Particle, null);

pub const Solver = struct {
    const Self = @This();

    // Fields

    gravity: Vec2 = .{ .x = 0.0, .y = 1000.0 },
    objects: ParticleArray,

    time: f32 = 0.0,
    timeDt: f32 = 0.0,

    subSteps: u32 = 1,
    constraints: Constraints = Constraints{ .circle = .{
        .center = .{ .x = 0.0, .y = 0.0 },
        .radius = 0.0,
    } },

    // Public methods

    pub fn init() Self {
        return Self{
            .objects = ParticleArray.init(a),
        };
    }

    pub fn deinit(self: *Self) void {
        self.objects.deinit();
    }

    pub fn addObject(self: *Self, pos: Vec2, radius: f32) *Particle {
        self.objects.append(Particle.init(pos, radius)) catch unreachable;
        return &self.objects.items[self.objects.items.len - 1];
    }

    pub fn update(self: *Self) void {
        self.time += self.timeDt;
        const stepDt = self.getStepTime();

        for (0..self.subSteps) |_| {
            self.applyGravity();
            self.checkCollisions(stepDt);
            self.applyConstraints();
            self.updateObjects(stepDt);
        }
    }

    pub fn setTimeStep(self: *Self, dt: f32) void {
        self.timeDt = dt;
    }

    pub fn setRate(self: *Self, rate: u32) void {
        self.timeDt = 1.0 / @as(f32, @floatFromInt(rate));
    }

    pub fn setConstraints(self: *Self, constraints: Constraints) void {
        self.constraints = constraints;
    }

    pub fn setSubSteps(self: *Self, subSteps: u32) void {
        self.subSteps = subSteps;
    }

    pub fn setObjectVelocity(self: *Self, obj: *Particle, velocity: Vec2) void {
        const stepDt = self.getStepTime();
        obj.setVelocity(velocity, stepDt);
    }

    pub fn getObjectsCount(self: *const Self) usize {
        return self.objects.items.len;
    }

    pub fn getTime(self: *const Self) f32 {
        return self.time;
    }

    // Private methods

    fn getStepTime(self: *const Self) f32 {
        return self.timeDt / @as(f32, @floatFromInt(self.subSteps));
    }

    fn applyGravity(self: *Self) void {
        for (self.objects.items) |*obj| {
            obj.accelerate(self.gravity);
        }
    }

    fn checkCollisions(self: *Self, dt: f32) void {
        _ = dt;
        const reponseCoef = 0.75;

        for (self.objects.items, 0..) |*objA, i| {
            for (self.objects.items[i + 1 ..]) |*objB| {
                const v = r.Vector2Subtract(objA.position, objB.position);
                const dist2 = r.Vector2LengthSqr(v);
                const minDist = objA.radius + objB.radius;
                const minDist2 = minDist * minDist;

                if (dist2 < minDist2) {
                    const dist = math.sqrt(dist2);
                    const n = r.Vector2Scale(v, 1.0 / dist);

                    const massRatioA = objB.radius / (objA.radius + objB.radius);
                    const massRatioB = objA.radius / (objA.radius + objB.radius);
                    const delta = 0.5 * reponseCoef * (dist - minDist);

                    // Positional correction
                    objA.position = r.Vector2Subtract(
                        objA.position,
                        r.Vector2Scale(
                            n,
                            delta * massRatioA,
                        ),
                    );
                    objB.position = r.Vector2Add(
                        objB.position,
                        r.Vector2Scale(
                            n,
                            delta * massRatioB,
                        ),
                    );
                }
            }
        }
    }

    fn applyConstraints(self: *Self) void {
        for (self.objects.items) |*obj| {
            self.constraints.applyConstraints(obj);
        }
    }

    fn updateObjects(self: *Self, dt: f32) void {
        for (self.objects.items) |*obj| {
            obj.update(dt);
        }
    }
};

const ConstraintsCircle = struct {
    const Self = @This();

    center: Vec2,
    radius: f32,

    inline fn applyConstraints(self: *const Self, obj: *Particle) void {
        const to_obj = r.Vector2Subtract(self.center, obj.position);
        const dist = r.Vector2Length(to_obj);

        const deltaRadius = self.radius - obj.radius;

        if (dist > deltaRadius) {
            const n = r.Vector2Scale(to_obj, 1.0 / dist);
            obj.position = r.Vector2Subtract(
                self.center,
                r.Vector2Scale(
                    n,
                    deltaRadius,
                ),
            );
        }
    }
};

const ConstraintsBox = struct {
    const Self = @This();

    min: Vec2,
    max: Vec2,

    inline fn applyConstraints(self: *const Self, obj: *Particle) void {
        _ = self;
        _ = obj;

        @panic("Not implemented yet");
    }
};

pub const Constraints = union(enum) {
    circle: ConstraintsCircle,
    box: ConstraintsBox,

    fn applyConstraints(self: Constraints, obj: *Particle) void {
        switch (self) {
            .circle => |c| {
                c.applyConstraints(obj);
            },
            .box => |b| {
                b.applyConstraints(obj);
            },
        }
    }
};
