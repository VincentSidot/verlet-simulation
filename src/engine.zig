const std = @import("std");
const math = std.math;

const r = @import("raylib.zig").c;

const a = std.heap.smp_allocator;

const Vec2 = r.Vector2;
const Rectangle = r.Rectangle;
const Color = r.Color;

inline fn clamp(comptime T: type, value: T, min: T, max: T) T {
    if (value <= min) return min;
    if (value >= max) return max;
    return value;
}

pub const Particle = struct {
    const Self = @This();

    position: Vec2,
    position_old: Vec2,

    acceleration: Vec2,
    radius: f32,

    color: Color,
    gridIndex: ?usize = null,

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

    fn handleCollision(objA: *Self, objB: *Self, reponseCoef: f32) bool {
        const v = r.Vector2Subtract(objA.position, objB.position);
        const dist2 = r.Vector2LengthSqr(v);
        const minDist = objA.radius + objB.radius;
        const minDist2 = minDist * minDist;

        if (dist2 < minDist2 and dist2 > 0.0) {
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
            return true;
        } else {
            return false;
        }
    }
};

const ParticleArray = std.array_list.AlignedManaged(Particle, null);

pub const Grid = struct {
    const Self = @This();

    const ParticleInfo = struct {
        idx: usize,
        raw: *Particle,
    };

    const NeighborIterator = struct {
        grid: *const Grid,
        particle: ParticleInfo,
        currentXOffset: isize = -1,
        currentYOffset: isize = -1,

        currentNode: ?*std.DoublyLinkedList.Node = null,

        fn incrementOffset(self: *NeighborIterator) bool {
            self.currentXOffset += 1;
            if (self.currentXOffset > 1) {
                self.currentXOffset = -1;
                self.currentYOffset += 1;
            }
            if (self.currentYOffset > 1) {
                return false;
            }

            return true;
        }

        pub fn next(self: *NeighborIterator) ?usize {
            if (self.currentNode) |node| {
                self.currentNode = node.next;
                const element = GridElement.getNode(node);
                const particleIdx = element.particleIdx;
                if (particleIdx != self.particle.idx) {
                    return particleIdx;
                } else {
                    return self.next();
                }
            } else {
                // Move to next cell
                if (!self.incrementOffset()) {
                    return null; // No more cells
                }

                const centerPos = self.particle.raw.gridIndex orelse unreachable;
                const centerY, const centerX = self.grid.computeDualIndex(centerPos);

                const offsetX = @as(isize, @intCast(centerX)) + self.currentXOffset;
                const offsetY = @as(isize, @intCast(centerY)) + self.currentYOffset;

                const gridSize = self.grid.gridSize;

                if (offsetX >= 0 and offsetX < gridSize.cols) {
                    if (offsetY >= 0 and offsetY < gridSize.rows) {
                        const cellIndex = self.grid.computeSingleIndex(
                            @intCast(offsetX),
                            @intCast(offsetY),
                        );
                        const cell = &self.grid.gridData[cellIndex];
                        self.currentNode = cell.elements.first;
                    }
                }
                return self.next();
            }
        }
    };

    const GridElement = struct {
        elements: std.DoublyLinkedList,

        const Node = struct {
            particleIdx: usize,
            node: std.DoublyLinkedList.Node,

            fn init(particleIdx: usize) *Node {
                const self = a.create(Node) catch unreachable;
                self.* = .{
                    .particleIdx = particleIdx,
                    .node = .{},
                };
                return self;
            }

            fn deinit(self: *Node) void {
                a.destroy(self);
            }
        };

        fn getNode(raw: *std.DoublyLinkedList.Node) *Node {
            return @fieldParentPtr("node", raw);
        }

        fn init() GridElement {
            return .{ .elements = .{} };
        }

        fn deinit(self: *GridElement) void {
            var it = self.elements.first;
            while (it) |raw| {
                const node: *Node = GridElement.getNode(raw);
                it = raw.next;
                self.elements.remove(raw);
                node.deinit();
            }
        }

        fn addParticle(self: *GridElement, particleIdx: usize) void {
            self.elements.append(&Node.init(particleIdx).node);
        }

        fn removeParticle(self: *GridElement, particleIdx: usize) void {
            var it = self.elements.first;
            while (it) |raw| {
                const node: *Node = GridElement.getNode(raw);
                it = raw.next;
                if (node.particleIdx == particleIdx) {
                    self.elements.remove(raw);
                    node.deinit();
                    return;
                }
            }
        }
    };

    const GridSize = struct {
        rows: usize,
        cols: usize,
    };

    constraintBounds: Rectangle,
    gridSize: GridSize,
    gridData: []GridElement,

    fn init(constraints: *const Constraints, size: GridSize) Self {
        const bounds = constraints.getBoundingBox();
        const totalCells = size.rows * size.cols;

        var self: Self = .{
            .constraintBounds = bounds,
            .gridSize = size,
            .gridData = a.alloc(GridElement, totalCells) catch unreachable,
        };

        for (0..totalCells) |i| {
            self.gridData[i] = GridElement.init();
        }

        return self;
    }

    fn deinit(self: *Self) void {
        for (self.gridData) |*cell| {
            cell.deinit();
        }
        a.free(self.gridData);
    }

    fn updateParticlePosition(self: *const Self, particle: ParticleInfo) void {
        const targetIndex = self.computeGridIndex(particle.raw.position);
        self.moveParticle(particle, targetIndex);
    }

    fn moveParticle(self: *const Self, particle: ParticleInfo, targetIndex: usize) void {
        if (particle.raw.gridIndex) |oldIndex| {
            if (oldIndex == targetIndex) return;

            self.gridData[oldIndex].removeParticle(particle.idx);
        }

        self.gridData[targetIndex].addParticle(particle.idx);
        particle.raw.gridIndex = targetIndex;
    }

    fn computeGridIndex(self: *const Self, pos: Vec2) usize {
        // Clamp to grid size by security
        const relX = (pos.x - self.constraintBounds.x) / self.constraintBounds.width;
        const relY = (pos.y - self.constraintBounds.y) / self.constraintBounds.height;

        const gridX: usize = clamp(
            usize,
            @as(usize, @intFromFloat(relX * @as(f32, @floatFromInt(self.gridSize.cols)))),
            0,
            self.gridSize.cols - 1,
        );
        const gridY: usize = clamp(
            usize,
            @as(usize, @intFromFloat(relY * @as(f32, @floatFromInt(self.gridSize.rows)))),
            0,
            self.gridSize.rows - 1,
        );

        return self.computeSingleIndex(gridX, gridY);
    }

    fn getIterator(self: *Grid, particle: ParticleInfo) NeighborIterator {
        return .{ .grid = self, .particle = particle };
    }

    inline fn computeSingleIndex(self: *const Self, x: usize, y: usize) usize {
        return y * self.gridSize.cols + x;
    }

    inline fn computeDualIndex(self: *const Self, idx: usize) struct { usize, usize } {
        const row = idx / self.gridSize.cols;
        const col = idx % self.gridSize.cols;
        return .{ row, col };
    }
};

pub const Solver = struct {
    const Self = @This();

    // Fields

    gravity: Vec2 = .{ .x = 0.0, .y = 1000.0 },
    objects: ParticleArray,

    time: f32 = 0.0,
    timeDt: f32 = 0.0,

    subSteps: u32 = 1,
    constraints: Constraints,

    grid: Grid,

    // Public methods

    pub fn init(constraints: Constraints) Self {
        return Self{
            .constraints = constraints,
            .objects = ParticleArray.init(a),
            .grid = Grid.init(&constraints, .{
                .cols = 100,
                .rows = 100,
            }),
        };
    }

    pub fn deinit(self: *Self) void {
        self.objects.deinit();
        self.grid.deinit();
    }

    pub fn addObject(self: *Self, pos: Vec2, radius: f32) *Particle {
        self.objects.append(Particle.init(pos, radius)) catch unreachable;
        const idx = self.objects.items.len - 1;
        const raw = &self.objects.items[idx];
        self.grid.updateParticlePosition(.{
            .idx = idx,
            .raw = raw,
        });

        return raw;
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

        for (self.objects.items, 0..) |*objA, objAIdx| {
            const objAInfo: Grid.ParticleInfo = .{
                .idx = objAIdx,
                .raw = objA,
            };
            var iterator = self.grid.getIterator(objAInfo);
            var maybeParticle = iterator.next();

            while (maybeParticle) |objBIndex| : (maybeParticle = iterator.next()) {
                const objB = &self.objects.items[objBIndex];
                if (objA.handleCollision(objB, reponseCoef)) {
                    self.grid.updateParticlePosition(objAInfo);
                    self.grid.updateParticlePosition(.{ .idx = objBIndex, .raw = objB });
                }
            }
        }

        // for (self.objects.items, 0..) |*objA, i| {
        //     for (self.objects.items[i + 1 ..]) |*objB| {
        //         if (objA.handleCollision(objB, reponseCoef)) {
        //             self.grid.updateParticlePosition(objA);
        //             self.grid.updateParticlePosition(objB);
        //         }
        //     }
        // }
    }

    fn applyConstraints(self: *Self) void {
        for (self.objects.items) |*obj| {
            self.constraints.applyConstraints(obj);
        }
    }

    fn updateObjects(self: *Self, dt: f32) void {
        for (self.objects.items, 0..) |*obj, i| {
            obj.update(dt);
            self.grid.updateParticlePosition(.{
                .idx = i,
                .raw = obj,
            });
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

    fn applyConstraints(self: *const Constraints, obj: *Particle) void {
        switch (self.*) {
            .circle => |c| {
                c.applyConstraints(obj);
            },
            .box => |b| {
                b.applyConstraints(obj);
            },
        }
    }

    fn getBoundingBox(self: *const Constraints) Rectangle {
        switch (self.*) {
            .circle => |c| {
                return .{
                    .x = c.center.x - c.radius,
                    .y = c.center.y - c.radius,
                    .width = c.radius * 2.0,
                    .height = c.radius * 2.0,
                };
            },
            .box => |b| {
                return .{
                    .x = b.min.x,
                    .y = b.min.y,
                    .width = b.max.x - b.min.x,
                    .height = b.max.y - b.min.y,
                };
            },
        }
    }
};
