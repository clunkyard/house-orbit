"""Orbit: a mobile hunter that circles its target at mid range and leads it.

Protocol 2, on the Python binding. The core tracks each robot it sees; Orbit
locks onto the nearest, circles it at its preferred range on irregular
reversals, leads the tracked motion and fires the heaviest shot that lands
in time and that the battery pays for. One pilot turns a desired world
velocity into throttle and turn, or into thrust, strafe and lift.
"""
import math

import clunkyard as cy

# Orbit's personality. Keys c2 and c3 override for 2D and 3D classic, f for
# every force ruleset, and a ruleset name for that ruleset alone.
PARAMS = {
    "searchArc": 60, "lockArc": 30, "lockMemory": 12, "range": 260, "speed": 85,
    "flipChance": 0.3, "flipMin": 2, "flipSpan": 2, "maxShot": 620, "governor": 40,
    "flight": 1.0, "slowFlight": 1.4, "weave": 2, "aimTol": 3, "reserve": 0.5,
    "c2": {"flipChance": 0.7, "flipMin": 0.6, "flipSpan": 1},
    "f": {"flipChance": 0.45, "flipMin": 1.5, "flipSpan": 1.5},
    "inertial-2d": {"flipChance": 0.5, "flipMin": 1.2, "flipSpan": 1.2},
}
TOP_SPEED = 95


def vec(v):
    return [v.x, v.y, v.z]


def length(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


class Orbit(cy.Robot):
    def on_start(self):
        classic = self.match.control == "direct"
        kind = ("c3" if self.match.is_3d else "c2") if classic else self.match.ruleset
        self.p = {k: v for k, v in PARAMS.items() if not isinstance(v, dict)}
        if not classic:
            self.p.update(PARAMS["f"])
        self.p.update(PARAMS.get(kind, {}))
        self.classic, self.three = classic, self.match.is_3d
        self.sudden_death = self.raw.number("hello.suddenDeath.startTick") or self.match.max_ticks
        self.rng = 7919 * sum(ord(ch) for ch in self.raw.text("hello.self.id") or "orbit") + 17
        self.target, self.sweep, self.flip, self.next_flip = None, None, 1, 0.0

    def rand(self):
        self.rng = (self.rng * 1103515245 + 12345) % 2147483648
        return self.rng / 2147483648

    def pick_target(self):
        """Keep the locked robot while the tracker remembers it; else take the nearest in view."""
        visible = {c.slot for c in self.sensors.contacts if c.source in ("scan", "proximity")}
        if self.target is not None and self.tracker.position(self.target) is None:
            self.target = None
        if self.target is None or (self.target not in visible and visible):
            nearest = self.sensors.nearest
            self.target = nearest.slot if nearest else None
        return visible

    def predicted(self, slot):
        position, velocity = vec(self.tracker.position(slot)), vec(self.tracker.velocity(slot))
        age = (self.clock.tick - self.tracker.last_seen(slot)) / self.match.tick_rate
        return [position[i] + velocity[i] * age for i in range(3)], velocity

    def power(self, distance, speed):
        """The heaviest shot that lands within the flight budget and that the battery pays for."""
        flight = self.p["slowFlight"] if speed < 30 else self.p["flight"]
        power = self.gun.power_for_flight_time(distance, flight)
        cap = 1.5 if self.battery.charge < 0.3 * self.battery.capacity else self.gun.max_power
        return round(max(0.5, min(power, cap)) * 10) / 10

    def avoid_storm(self, desired, pos):
        """Step out across the storm's path, on the side away from its centre."""
        storm = self.nav.storm
        if storm is None:
            return desired
        ax, ay = pos[0] - storm.center.x, pos[1] - storm.center.y
        d, sv = max(1.0, math.hypot(ax, ay)), math.hypot(storm.velocity.x, storm.velocity.y)
        margin = storm.radius + 60 + sv
        if d >= margin:
            return desired
        px, py = (-storm.velocity.y / sv, storm.velocity.x / sv) if sv > 1 else (ax / d, ay / d)
        if px * ax + py * ay < 0:
            px, py = -px, -py
        ex, ey = px + 0.5 * ax / d, py + 0.5 * ay / d
        en, push = max(0.01, math.hypot(ex, ey)), 95 * min(1, (margin - d) / 60)
        return [desired[0] * 0.3 + ex / en * push, desired[1] * 0.3 + ey / en * push, desired[2]]

    def pilot(self, desired, pos):
        me, three = self.me, self.three
        vel = vec(me.velocity)
        # The storm first; walls and the safe zone below still win.
        desired = self.avoid_storm(desired, pos)
        edges = vec(self.arena.size)
        for i in range(3 if three else 2):
            margin = 60 + vel[i] * vel[i] / 180
            if pos[i] < margin:
                desired[i] = max(desired[i], (margin - pos[i]) * 1.5)
            if pos[i] > edges[i] - margin:
                desired[i] = min(desired[i], (edges[i] - margin - pos[i]) * 1.5)
        # Head for the safe zone a few seconds before sudden death, then stay in it.
        zone = self.nav.zone
        centre = vec(zone.center) if zone else [e / 2 for e in edges]
        offset = [pos[i] - centre[i] for i in range(3)]
        if not three or (zone and zone.shape == "column"):
            offset[2] = 0
        r = length(offset)
        if zone:
            safe = max(40, zone.radius - 35)
        else:
            safe = math.inf if self.clock.tick < self.sudden_death - 100 else 280
        if r > safe:
            pull = min(95, 25 + r - safe)
            desired = [-x / r * pull for x in offset]
        g = self.arena.gravity.z
        if g:
            desired[2] = cy.angles.clamp((self.arena.ground_height(pos[0], pos[1]) + 115 - pos[2]) * 1.2, -35, 35)
        if not three:
            desired[2] = 0
        if self.classic:
            speed = length(desired)
            heading = math.degrees(math.atan2(desired[1], desired[0]))
            turn = cy.angles.angle_diff(heading, me.heading)
            throttle = cy.angles.clamp(speed / TOP_SPEED, 0, 1)
            # Reverse rather than turn around when the goal is behind.
            if abs(turn) > 100:
                turn = cy.angles.angle_diff(heading + 180, me.heading)
                throttle = -throttle
            pitch_turn = None
            if three:
                sign = -1 if throttle < 0 else 1
                pitch = math.degrees(math.atan2(desired[2], math.hypot(desired[0], desired[1]))) * sign
                pitch_turn = cy.angles.clamp((pitch - me.pitch) / 10)
            self.drive.set(throttle=throttle * max(0.0, math.cos(math.radians(min(80, abs(turn))))),
                           turn=cy.angles.clamp(turn / 15), pitch_turn=pitch_turn)
            return
        acc = [(desired[i] - vel[i]) * 2 for i in range(3)]
        acc[2] -= g
        # Each local actuator is bounded on its own, so gravity compensation
        # survives a lateral demand.
        att, w = me.attitude, me.angular_velocity
        local = [cy.angles.clamp(sum(acc[i] * a[i] for i in range(3)) / 100) for a in (vec(att.forward), vec(att.right), vec(att.up))]
        self.drive.forces(thrust=local[0], strafe=local[1], lift=local[2] if three else 0,
                          yaw_torque=cy.angles.clamp(-w.yaw / 60),
                          pitch_torque=cy.angles.clamp(-w.pitch / 60) if three else 0,
                          roll_torque=cy.angles.clamp(-w.roll / 60) if three else 0, brake=0)

    def on_turn(self):
        p, me, three, tick = self.p, self.me, self.three, self.clock.tick
        now = self.clock.seconds
        pos = vec(me.position)
        visible = self.pick_target()
        centre = [e / 2 for e in vec(self.arena.size)]
        desired = [(centre[i] - pos[i]) * 0.45 for i in range(3)]
        track = self.predicted(self.target) if self.target is not None else None
        fresh = track is not None and tick - self.tracker.last_seen(self.target) < p["lockMemory"]
        cue = self.tracker.cue
        # Scanning: hold a fresh track with a narrow arc; otherwise sweep wide.
        if fresh:
            there = cy.Vec3(*track[0])
            self.scanner.set(heading=cy.angles.bearing_to(me.position, there),
                             pitch=cy.angles.elevation_to(me.position, there) if three else 0, arc=p["lockArc"])
        elif cue is not None:
            # Hunt toward the cue: scan across its bearing error and close in.
            bearing = cy.angles.bearing_to(me.position, cue.position)
            elevation = cy.angles.elevation_to(me.position, cue.position) if three else 0
            spread = min(30, max(cue.bearing_error, 3))
            self.scanner.set(heading=bearing + spread * math.sin(tick * 0.9), pitch=elevation,
                             arc=max(p["lockArc"], min(p["searchArc"], spread)))
            b, h = math.radians(bearing), math.radians(elevation)
            desired = [60 * math.cos(h) * math.cos(b), 60 * math.cos(h) * math.sin(b), 60 * math.sin(h)]
        else:
            self.sweep = cy.angles.normalize((me.heading if self.sweep is None else self.sweep) + p["searchArc"] * 0.9)
            turns = tick // math.ceil(360 / (p["searchArc"] * 0.9))
            self.scanner.set(heading=self.sweep, pitch=(-40, 0, 40)[turns % 3] if three else 0, arc=p["searchArc"])
        if track:
            there, velocity = track
            rel = [there[i] - pos[i] for i in range(3)]
            distance = length(rel)
            power = self.power(distance, length(velocity))
            aim = self.gun.lead_point(cy.Vec3(*there), cy.Vec3(*velocity), power)
            heading = aim.heading if aim else cy.angles.bearing_to(me.position, cy.Vec3(*there))
            pitch = (aim.pitch if aim else cy.angles.elevation_to(me.position, cy.Vec3(*there))) if three else 0
            self.gun.aim(heading, pitch)
            if now >= self.next_flip:
                if self.rand() < p["flipChance"]:
                    self.flip = -self.flip
                self.next_flip = now + p["flipMin"] + self.rand() * p["flipSpan"]
            # Circle the target at the preferred range, weaving vertically in 3D.
            u = [x / max(1.0, distance) for x in rel]
            tangent = [-u[1], u[0], 0.4 * math.sin(tick / 16) if three else 0]
            tn = max(0.01, length(tangent))
            radial = cy.angles.clamp((distance - p["range"]) * 0.65, -70, 70)
            desired = [tangent[i] / tn * p["speed"] * self.flip + u[i] * radial for i in range(3)]
            if three and not self.arena.gravity.z:
                desired[2] += 18 * math.cos(tick / 13) * p["weave"]
            now_aim = self.gun.aim_now
            aligned = (abs(cy.angles.angle_diff(heading, now_aim.heading)) < p["aimTol"]
                       and (not three or abs(pitch - now_aim.pitch) < p["aimTol"]))
            # Never fire from memory alone: the target must be in this scan.
            if self.target in visible and aligned and aim and distance < p["maxShot"] and self.gun.can_fire(power, p["reserve"]):
                self.gun.fire(power)
            self.status.set("ORBIT", distance)
        else:
            self.status.set("SEARCH", self.scanner.get("heading"))
        # On a low battery, hold speed to what the regeneration sustains.
        if self.battery.charge < p["governor"] / 100 * self.battery.capacity:
            sustainable = self.battery.sustainable_drive * TOP_SPEED
            speed = length(desired)
            if speed > sustainable:
                desired = [x * sustainable / speed for x in desired]
        self.pilot(desired, pos)


cy.run(Orbit())
