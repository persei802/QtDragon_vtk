#!/usr/bin/env python3
# VTKDragon - gcode parser
# Copyright (c) 2026  Jim Sloot <persei802@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import sys
import math
import re
from dataclasses import dataclass, field

@dataclass
class ReturnFromSubroutine(Exception):
    pass

@dataclass
class Subroutine:
    level: int
    number: int
    start: int
    end: int

@dataclass
class WhileBlock:
    level: int
    number: int
    condition: str
    start: int
    end: int

@dataclass
class RepeatBlock:
    level: int
    number: int
    count: str
    start: int
    end: int

@dataclass
class CannedCycle:
    code: str = ""
    x: float | None=None
    y: float | None=None
    z: float | None=None
    r_plane: float = 0.0
    depth: float = 0.0
    dwell: float | None=None
    peck: float | None=None
    return_mode: str = "G98"
    repetitions: int = 1
    active: bool = False

@dataclass
class ToolpathSegment:
    line_number: int
    motion: str
    points: list = field(default_factory=list)
    feed: float = 0.0
    feed_mode: str = "G94"
#    spindle: float = 0.0
#    spindle_on: bool = False
#    spindle_direction: int = 0
    tool: int = 0
    wcs: str = "G54"
    tlo: float = 0.0
    @property
    def start(self):
        return self.points[0]
    @property
    def end(self):
        return self.points[-1]

@dataclass
class GCodeState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    u: float = 0.0
    v: float = 0.0
    w: float = 0.0
    line_number: str = "0"
    line: str = ""
    absolute: bool = True
    polar: bool = False
    radius: float = 0.0
    angle: float = 0.0
    metric: bool = True
    plane: str = "G17"
    motion: str = "G0"
    feed_mode: str = "G94"
    feed: float = 0.0
    tool: int = 0
#    g92_pending: bool = False
#    g92_offset: tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    tool_length_offset: float = 0.0
    tool_length_comp: bool = False
    tool_length_register: int = 0
    parameters: dict = field(default_factory=dict)

@dataclass
class GCodeError(Exception):
    def __init__(self, line_number, line, message):
        self.line_number = line_number
        self.line = line
        self.message = message

        super().__init__(
            f"G-code error at line {line_number}: "
            f"{message}\n"
            f"    {line.strip()}"
        )

@dataclass
class ProgramEnd(Exception):
    pass

class GCodeParser:
    AXES = ("X", "Y", "Z", "A", "B", "C", "U", "V", "W")
    AXIS_INDEX = {axis: i for i, axis in enumerate(AXES)}

    def __init__(self, callback=None, threshold=1000, arc_segments=64):
        self.callback = callback
        self.segment_threshold = threshold
        self.num_segments = max(1, arc_segments)
        self.state = GCodeState()
        self.canned = CannedCycle()
        self.segments = []
        self.arc_tolerance = 1e-3
        self.arc_endpoint_tolerance = 0.002
        self.max_loop_iterations = 10000
        self.max_call_depth = 4
        self.dwell_time = 0.0
        self.tool_lengths = {}
        self.subroutines = {}
        self.repeats = {}
        self.whiles = {}
        self.call_stack = []
        self.tool_list = []
        self.error_occurred = False
        self.error_message = ""
        self.error_line_number = 0
        self.error_line = ""
        self.line_type = {
            "G0": "traverse",
            "G1": "feed",
            "G2": "arcfeed",
            "G3": "arcfeed",
            "G38.2" : "feed"}

#    def apply_coordinate_offset(self, point):
#        g92_offset = self.state.g92_offset
#        result = tuple(point[i] + g92_offset[i] for i in range(9))
#        if self.state.tool_length_comp:
#            result = list(result)
#            result[self.AXIS_INDEX["Z"]] += (self.state.tool_length_offset)
#            result = tuple(result)
#        return result

#    def make_g92(self, axes):
#        current = (
#            self.state.x,
#            self.state.y,
#            self.state.z,
#            self.state.a,
#            self.state.b,
#            self.state.c,
#            self.state.u,
#            self.state.v,
#            self.state.w,
#        )
#        g92 = list(self.state.g92_offset)
#        for letter, value in axes.items():
#            index = self.AXIS_INDEX[letter]
#            if (not self.state.metric and letter in ("X", "Y", "Z", "U", "V", "W")):
#                value *= 25.4
            # G92 establishes the offset required to make
            # the current position equal to the programmed value.
#            g92[index] = current[index] - value
#        self.state.g92_offset = tuple(g92)

    def set_tool_length(self, tool, length):
        self.tool_lengths[int(tool)] = float(length)

    def parse_file(self, filename):
        self.reset_state()
        self.error_occurred = False
        self.error_message = ""
        self.error_line_number = 0
        self.error_line = ""
        with open(filename, "r") as f:
            for line in f:
                line = re.sub("N\d+", "", line, re.IGNORECASE)
                self.lines.append(line.strip())
        try:
            self.extract_program(self.lines)
            self.extract_subroutines(self.lines)
            self.extract_repeats(self.lines)
            self.extract_whiles(self.lines)
            self.execute_lines(1, len(self.lines))
        except GCodeError as e:
            self.error_occurred = True
            self.error_message = str(e)
            return False
        except ProgramEnd:
            self.flush_segments()
        return True

    def execute_lines(self, start, end):
        self.state.line_number = start
        while self.state.line_number < end:
            line = self.lines[self.state.line_number - 1]
            sub_match = re.match(r"O(\d+)\s+SUB", line, re.IGNORECASE)
            call_match = re.match(r"O(\d+)\s+CALL", line, re.IGNORECASE)
            rep_match = re.match(r"O(\d+)\s+REPEAT", line, re.IGNORECASE)
            whl_match = re.match(r"O(\d+)\s+WHILE", line, re.IGNORECASE)
            # O-word SUB
            if sub_match:
                number = int(sub_match.group(1))
                line_number = self.subroutines[number].end
            # O-word CALL
            elif call_match:
                number = int(call_match.group(1))
                self.execute_subroutine(number)
            # O-word REPEAT
            elif rep_match:
                number = int(rep_match.group(1))
                self.execute_repeat(number)
                line_number = self.repeats[number].end
            # O-word WHILE
            elif whl_match:
                number = int(whl_match.group(1))
                self.execute_while(number)
                line_number = self.whiles[number].end
            else:
                self.parse_line()
            if len(self.segments) >= self.segment_threshold:
                self.flush_segments()
            self.state.line_number += 1

    def flush_segments(self):
        if not self.segments: return
        if self.callback:
            self.callback(self.segments)
        self.segments.clear()

    def execute_subroutine(self, number):
        if number not in self.subroutines:
            raise ValueError(f"Undefined SUB O{number}")
        if number in self.call_stack:
            chain = self.call_stack + [number]
            chain_text = " -> ".join(f"O{n}" for n in chain)
            raise RuntimeError(f"Recursive subroutine call detected: {chain_text}")
        if len(self.call_stack) >= self.max_call_depth:
            chain = self.call_stack + [number]
            chain_text = " -> ".join(f"O{n}" for n in chain)
            raise RuntimeError(f"Maximum subroutine nesting depth exceeded: {chain_text}")
        self.call_stack.append(number)
        sub = self.subroutines[number]
        try:
            self.execute_lines(sub.start, sub.end)
        finally:
            self.call_stack.pop()

    def return_from_subroutine(self):
        raise ReturnFromSubroutine

    def execute_repeat(self, number):
        if number not in self.repeats:
            raise ValueError(f"Undefined REPEAT O{number}")
        rep = self.repeats[number]
        count = int(self.evaluate_expression(rep.count))
        for _ in range(count):
            self.execute_lines(rep.start, rep.end)

    def execute_while(self, number):
        if number not in self.whiles:
            raise ValueError(f"Undefined WHILE O{number}")
        wblk = self.whiles[number]
        iterations = 0
        while iterations < self.max_loop_iterations:
            condition = self.evaluate_condition(wblk.condition)
            if condition:
                self.execute_lines(wblk.start, wblk.end)
                iterations += 1
            else:
                break

    def parse_line(self):
        line_number = self.state.line_number
        original_line = self.lines[line_number - 1]
        try:
            self._parse_line()
        except GCodeError:
            raise
        except ProgramEnd:
            raise
        except Exception as e:
            raise GCodeError(line_number, original_line, str(e)) from e

    def _parse_line(self):
        line_number = self.state.line_number
        line = self.lines[line_number - 1]
        line = line.split(";")[0]
        # filter out comments and Nxxx numbers
        line = re.sub(r"%|N\d+\s+|\(.*?\)", "", line, flags=re.IGNORECASE)
        line = line.strip()
        if not line: return
        if line.startswith("o") or line.startswith("O"): return
        if line.startswith("#"):
            self.resolve_parameters(line)
            return
        # tokenize words
        axes, arc_params, feed, tool, m_codes, values = self.tokenize_words(line)
        self.update_modal_state(feed=feed, tool=tool, h_value=values.get('H'))
        self.handle_m_codes(m_codes)
        # handle G92
#        if self.state.g92_pending:
#            self.make_g92(axes)
#            self.state.g92_pending = False
#            return
        # check if a canned cycle is active
        if self.canned.active:
            self.canned.x = axes.get("X") or self.state.x
            self.canned.y = axes.get("Y") or self.state.y
            self.canned.repetitions = values.get("L") or 1
            if self.canned.code in ("G81", "G82", "G83"):
                self.execute_canned_drill()
            return
        # generate motion
        if self.state.motion in ("G0", "G1", "G38.2", "G38.3", "G38.4", "G38.5"):
            if axes:
                x = axes.get("X")
                y = axes.get("Y")
                z = axes.get("Z")
                a = axes.get("A")
                b = axes.get("B")
                c = axes.get("C")
                u = axes.get("U")
                v = axes.get("V")
                w = axes.get("W")
                self.make_motion(x, y, z, a, b, c, u, v, w)
            elif self.state.polar:
                x, y = self.resolve_polar_xy(self.state.radius, self.state.angle)
                self.make_motion(x=x, y=y)
        elif self.state.motion in ("G2", "G3"):
            if axes or arc_params:
                self.make_arc(axes, arc_params)
        elif self.state.motion == "G4":
            if values.get('P') is None:
                raise ValueError("G4 requires P")
            self.dwell_time += values['P']

    def handle_gcode(self, code):
        if code in ('0', '1', '2', '3', '4', '38.2', '38.3', '38.4', '38.5', '80'):
            self.canned.active = False
            self.canned.x = None
            self.canned.y = None
            self.canned.return_mode = "G98"
            self.state.motion = f"G{code}"
        elif code in ('17', '18', '19'):
            self.state.plane = f"G{code}"
        elif code == '20':
            self.state.metric = False
        elif code == '21':
            self.state.metric = True
#        elif code in ('28', '40'):
#            pass
        elif code == '43':
            self.state.tool_length_comp = True
        elif code == '49':
            self.state.tool_length_comp = False
            self.state.tool_length_register = 0
            self.state.tool_length_offset = 0.0
#        elif code in ('53', '54', '55', '56', '57', '58', '59', '59.1', '59.2'):
#            pass
#        elif code == '64':
#            pass
        elif code in ('81', '82', '83'):
            self.canned.active = True
            self.canned.code = f"G{code}"
            self.canned.z = self.state.z
        elif code == '90':
            self.state.absolute = True
        elif code == '91':
            self.state.absolute = False
        elif code == '92':
            self.state.g92_pending = True
        elif code == '92.1':
            self.state.g92_offset = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        elif code in ('93', '94'):
            self.state.feed_mode = f"G{code}"
        elif code in ('98', '99'):
            self.canned.return_mode = f"G{code}"

    def handle_polar(self, letter, word):
        if word.startswith("#"):
            num = word.replace("#", "").strip()
            value = self.state.parameters[int(num)]
        elif word.startswith("["):
            value = self.evaluate_expression(word)
        else:
            value = float(word)
        if letter == "@":
            self.state.radius = value if self.state.metric else value * 25.4
        elif letter == "^":
            self.state.angle = value
        self.state.polar = True

    def handle_axis_word(self, letter, word):
        if word.startswith("#"):
            num = word.replace("#", "").strip()
            value = self.state.parameters[int(num)]
        elif word.startswith("["):
            value = self.evaluate_expression(word)
        else:
            value = float(word)
        if not self.state.metric:
            value *= 25.4
        if letter == "Z" and self.canned.active:
            self.canned.depth = value
        return value

    def update_modal_state(self, feed=None, tool=None, h_value=None):
        if feed is not None:
            self.state.feed = feed
        if tool is not None:
            self.state.tool = tool
        if h_value is not None:
            h_value = int(h_value)
            if h_value not in self.tool_lengths:
                raise ValueError(f"Unknown tool length register H{h_value}")
            self.state.tool_length_register = int(h_value)
            if h_value not in self.tool_lengths:
                raise ValueError(f"Unknown tool length register H{h_value}")
            self.state.tool_length_register = h_value
            self.state.tool_length_offset = self.tool_lengths[h_value]

    def handle_m_codes(self, m_codes):
        for m in m_codes:
            if m == 2:
                raise ProgramEnd
            elif m == 6:
                self.tool_list.append(self.state.tool)
            elif m == 30:
                raise ProgramEnd

    def execute_canned_drill(self):
        cycle_start_x = self.state.x
        cycle_start_y = self.state.y
        old_z = self.state.z
        repetitions = self.canned.repetitions or 1
        r_plane, depth, return_z = self.resolve_cycle_z(self.canned.depth, self.canned.r_plane, old_z)
        # preliminary move to R plane
        self.make_motion(z=r_plane, absolute=True)
        for repeat in range(repetitions):
            if self.state.polar:
                cycle_start_radius = math.hypot(cycle_start_x, cycle_start_y)
                cycle_start_angle = math.atan2(cycle_start_y, cycle_start_x)
                cycle_start_angle = math.degrees(cycle_start_angle)
                x, y = self.resolve_polar_xy(cycle_start_radius, cycle_start_angle, repeat)
            else:
                x, y = self.resolve_cycle_xy(cycle_start_x, cycle_start_y, repeat)
            # preliminary move
            self.make_motion(x=x, y=y, absolute=True)
            # cycle specific drilling
            if self.canned.code == "G81":
                self.execute_g81(depth)
            elif self.canned.code == "G82":
                self.execute_g82(depth)
            elif self.canned.code == "G83":
                self.execute_g83(depth)
            else:
                raise GCodeError(self.state.line_number,
                    f"Unsupported canned cycle {self.canned.code}",
                    self.lines[self.state.line_number - 1])
            self.make_motion(z=return_z, absolute=True)

    def resolve_cycle_xy(self, start_x, start_y, repeat=0):
        if self.state.absolute:
            x = start_x if self.canned.x is None else self.canned.x
            y = start_y if self.canned.y is None else self.canned.y
        else:
            dx = self.canned.x or 0.0
            dy = self.canned.y or 0.0
            x = start_x + dx * (repeat + 1)
            y = start_y + dy * (repeat + 1)
        return x, y

    def resolve_polar_xy(self, start_radius, start_angle, repeat=0):
        if self.state.absolute:
            target_radius = start_radius if self.state.radius is None else self.state.radius
            target_angle = start_angle if self.state.angle is None else self.state.angle
        else:
            dr = self.state.radius or 0.0
            da = self.state.angle or 0.0
            target_radius = start_radius + dr * (repeat + 1) 
            target_angle = start_angle + da * (repeat + 1)

        rads = math.radians(target_angle)
        x = target_radius * math.cos(rads)
        y = target_radius * math.sin(rads)
        return x, y

    def resolve_cycle_z(self, z, r, old_z):
        start_z = self.state.z
        if self.state.absolute:
            r_plane = r
            depth = z
        else:
            r_plane = start_z + r
            depth = r_plane + z
        if self.canned.return_mode == "G98":
            return_z = max(old_z, r_plane)
        else:
            return_z = r_plane
        return r_plane, depth, return_z

    def execute_g81(self, depth):
        self.make_motion(z=depth, mode="G1", absolute=True)

    def execute_g82(self, depth):
        self.dwell_time += self.canned.dwell
        self.make_motion(z=depth, mode="G1", absolute=True)

    def execute_g83(self, depth):
        r = self.canned.r_plane
        peck = self.canned.peck
        if peck <= 0:
            raise GCodeError(self.state.line_number, "Peck value must be > 0", self.lines[self.state.line_number - 1])
        current_z = r
        while True:
            next_z = current_z - peck
            if next_z < depth: next_z = depth
            self.make_motion(z=next_z, mode="G1", absolute=True)
            if next_z == depth:
                break
            self.make_motion(z=r, absolute=True)
            current_z = next_z

    def tokenize_words(self, line):
        pattern = r"([A-Za-z@^])\s*([-+]?(?:\d+(?:\.\d*)?|\#\d*|\.\d+|\[[^\]]+\]))"
        matches = list(re.finditer(pattern, line, re.VERBOSE))
        if not matches or matches[0].group(1) == "O":
            # axes, arc_params, feed, tool, mcodes, values
            return({}, {}, None, None, [], {})
        axes = {}
        arc_params = {}
        feed = None
        tool = None
        m_codes = []
        values = {}
        self.state.radius = None
        self.state.angle = None
        self.state.polar = False
        for match in matches:
            letter = match.group(1)
            value = match.group(2)
            letter = letter.upper()
            if letter in ("@", "^"):
                self.handle_polar(letter, value)
            elif letter == "G":
                self.handle_gcode(value)
            elif letter in self.AXES:
                axes[letter] = self.handle_axis_word(letter, value)
            elif letter in ("I", "J", "K", "R"):
                value = float(value) if self.state.metric else float(value) * 25.4
                arc_params[letter] = value
                if letter == "R":
                    values["R"] = value
                    if self.canned.active:
                        self.canned.r_plane = value
            elif letter == "F":
                feed = float(value) if self.state.metric else float(value) * 25.4
            elif letter == "H":
                values['H'] = float(value)
            elif letter == 'L':
                values['L'] = int(value)
            elif letter == "M":
                m_codes.append(int(value))
            elif letter == "P":
                if self.canned.active:
                    self.canned.dwell = float(value)
                else:
                    values['P'] = float(value)
            elif letter == "Q":
                if self.canned.active:
                    self.canned.peck = float(value) if self.state.metric else float(value) * 25.4
                else:
                    values['Q'] = int(value)
            elif letter == "S":
                pass
            elif letter == "T":
                tool = int(value)
            else:
                raise ValueError(f"Unsupported G-code word {letter}")
                
        return axes, arc_params, feed, tool, m_codes, values

    def make_motion(self, x=None, y=None, z=None, a=None, b=None, c=None, u=None, v=None, w=None, mode=None, absolute=None):
        start = (self.state.x, self.state.y, self.state.z,
                 self.state.a, self.state.b, self.state.c,
                 self.state.u, self.state.v, self.state.w)
        end = list(start)
        motion_mode = self.state.motion if mode is None else mode
        distance_absolute = self.state.absolute if absolute is None else absolute
        for i, axis in enumerate((x, y, z, a, b, c, u, v, w)):
            if axis is None:
                continue
            if distance_absolute:
                end[i] = axis
            else:
                end[i] += axis
        # g92 offsets apply to segments but not to states
#        points = [self.apply_coordinate_offset(end)]
        points = [(end)]
        segment = (self.state.line_number, self.line_type[motion_mode], points, self.state.feed, self.state.feed_mode)
        self.segments.append(segment)
        self.state.x = end[0]
        self.state.y = end[1]
        self.state.z = end[2]
        self.state.a = end[3]
        self.state.b = end[4]
        self.state.c = end[5]
        self.state.u = end[6]
        self.state.v = end[7]
        self.state.w = end[8]

    def arc_points(self, start, end, offset1, offset2, clockwise):
        axis1, axis2, axis3, _, _ = self.plane_axes()
        index = self.AXIS_INDEX
        u0 = start[index[axis1]]
        v0 = start[index[axis2]]
        u1 = end[index[axis1]]
        v1 = end[index[axis2]]
        cu, cv, radius = self.validate_center_arc(start, end, offset1, offset2, axis1, axis2)
        start_angle = math.atan2(v0 - cv, u0 - cu)
        end_angle = math.atan2(v1 - cv, u1 - cu)
        # Same XY start and end means a full circle
        same_endpoint = (
            math.isclose(u0, u1, abs_tol=self.arc_tolerance) and
            math.isclose(v0, v1, abs_tol=self.arc_tolerance))
        if same_endpoint:
            sweep = 2.0 * math.pi
        else:
            sweep = self.arc_sweep(start_angle, end_angle, clockwise)
        points = []
        for n in range(self.num_segments + 1):
            t = n / self.num_segments
            if clockwise:
                angle = start_angle - sweep * t
            else:
                angle = start_angle + sweep * t
            u = cu + radius * math.cos(angle)
            v = cv + radius * math.sin(angle)
            point = list(start)
            point[index[axis1]] = u
            point[index[axis2]] = v
            # linearly interpolate every other axis
            for i in range(len(point)):
                if i == index[axis1] or i == index[axis2]:
                    continue
                point[i] = (start[i] + (end[i] - start[i]) * t)
            points.append(tuple(point))
        # Force exact endpoint
        points[-1] = end
        return points

    def arc_points_radius(self, start, end, radius, clockwise):
        axis1, axis2, axis3, _, _ = self.plane_axes()
        index = self.AXIS_INDEX
        u0 = start[index[axis1]]
        v0 = start[index[axis2]]
        u1 = end[index[axis1]]
        v1 = end[index[axis2]]
        radius = float(radius)
        # validate radius
        if abs(radius) <= self.arc_tolerance:
            raise ValueError("Invalid R arc: zero radius")
        # R-format cannot describe a full circle
        if (math.isclose(u0, u1, abs_tol=self.arc_tolerance) and math.isclose(v0, v1, abs_tol=self.arc_tolerance)):
            raise ValueError("Invalid R arc: start and end points are identical")
        # validate chord
        du = u1 - u0
        dv = v1 - v0
        chord = math.hypot(du, dv)
        # Start and end cannot be farther apart than diameter.
        if chord > (2.0 * abs(radius) + self.arc_tolerance):
            raise ValueError("Invalid R arc: chord is longer than diameter")
        # Midpoint of the chord.
        mid_u = (u0 + u1) / 2.0
        mid_v = (v0 + v1) / 2.0
        # Distance from chord midpoint to either possible center.
        half_chord = chord / 2.0
        center_distance = math.sqrt(max(0.0, abs(radius) ** 2 - half_chord ** 2))
        # Unit vector perpendicular to the chord.
        perp_u = -dv / chord
        perp_v = du / chord
        # Two possible centers.
        center1 = ( mid_u + perp_u * center_distance, mid_v + perp_v * center_distance)
        center2 = ( mid_u - perp_u * center_distance, mid_v - perp_v * center_distance)
        # Determine which center gives the required
        # minor/major arc in the requested direction.
        def arc_info(center):
            cu, cv = center
            start_angle = math.atan2(v0 - cv, u0 - cu)
            end_angle = math.atan2(v1 - cv, u1 - cu)
            sweep = self.arc_sweep(start_angle, end_angle, clockwise)
            return start_angle, sweep

        candidate1 = arc_info(center1)
        candidate2 = arc_info(center2)
        if radius > 0.0:
            candidates = [(center1, candidate1), (center2, candidate2)]
            center, (start_angle, sweep) = min(candidates, key=lambda item: item[1][1])
        else:
            candidates = [(center1, candidate1), (center2, candidate2)]
            center, (start_angle, sweep) = max(candidates, key=lambda item: item[1][1])
        cu, cv = center
        points = []
        k = index[axis3]
        for n in range(self.num_segments + 1):
            t = n / self.num_segments
            if clockwise:
                angle = start_angle - sweep * t
            else:
                angle = start_angle + sweep * t
            u = cu + abs(radius) * math.cos(angle)
            v = cv + abs(radius) * math.sin(angle)
            point = list(start)
            point[index[axis1]] = u
            point[index[axis2]] = v
            # linearly interpolate every other axis
            for i in range(len(point)):
                if i == index[axis1] or i == index[axis2]:
                    continue
                point[i] = (start[i] + (end[i] - start[i]) * t)
            points.append(tuple(point))
        points[-1] = end
        return points

    def make_arc(self, axes, params):
        start = (self.state.x, self.state.y, self.state.z,
                 self.state.a, self.state.b, self.state.c,
                 self.state.u, self.state.v, self.state.w)
        end = list(start)
        for letter, value in axes.items():
            index = self.AXIS_INDEX[letter]
            if self.state.absolute:
                end[index] = value
            else:
                end[index] += value
        end = tuple(end)
        _, _, _, offset1_letter, offset2_letter = self.plane_axes()
        if "R" in params and any(letter in params for letter in ("I", "J", "K")):
            raise ValueError(f"Line {self.state.line_number}: R cannot be combined with I/J/K")
        if "R" in params:
            radius = params["R"]
            points = self.arc_points_radius(start, end, radius, clockwise=(self.state.motion == "G2"))
        else:
            offset1 = params.get(offset1_letter, 0.0)
            offset2 = params.get(offset2_letter, 0.0)
            points = self.arc_points(start, end, offset1, offset2, clockwise=(self.state.motion == "G2"))

#        display_points = [self.apply_coordinate_offset(point) for point in points]
        display_points = [tuple(point) for point in points]
        segment = (self.state.line_number, self.line_type[self.state.motion], display_points, self.state.feed, self.state.feed_mode)
        self.segments.append(segment)
        self.state.x = end[0]
        self.state.y = end[1]
        self.state.z = end[2]
        self.state.a = end[3]
        self.state.b = end[4]
        self.state.c = end[5]
        self.state.u = end[6]
        self.state.v = end[7]
        self.state.w = end[8]

## Helper functions ##
    def extract_program(self, lines):
        if lines[0] != "%":
            raise GCodeError(1, "Program does not begin with %", lines[0])
            return
        if lines[-1] != "%":
            raise GCodeError(1, "Program does not end with %", lines[-1])

    def extract_subroutines(self, lines):
        level = 0
        for line_number, line in enumerate(lines, start=1):
            start_match = re.fullmatch(r"O(\d+)\s+SUB", line, re.IGNORECASE)
            end_match = re.fullmatch(r"O(\d+)\s+ENDSUB", line, re.IGNORECASE)
            if start_match:
                level += 1
                number = int(start_match.group(1))
                sub = Subroutine(level=level, number=number, start=line_number+1, end=None)
                self.subroutines[number] = sub
            elif end_match:
                number = int(end_match.group(1))
                try:
                    sub = self.subroutines[number]
                    sub.end = line_number
                except KeyError as e:
                    print(f"ENDSUB {number} at line {line_number} has no matching SUB")
                level -= 1

    def extract_repeats(self, lines):
        level = 0
        for line_number, line in enumerate(lines, start=1):
            start_match = re.match(r"O(\d+)\s+repeat\s+(\[.*\])\s*$", line, re.IGNORECASE)
            end_match = re.match(r"O(\d+)\s+endrepeat", line, re.IGNORECASE)
            if start_match:
                level += 1
                number = int(start_match.group(1))
                if start_match.group(2) is None:
                    raise ValueError(f"Missing Repeat count at line {line_number}")
                expression = start_match.group(2)
                rep = RepeatBlock(level=level, number=number, count=expression, start=line_number+1, end=None)
                self.repeats[number] = rep
                if not (expression.startswith('[') and expression.endswith(']')):
                    raise ValueError(f"Line {line_number}: REPEAT count must be enclosed in []")
            elif end_match:
                number = int(end_match.group(1))
                try:
                    rep = self.repeats[number]
                    rep.end = line_number
                except KeyError as e:
                    print(f"ENDREPEAT {number} at line {line_number} has no matching REPEAT")
                level -= 1

    def extract_whiles(self, lines):
        level = 0
        for line_number, line in enumerate(lines, start=1):
            start_match = re.fullmatch(r"O(\d+)\s+while\s+(\[.*\])\s*", line, re.IGNORECASE)
            end_match = re.fullmatch(r"O(\d+)\s+endwhile", line, re.IGNORECASE)
            if start_match:
                level += 1
                number = int(start_match.group(1))
                if start_match.group(2) is None:
                    raise ValueError(f"Missing condition at line {line_number}")
                condition = start_match.group(2)
                wblk = WhileBlock(level=level, number=number, condition=condition, start=line_number+1, end=None)
                self.whiles[number] = wblk
                if not (condition.startswith('[') and condition.endswith(']')):
                    raise ValueError(f"Line {line_number}: WHILE condition must be enclosed in []")
            elif end_match:
                number = int(end_match.group(1))
                try:
                    wblk = self.whiles[number]
                    wblk.end = line_number
                except KeyError as e:
                    print(f"ENDWHILE {number} at line {line_number} has no matching WHILE")
                level -= 1

    def resolve_parameters(self, line):
        line = re.sub(r"\([^)]*\)", "", line).strip()
        match = re.match(r"\s*#(\d+)\s*=\s(.*)", line)
        if not match:
            raise ValueError(f"Undefined parameter {line}")
        number = int(match.group(1))
        expression = match.group(2)
        self.state.parameters[number] = self.evaluate_expression(expression)

    def evaluate_condition(self, condition):
        condition = condition.strip()
        if not re.match(r"\[(.*?)\]", condition, re.IGNORECASE):
            raise ValueError("Bracket nesting not allowed")
        condition = condition.replace("[", "").replace("]", "")
        match = re.match(r"(.+?)\s+(EQ|NE|LT|LE|GT|GE)\s+(.+)", condition, re.IGNORECASE)
        if match:
            left = self.evaluate_expression(match.group(1).strip())
            right = self.evaluate_expression(match.group(3).strip())
            operator = match.group(2).upper()
            if operator == "EQ":
                return left == right
            elif operator == "NE":
                return left != right
            elif operator == "LT":
                return left < right
            elif operator == "LE":
                return left <= right
            elif operator == "GT":
                return left > right
            elif operator == "GE":
                return left >= right
        return bool(self.evaluate_expression(condition))

    def evaluate_expression(self, expression):
        expression = expression.strip()
        expression = expression.replace("[", "(")
        expression = expression.replace("]", ")")
        for match in re.finditer(r"#(\d+)", expression, re.IGNORECASE):
            number = int(match.group(1))
            if number not in self.state.parameters:
                raise ValueError(f"Undefined parameter #{number}")
            value = self.state.parameters[number]
            expression = expression.replace(match.group(0), str(value))

        operators = {
            "EQ": "==",
            "NE": "!=",
            "LT": "<",
            "LE": "<=",
            "GT": ">",
            "GE": ">=",
            "AND": " and ",
            "OR": " or ",
            "XOR": " ^ ",
        }

        for gcode_op, python_op in operators.items():
            expression = re.sub( rf"\b{gcode_op}\b", python_op, expression, re.IGNORECASE)

        functions = {
            "ABS": abs,
            "SQRT": math.sqrt,
            "SIN": self._sin,
            "COS": self._cos,
            "TAN": self._tan,
            "ASIN": self._asin,
            "ACOS": self._acos,
            "ATAN": self._atan,
            "EXP": math.exp,
            "LN": math.log,
            "ROUND": round,
            "FIX": math.floor,
            "FUP": math.ceil
        }
        eval_locals = {}
        for name, function in functions.items():
            python_name =  f"_gcode_{name.lower()}"
            expression = re.sub(rf"\b{name}\b", python_name, expression, flags=re.IGNORECASE)
            eval_locals[python_name] = function
        try:
            return eval(expression, {"__builtins__": {}}, eval_locals)
        except Exception as e:
            raise ValueError(f"Unable to evaluate expression [{expression}]: {e}")

    def get_parameter(self, number):
        if number not in self.state.parameters:
            raise ValueError(f"Undefined parameter #{number}")
        return self.state.parameters[number]

    def arc_sweep(self, start_angle, end_angle, clockwise):
        if clockwise:
            sweep = start_angle - end_angle
        else:
            sweep = end_angle - start_angle
        if sweep <= 0.0:
            sweep += 2.0 * math.pi
        return sweep

    def plane_axes(self):
        if self.state.plane == "G17":
            return "X", "Y", "Z", "I", "J"
        elif self.state.plane == "G18":
            return "X", "Z", "Y", "I", "K"
        elif self.state.plane == "G19":
            return "Y", "Z", "X", "J", "K"
        raise ValueError(f"Unsupported plane: {self.state.plane}")

    def validate_center_arc(self, start, end, offset1, offset2, axis1, axis2):
        index = {"X": 0, "Y": 1, "Z": 2}
        u0 = start[index[axis1]]
        v0 = start[index[axis2]]
        u1 = end[index[axis1]]
        v1 = end[index[axis2]]
        cu = u0 + offset1
        cv = v0 + offset2
        radius = math.hypot(offset1, offset2)
        if radius <= self.arc_tolerance:
            raise ValueError("Invalid arc: zero radius")
        end_radius = math.hypot(u1 - cu, v1 - cv)
        radius_error = abs(end_radius - radius)
        if radius_error > self.arc_endpoint_tolerance:
            raise ValueError(f"Invalid arc: end point radius mismatch ({end_radius:.6f} != {radius:.6f})")
        return cu, cv, radius

    def reset_state(self):
        self.state = GCodeState()
        self.lines = []
        self.dwell_time = 0.0
        self.segments = []
        self.subroutines.clear()
        self.repeats.clear()
        self.whiles.clear()
        self.tool_list.clear()
        self.call_stack.clear()

    # wrappers for math trig functions that use radians
    def _sin(self, value):
        return math.sin(math.radians(value))

    def _cos(self, value):
        return math.cos(math.radians(value))

    def _tan(self, value):
        return math.tan(math.radians(value))

    def _asin(self, value):
        return math.degrees(math.asin(value))

    def _acos(self, value):
        return math.degrees(math.acos(value))

    def _atan(self, value):
        return math.degrees(math.atan(value))

def main():
    parser = GCodeParser(callback=test, threshold=1000, arc_segments=32)
    parser.set_tool_length(1, 00.0)
    if len(sys.argv) < 2:
        print("No file specified - using test.ngc")
        fname = "test.ngc"
    else:
        fname = sys.argv[1]
    rtn = parser.parse_file(fname)
    if not rtn:
        print(parser.error_message)

def test(segments):
    print(f"Got {len(segments)} segments")
#    for segment in segments:
#        print(segment.points)

if __name__ == "__main__":
    main()
    sys.exit()
