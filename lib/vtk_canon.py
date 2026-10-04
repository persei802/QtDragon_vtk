#!/usr/bin/env python3
# VTKDragon - canon module - translates parser results into vtk line segments
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

import os
import vtk
from .gcode_parser import GCodeParser, GCodeError
# status message alert levels
DEFAULT =  0
WARNING =  1
ERROR = 2


class VTKCanon(object):
    def __init__(self, actor, colors):
        super().__init__()
        self.path_actor = actor
        self.path_colors = colors
        self.feed = 0.0
        self.traverse = 0.0
        self.dwells = 0.0
        self.motion_time = 0.0
        self.tool_list = []
        self.line_to_cells = {}
        self.gcode_properties = None
        self.arc_segments = 64
        self.segment_threshold = 1000
        self.parser = GCodeParser(callback=self.record_motion,
                                  threshold=self.segment_threshold,
                                  arc_segments=self.arc_segments)

    def load_preview(self, filename=None):
        filename = filename or self.last_filename
        if filename is None or not os.path.isfile(filename):
            print(f"Can't load preview, invalid file: {filename}")
            return None
        self.last_filename = filename
        self.traverse = 0
        self.feed = 0
        self.motion_time = 0
        self.path_actor.points.Reset()
        self.path_actor.lines.Reset()
        self.path_actor.colors.Reset()
        self.line_to_cells.clear()
        try:
            rtn = self.parser.parse_file(filename)
        except Exception as e:
            import traceback
            print(f"\n{type(e).__name__}: {e}")
            traceback.print_exc()
            raise
            return e
        if not rtn:
            return self.parser.error_message
        self.tool_list = self.parser.tool_list
        self.dwell_time = self.parser.dwell_time
        return True

    # callback method for gcode_parser
    # segments come in batches of segment_threshold size or less
    def record_motion(self, segments):
        last_point = None
        last_id = None
        for line_number, line_type, points, feed_rate, feed_mode in segments:
            # this section handles multiple segments for 1 line, arcs for example
            for point in points:
                point = point[:3]
                point_id = self.path_actor.points.InsertNextPoint(point)
                if last_point is None:
                    last_point = point
                    last_id = point_id
                    continue
                line = vtk.vtkLine()
                line.GetPointIds().SetId(0, last_id)
                line.GetPointIds().SetId(1, point_id)
                cell_id = self.path_actor.lines.InsertNextCell(line)
                self.line_to_cells.setdefault(line_number, []).append(cell_id)
                self.path_actor.colors.InsertNextTypedTuple(self.path_colors[line_type])
                # update data for gcode properties
                self.update_props(line_type, last_point, point, feed_mode, feed_rate)
                last_point = point
                last_id = point_id
        # create path actor lines but do not render yet
        polydata = self.path_actor.poly_data
        polydata.GetCellData().SetScalars(self.path_actor.colors)
        polydata.Modified()
        self.path_actor.data_mapper.Update()
        self.path_actor.Modified()

    def update_props(self, line_type, start, end, mode, feed):
        if line_type == 'traverse':
            dist = self.calc_dist(start, end)
            self.traverse += dist
        elif line_type == 'feed' or line_type == 'arcfeed':
            if feed == 0.0: return
            dist = self.calc_dist(start, end)
            self.feed += dist
            if mode == 'G93':
                self.motion_time += feed
            else:
                self.motion_time += (dist * 60) / feed

    def calc_dist(self, start, end):
        (x,y,z) = start
        (p,q,r) = end
        return ((x-p)**2 + (y-q)**2 + (z-r)**2) ** 0.5

class TestCanon(object):
    def __init__(self):
        super().__init__()
    
    def record_motion(self, segments):
        print(f"Received {len(segments)} segments")

def main():
    canon = TestCanon()
    parser = GCodeParser(canon.record_motion, threshold=1000, arc_segments=64)
    if len(sys.argv) < 2:
        print("No file specified - using test.ngc")
        fname = "test.ngc"
    else:
        fname = sys.argv[1]
    try:
        parser.parse_file(fname)
    except GCodeError as e:
        print(e)
        sys.exit()

if __name__ == "__main__":
    main()
    sys.exit()
