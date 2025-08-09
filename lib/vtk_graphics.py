#!/usr/bin/env python
import os
import shutil
import gcode
import math
import linuxcnc
import vtk
from PyQt5.QtGui import QColor

# Fix polygons not drawing correctly on some GPU
# https://stackoverflow.com/questions/51357630/vtk-rendering-not-working-as-expected-inside-pyqt?rq=1
import vtk.qt
vtk.qt.QVTKRWIBase = "QGLWidget"
# Fix end

from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from qtvcp.core import Info, Status, Tool
from .base_canon import BaseCanon
from .base_canon import StatCanon
   
INFO = Info()
STATUS = Status()
TOOL = Tool()
MACHINE_UNITS = 'mm' if INFO.MACHINE_IS_METRIC else 'in'
# status message alert levels
DEFAULT =  0
WARNING =  1
ERROR = 2
COLOR_MAP = {
    'traverse':    (76, 128, 128, 85),
    'arcfeed':     (255, 255, 255, 128),
    'feed':        (255, 255, 255, 240),
    'path':        (1.0, 1.0, 0.0),
    'dwell':       (255, 128, 128, 240),
    'limits':      (1.0, 0.0, 0.0),
    'label_ok':    (0.86, 0.64, 0.86),
    'label_limit': (1.00, 0.21, 0.23),
    'user':        (220, 220, 220, 255)}


class VTKCanon(StatCanon):
    def __init__(self, colors=COLOR_MAP):
        super(VTKCanon, self).__init__()
        BaseCanon.__init__(self)
        self.units = MACHINE_UNITS
        self.path_colors = colors
        self.path_actor = PathActor()
        self.path_points = list()
        self.previous_origin = self.origin
        self.ignore_next = False  # hacky way to ignore the second point next to a offset change

    # override function to handle it here
    def rotate_and_translate(self, x, y, z, a, b, c, u, v, w):
        return x, y, z, a, b, c, u, v, w

    def add_path_point(self, line_type, start_point, end_point):
        if self.ignore_next is True:
            self.ignore_next = False
            return

        if self.previous_origin != self.origin:
            self.previous_origin = self.origin
            self.ignore_next = True
            return

        if self.units == 'mm':
            start_point_list = list()
            for point in start_point:
                point *= 25.4
                start_point_list.append(point)

            end_point_list = list()
            for point in end_point:
                point *= 25.4
                end_point_list.append(point)

            line = list()
            line.append(start_point_list)
            line.append(end_point_list)
            self.path_points.append((line_type, line))

        else:
            line = list()
            line.append(start_point)
            line.append(end_point)
            self.path_points.append((line_type, line))

    def draw_lines(self):
        index = 0
        end_point = None
        last_line_type = None
        new_points = vtk.vtkPoints()
        new_colors = vtk.vtkUnsignedCharArray()
        new_colors.SetNumberOfComponents(4)
        new_lines = vtk.vtkCellArray()
        for line in self.path_points:
            line_type = line[0]
            line_data = line[1]
            start_point = line_data[0]
            end_point = line_data[1]
            last_line_type = line_type
            new_points.InsertNextPoint(start_point[:3])
            new_colors.InsertNextTypedTuple(self.path_colors[line_type])
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, index)
            line.GetPointIds().SetId(1, index + 1)
            new_lines.InsertNextCell(line)
            index += 1
        if end_point:
            new_points.InsertNextPoint(end_point[:3])
            new_colors.InsertNextTypedTuple(self.path_colors[last_line_type])
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, index - 1)
            line.GetPointIds().SetId(1, index)
            self.path_actor.lines.InsertNextCell(line)
            # free up memory, lots of it for big files
        self.path_points = list()
        mapper = self.path_actor.data_mapper
        polydata = self.path_actor.poly_data
        polydata.SetPoints(new_points)
        polydata.SetLines(new_lines)
        polydata.GetCellData().SetScalars(new_colors)
        mapper.SetInputData(polydata)
        mapper.Update()
        self.path_actor.SetMapper(mapper)
        self.path_actor.Modified()

    def get_path_actor(self):
        return self.path_actor


class VTKGraphics(QVTKRenderWindowInteractor, StatCanon):
    def __init__(self, parent=None):
        super(VTKGraphics, self).__init__()
        self.parent = parent
        self.canon = VTKCanon()
        self.current_position = None
        self.saved_path = None
        self.saved_extents = None
        self.saved_lines = None
        inifile = INFO.INIPATH
        if inifile is None or not os.path.isfile(inifile):
            raise ValueError("Invalid INI file: %s", inifile)
        self.config_dir = os.path.dirname(inifile)
        temp = INFO.get_error_safe_setting("EMCIO", "RANDOM_TOOLCHANGER")
        self.random = int(temp or 0)
        temp = INFO.TRAJ_COORDINATES
        self.geometry = temp.upper()
        temp = INFO.PARAMETER_FILE
        self.parameter_file = os.path.join(self.config_dir, temp)
        self.temp_parameter_file = os.path.join(self.parameter_file + '.temp')
        self.last_filename = None
        self._current_file = None
        self.gcode_properties = None

        self.view_directions = {
            "x": ((1, 0, 0), (0, 0, 1)),   # Looking down +X
            "y": ((0, -1, 0), (0, 0, 1)),  # Looking down -Y
            "z": ((0, 0, 1), (0, 1, 0)),   # Looking down +Z
            "p": ((1, -1, 1), (0, 0, 1))}  # isometric

        self.g5x_index = STATUS.stat.g5x_index
        self.g5x_offset = STATUS.stat.g5x_offset
        self.g92_offset = STATUS.stat.g92_offset
        self.rotation_offset = STATUS.stat.rotation_xy

        self.spindle_position = (0.0, 0.0, 0.0)
        self.spindle_rotation = (0.0, 0.0, 0.0)
        self.tooltip_position = (0.0, 0.0, 0.0)

        self.tool_no = 0
        self.tool_keys = ['id', 'pocket',
                          'xoffset', 'yoffset', 'zoffset',
                          'aoffset', 'boffset', 'coffset',
                          'uoffset', 'voffset', 'woffset',
                          'diameter', 'frontangle', 'backangle', 'orientation']
        self.units = MACHINE_UNITS
        self.axis = STATUS.stat.axis
        # set up the camera
        self.camera = vtk.vtkCamera()
        self.camera.ParallelProjectionOn()
        if self.units == 'mm':
            self.clipping_range_near = 0.01
            self.clipping_range_far = 10000.0
        else:
            self.clipping_range_near = 0.001
            self.clipping_range_far = 100.0
        self.camera.SetClippingRange(self.clipping_range_near, self.clipping_range_far)
        # set up the renderer
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetActiveCamera(self.camera)
        self.renderer_window = self.GetRenderWindow()
        self.renderer_window.AddRenderer(self.renderer)
        self.interactor = self.renderer_window.GetInteractor()
        self.interactor.SetInteractorStyle(None)
        self.interactor.SetRenderWindow(self.renderer_window)
        # set up machine actor
        self.machine_actor = Machine(self.axis)
        self.machine_actor.SetCamera(self.camera)
        # set up axes actor
        self.axes_actor = Axes()
        transform = vtk.vtkTransform()
        transform.Translate(*self.g5x_offset[:3])
        transform.RotateZ(self.rotation_offset)
        self.axes_actor.SetUserTransform(transform)
        # set up origin actor
        self.origin_actor = Origin()
        # set up path cache
        self.path_cache = PathCache(self.tooltip_position)
        self.path_cache_actor = self.path_cache.get_actor()
        # set up tool actor
        self.tool = Tool(self.get_tool_array())
        self.tool_actor = self.tool.get_actor()
        self.show_extents = bool()
        self.show_dimensions = bool()
        self.show_machine = bool()
        # set up path actor
        self.path_actor = self.canon.get_path_actor()
        # set up extents actor
        self.extents_actor = PathBoundaries(self.path_actor)
        self.extents_actor.SetCamera(self.camera)
        # set up dimensions actor
        self.dimensions_actor = Dimensions()
        # add all the actors to the renderer
        self.renderer.AddActor(self.tool_actor)
        self.renderer.AddActor(self.machine_actor)
        self.renderer.AddActor(self.axes_actor)
        self.renderer.AddActor(self.origin_actor)
        self.renderer.AddActor(self.path_cache_actor)
        self.renderer.AddActor(self.extents_actor)
        self.renderer.AddActor(self.dimensions_actor)
        self.renderer.AddActor(self.path_actor)
        self.renderer.ResetCamera()
        self.interactor.AddObserver("LeftButtonPressEvent", self.button_event)
        self.interactor.AddObserver("LeftButtonReleaseEvent", self.button_event)
        self.interactor.AddObserver("MiddleButtonPressEvent", self.button_event)
        self.interactor.AddObserver("MiddleButtonReleaseEvent", self.button_event)
        self.interactor.AddObserver("RightButtonPressEvent", self.button_event)
        self.interactor.AddObserver("RightButtonReleaseEvent", self.button_event)
        self.interactor.AddObserver("MouseMoveEvent", self.mouse_move)
        self.interactor.AddObserver("MouseWheelForwardEvent", self.mouse_scroll_forward)
        self.interactor.AddObserver("MouseWheelBackwardEvent", self.mouse_scroll_backward)
        self.interactor.Initialize()
        self.renderer_window.Render()
        # background colours
        self._background_color = QColor(60, 60, 60, 255)
        self._background_color2 = QColor(10, 10, 10, 255)
        self.delay = 0
        self._last_filename = str()

        # Add the observers to watch for particular events. These invoke
        # Python functions.
        self.rotating = 0
        self.panning = 0
        self.zooming = 0
        self.pan_mode = True

        # view settings
        self.showDimensions(False)
        self.showProgramBounds(False)
        self.showMachineBounds(True)
        self.setview('p')

    def _hal_init(self):
        STATUS.connect('file-loaded', lambda w, filename: self.load_program(filename))
        STATUS.connect('motion-mode-changed', lambda w, mode: self.motion_type(mode))
        STATUS.connect('user-system-changed', lambda w, data: self.update_g5x_index(data))
        STATUS.connect('periodic', self.periodic_check)
        STATUS.connect('tool-in-spindle-changed', lambda w, tool: self.update_tool(tool))

    # Handle the mouse button events.
    def button_event(self, obj, event):
        if event == "LeftButtonPressEvent":
            if self.pan_mode is True:
                self.panning = 1
            else:
                self.rotating = 1

        elif event == "LeftButtonReleaseEvent":
            if self.pan_mode is True:
                self.panning = 0
            else:
                self.rotating = 0

        elif event == "RightButtonPressEvent":
            if self.pan_mode is True:
                self.rotating = 1
            else:
                self.panning = 1

        elif event == "RightButtonReleaseEvent":
            if self.pan_mode is True:
                self.rotating = 0
            else:
                self.panning = 0

        elif event == "MiddleButtonPressEvent":
            self.zooming = 1
        elif event == "MiddleButtonReleaseEvent":
            self.zooming = 0

    def mouse_scroll_backward(self, obj, event):
        self.zoomout()

    def mouse_scroll_forward(self, obj, event):
        self.zoomin()

    # General high-level logic
    def mouse_move(self, obj, event):
        lastXYpos = self.interactor.GetLastEventPosition()
        lastX = lastXYpos[0]
        lastY = lastXYpos[1]
        xypos = self.interactor.GetEventPosition()
        x = xypos[0]
        y = xypos[1]
        center = self.renderer_window.GetSize()
        centerX = center[0] / 2.0
        centerY = center[1] / 2.0

        if self.rotating:
#            self.rotate(self.renderer, self.camera, x, y, lastX, lastY, centerX, centerY)
            self.rotate(x, y, lastX, lastY, centerX, centerY)
        elif self.panning:
#            self.pan(self.renderer, self.camera, x, y, lastX, lastY, centerX, centerY)
            self.pan(x, y, lastX, lastY, centerX, centerY)
        elif self.zooming:
#            self.dolly(self.renderer, self.camera, x, y, lastX, lastY, centerX, centerY)
            self.dolly(x, y, lastX, lastY, centerX, centerY)

    # Routines that translate the events into camera motions.
    # This one is associated with the left mouse button. It translates x
    # and y relative motions into camera azimuth and elevation commands.
#    def rotate(self, renderer, camera, x, y, lastX, lastY, centerX, centerY):
    def rotate(self, x, y, lastX, lastY, centerX, centerY):
        self.camera.Azimuth(lastX - x)
        self.camera.Elevation(lastY - y)
        self.camera.OrthogonalizeViewUp()
        self.camera.SetClippingRange(self.clipping_range_near, self.clipping_range_far)
        self.renderer_window.Render()
        # self.renderer.ResetCamera()
        self.interactor.ReInitialize()

    # Pan translates x-y motion into translation of the focal point and position.
    def pan(self, x, y, lastX, lastY, centerX, centerY):
        FPoint = self.camera.GetFocalPoint()
        FPoint0 = FPoint[0]
        FPoint1 = FPoint[1]
        FPoint2 = FPoint[2]
        PPoint = self.camera.GetPosition()
        PPoint0 = PPoint[0]
        PPoint1 = PPoint[1]
        PPoint2 = PPoint[2]
        self.renderer.SetWorldPoint(FPoint0, FPoint1, FPoint2, 1.0)
        self.renderer.WorldToDisplay()
        DPoint = self.renderer.GetDisplayPoint()
        focalDepth = DPoint[2]
        APoint0 = centerX + (x - lastX)
        APoint1 = centerY + (y - lastY)
        self.renderer.SetDisplayPoint(APoint0, APoint1, focalDepth)
        self.renderer.DisplayToWorld()
        RPoint = self.renderer.GetWorldPoint()
        RPoint0 = RPoint[0]
        RPoint1 = RPoint[1]
        RPoint2 = RPoint[2]
        RPoint3 = RPoint[3]

        if RPoint3 != 0.0:
            RPoint0 = RPoint0 / RPoint3
            RPoint1 = RPoint1 / RPoint3
            RPoint2 = RPoint2 / RPoint3

        self.camera.SetFocalPoint((FPoint0 - RPoint0) / 1.0 + FPoint0,
                             (FPoint1 - RPoint1) / 1.0 + FPoint1,
                             (FPoint2 - RPoint2) / 1.0 + FPoint2)

        self.camera.SetPosition((FPoint0 - RPoint0) / 1.0 + PPoint0,
                           (FPoint1 - RPoint1) / 1.0 + PPoint1,
                           (FPoint2 - RPoint2) / 1.0 + PPoint2)

        self.renderer_window.Render()

    # Dolly converts y-motion into a camera dolly commands.
#    def dolly(self, renderer, camera, x, y, lastX, lastY, centerX, centerY):
    def dolly(self, x, y, lastX, lastY, centerX, centerY):
        dollyFactor = pow(1.02, (0.5 * (y - lastY)))
        if self.camera.GetParallelProjection():
            parallelScale = self.camera.GetParallelScale() * dollyFactor
            self.camera.SetParallelScale(parallelScale)
        else:
            self.camera.Dolly(dollyFactor)
            self.renderer.ResetCameraClippingRange()
        self.renderer_window.Render()

    def tlo(self, tlo):
        pass

    def load_program(self, fname=None):
        if fname is None: return
        self._current_file = fname
        rtn = self.load_preview(fname)
        if rtn is None: return
        self.canon.draw_lines()
        self.calculate_gcode_properties()
        # move the path to current offsets
        path_position = self.g5x_offset
        path_transform = vtk.vtkTransform()
        path_transform.Translate(*path_position[:3])
        path_transform.RotateWXYZ(*path_position[5:9])
        self.path_actor.SetUserTransform(path_transform)
        self.path_actor.GetProperty().SetOpacity(0.8)
        # update extents and dimensions actors
        self.path_actor = self.canon.get_path_actor()
        self.extents_actor.update(self.path_actor)
        self.dimensions_actor.update_offset(self.g5x_offset[:3])
        self.dimensions_actor.update(self.path_actor, self.machine_actor)
        self.update_render()

    def load_preview(self, filename=None):
        filename = filename or self.last_filename
        if filename is None:
            filename = STATUS.stat.file
        if filename is None or not os.path.isfile(filename):
            self.parent.add_status(f"Can't load preview, invalid file: {filename}", WARNING)
            return None
        self.last_filename = filename
        # create the object which handles the canonical motion callbacks
        # (straight_feed, straight_traverse, arc_feed, rigid_tap, etc.)
        if os.path.exists(self.parameter_file):
            shutil.copy(self.parameter_file, self.temp_parameter_file)
        self.canon.parameter_file = self.temp_parameter_file
        # Some initialization g-code to set the units and optional user code
        unitcode = "G21" if INFO.MACHINE_IS_METRIC else "G20"
        initcode = INFO.get_error_safe_setting("RS274NGC", "RS274NGC_STARTUP_CODE", "")
        load_result = True
        # THIS IS WHERE IT ALL HAPPENS: gcode.parse will execute the code,
        # call back to the canon with motion commands, and record a history
        # of all the movements.
        try:
            result, seq = gcode.parse(filename, self.canon, unitcode, initcode)
            if result > gcode.MIN_ERROR:
                msg = gcode.strerror(result)
                fname = os.path.basename(filename)
                self.report_gcode_error(msg, seq, fname)
        except Exception as e:
            self.report_gcode_error(e)
            self.gcode_properties = None
            load_result = False
        finally:
            os.unlink(self.temp_parameter_file)
            os.unlink(self.temp_parameter_file + '.bak')
        return load_result

    def report_gcode_error(self, msg, seq=None, filename=None):
        if seq is None:
            self.parent.add_status(f"GCode parse error: {msg}", WARNING)
        else:
            self.parent.add_status(f"GCode error in {filename} near line {seq}: {msg}", WARNING)

    def motion_type(self, value):
        if value == linuxcnc.MOTION_TYPE_TOOLCHANGE:
            self.update_tool()

    def update_position(self, position):
        self.spindle_position = position[:3]
        self.spindle_rotation = position[3:6]
        tool_transform = vtk.vtkTransform()
        tool_transform.Translate(*self.spindle_position)
        tool_transform.RotateX(-self.spindle_rotation[0])
        tool_transform.RotateY(-self.spindle_rotation[1])
        tool_transform.RotateZ(-self.spindle_rotation[2])
        self.tool_actor.SetUserTransform(tool_transform)
        tlo = TOOL.GET_TOOL_INFO(self.tool_no)
        self.tooltip_position = [pos - tlo for pos, tlo in zip(self.spindle_position, tlo[2:5])]
        self.path_cache.add_line_point(self.tooltip_position)
        self.update_render()

    def update_g5x_offset(self):
        offset = self.g5x_offset
        transform = vtk.vtkTransform()
        transform.Translate(*offset[:3])
        transform.RotateWXYZ(*offset[5:9])
        self.axes_actor.SetUserTransform(transform)
#        self.path_actor.SetUserTransform(transform)
#        self.interactor.ReInitialize()
        self.update_render()

    def update_g5x_index(self, index):
        self.g5x_index = int(index)

    def update_g92_offset(self):
        if STATUS.is_mdi_mode() or STATUS.is_auto_mode():
            path_offset = list(map(add, self.g92_offset, self.original_g92_offset))
            # determine change in g92 offset since path was drawn
            new_path_position = list(map(add, self.g5x_offset[:9], path_offset))
            transform = vtk.vtkTransform()
            transform.Translate(*new_path_position[:3])
            self.axes_actor.SetUserTransform(transform)
#            self.path_actor.SetUserTransform(transform)
#            self.interactor.ReInitialize()
            self.update_render()

    def update_tool(self, tool):
        self.tool_no = tool
        self.renderer.RemoveActor(self.tool_actor)
        self.tool = Tool(self.get_tool_array())
        self.tool_actor = self.tool.get_actor()
        tool_transform = vtk.vtkTransform()
        tool_transform.Translate(*self.spindle_position)
        tool_transform.RotateX(-self.spindle_rotation[0])
        tool_transform.RotateY(-self.spindle_rotation[1])
        tool_transform.RotateZ(-self.spindle_rotation[2])
        self.tool_actor.SetUserTransform(tool_transform)
        self.renderer.AddActor(self.tool_actor)
        self.update_render()

    def update_render(self):
        self.renderer_window.Render()

    def get_tool_array(self):
        tool_array = {}
        array = TOOL.GET_TOOL_INFO(self.tool_no)
        for i, val in enumerate(self.tool_keys):
            tool_array[val] = array[i]
        return tool_array

    def periodic_check(self, w):
        STATUS.stat.poll()
        position = STATUS.stat.actual_position
        if position != self.current_position:
            self.current_position = position
            self.update_position(position)
        if self.delay < 9:
            self.delay += 1
        else:
            self.delay = 0
            g5x_offset = STATUS.stat.g5x_offset
            g92_offset = STATUS.stat.g92_offset
            if g5x_offset != self.g5x_offset:
                self.g5x_offset = g5x_offset
                self.update_g5x_offset()
            if g92_offset != self.g92_offset:
                self.g92_offset = g92_offset
                self.update_g92_offset()
        return True

    def setview(self, view):
        old_pos = self.camera.GetPosition()
        old_fp = self.camera.GetFocalPoint()
        old_dist = self.calc_dist(old_pos, old_fp)

        bounds = self.extents_actor.GetBounds()
        center = self.g5x_offset[:3]
        
        if view not in self.view_directions: return
        view_dir, view_up = self.view_directions[view]

        self.camera.SetFocalPoint(*center)
        self.camera.SetPosition(
            center[0] + view_dir[0] * old_dist,
            center[1] + view_dir[1] * old_dist,
            center[2] + view_dir[2] * old_dist)
        self.camera.SetViewUp(*view_up)
        self.camera.SetParallelScale(self.camera.GetParallelScale())
        self.renderer.ResetCameraClippingRange()
        self.update_render()
        
    def printView(self):
        fp = self.camera.GetFocalPoint()
        p = self.camera.GetPosition()
        print('position {}'.format(p))
        vu = self.camera.GetViewUp()
        d = self.camera.GetDistance()
        print('distance {}'.format(d))

    def setViewMachine(self):
        self.machine_actor.SetCamera(self.camera)
        self.renderer.ResetCamera()
        self.interactor.ReInitialize()

    def setViewPath(self):
        position = self.g5x_offset
        self.camera.SetViewUp(0, 0, 1)
        self.camera.SetFocalPoint(position[0],
                                  position[1],
                                  position[2])
        self.camera.SetPosition(position[0] + 1000,
                                position[1] - 1000,
                                position[2] + 1000)
        self.camera.Zoom(1.0)
        self.interactor.ReInitialize()

    def clear_live_plotter(self):
        self.renderer.RemoveActor(self.path_cache_actor)
        self.path_cache = PathCache(self.tooltip_position)
        self.path_cache_actor = self.path_cache.get_actor()
        self.renderer.AddActor(self.path_cache_actor)
        self.update_render()

    def enable_panning(self, enabled):
        self.pan_mode = enabled

    def zoomin(self):
        camera = self.camera
        if camera.GetParallelProjection():
            parallelScale = camera.GetParallelScale() * 0.9
            camera.SetParallelScale(parallelScale)
        else:
            self.renderer.ResetCameraClippingRange()
            camera.Zoom(0.9)
        self.update_render()

    def zoomout(self):
        camera = self.camera
        if camera.GetParallelProjection():
            parallelScale = camera.GetParallelScale() * 1.1
            camera.SetParallelScale(parallelScale)
        else:
            self.renderer.ResetCameraClippingRange()
            camera.Zoom(1.1)
        self.update_render()

    def set_alpha_mode(self, alpha):
        val = 0.5 if alpha else 1.0
        self.path_actor.GetProperty().SetOpacity(val)
        self.update_render()

    def showProgramBounds(self, show):
        self.show_extents = show
        if self.extents_actor is None: return
        self.extents_actor.SetXAxisVisibility(show)
        self.extents_actor.SetYAxisVisibility(show)
        self.extents_actor.SetZAxisVisibility(show)
        self.update_render()

    def showDimensions(self, show):
        self.show_dimensions = show
        if self.dimensions_actor is None: return
        self.dimensions_actor.SetVisibility(show)
        self.update_render()

    def showMachineBounds(self, show):
        self.show_machine = show
        self.machine_actor.SetVisibility(show)
        self.update_render()

    def showMachineLabels(self, show):
        self.machine_actor.SetXAxisLabelVisibility(show)
        self.machine_actor.SetYAxisLabelVisibility(show)
        self.machine_actor.SetZAxisLabelVisibility(show)
        self.update_render()

    def setBackgroundColor(self, color):
        self._background_color = color
        self.renderer.SetBackground(color.getRgbF()[:3])
        self.update_render()

    def setBackgroundColor2(self, color2):
        self._background_color2 = color2
        self.renderer.GradientBackgroundOn()
        self.renderer.SetBackground2(color2.getRgbF()[:3])
        self.update_render()

    def calculate_gcode_properties(self):
        props = {}
        loaded_file = self._current_file
        max_speed = float(INFO.MAX_TRAJ_VELOCITY / 60 or 1)
        if not loaded_file:
            props['name'] = "No file loaded"
        else:
            ext = os.path.splitext(loaded_file)[1]
            name = os.path.basename(loaded_file)
            program_filter = INFO.PROGRAM_FILTERS_EXTENSIONS[0][1]
            if '*' + ext in program_filter:
                props['name'] = name
            else:
                props['name'] = f"generated from {name}"

        size = os.stat(loaded_file).st_size
        lines = sum(1 for line in open(loaded_file))
        props['size'] = f"{size} bytes\n{lines} gcode lines"
        if 200 in self.canon.state.gcodes:
            units = "in"
            fmt = '.4f'
        else:
            units = "mm"
            fmt = '.3f'
        g0 = sum(self.calc_dist(l[1][:3], l[2][:3]) for l in self.canon.traverse)
        g1 = (sum(self.calc_dist(l[1][:3], l[2][:3]) for l in self.canon.feed)
            + sum(self.calc_dist(l[1][:3], l[2][:3]) for l in self.canon.arcfeed))
        gt = (sum(self.calc_dist(l[1][:3], l[2][:3])/min(max_speed, l[3]) for l in self.canon.feed) +
              sum(self.calc_dist(l[1][:3], l[2][:3])/min(max_speed, l[3])  for l in self.canon.arcfeed) +
              sum(self.calc_dist(l[1][:3], l[2][:3])/max_speed  for l in self.canon.traverse) +
              self.canon.dwell_time)

        props['toollist'] = self.canon.tool_list
        self.canon.calc_extents()
        if units == 'mm':
            g0 = g0 * 25.4
            g1 = g1 * 25.4
            min_ext = [i * 25.4 for i in self.canon.min_extents]
            max_ext = [i * 25.4 for i in self.canon.max_extents]
            min_rxy = [i * 25.4 for i in self.canon.min_extents_zero_rxy]
            max_rxy = [i * 25.4 for i in self.canon.max_extents_zero_rxy]
        else:
            min_ext = self.canon.min_extents
            max_ext = self.canon.max_extents
            min_rxy = self.canon.min_extents_zero_rxy
            max_rxy = self.canon.max_extents_zero_rxy
            
        props['g0'] = f"{g0:{fmt}} {units}"
        props['g1'] = f"{g1:{fmt}} {units}"
        if gt > 120:
            props['run'] = f"{(gt/60):.1f} Minutes"
        else:
            props['run'] = f"{int(gt)} Seconds"
        for i, axis in enumerate('xyz'):
            props[axis] = f'{min_ext[i]:{fmt}} to {max_ext[i]:{fmt}} = {(max_ext[i] - min_ext[i]):{fmt}} {units}'
            props[axis + '_zero_rxy'] = f'{min_rxy[i]:{fmt}} to {max_rxy[i]:{fmt}} = {(max_rxy[i] - min_rxy[i]):{fmt}} {units}'
        props['machine_unit_sys'] = 'Metric' if self.units == 'mm' else 'Imperial'
        props['gcode_units'] = units
        self.gcode_properties = props
        STATUS.emit('graphics-gcode-properties', self.gcode_properties)

    def calc_dist(self, start, end):
        (x,y,z) = start
        (p,q,r) = end
        return ((x-p)**2 + (y-q)**2 + (z-r)**2) ** .5


class PathActor(vtk.vtkActor):
    def __init__(self):
        super(PathActor, self).__init__()
        self.colors = vtk.vtkUnsignedCharArray()
        self.colors.SetNumberOfComponents(4)
        self.points = vtk.vtkPoints()
        self.lines = vtk.vtkCellArray()
        self.poly_data = vtk.vtkPolyData()
        self.data_mapper = vtk.vtkPolyDataMapper()

# this draws the program boundary outline
class PathBoundaries(vtk.vtkCubeAxesActor):
    def __init__(self, actor):
        self.colors = COLOR_MAP
        bounds = actor.GetBounds()
        self.SetBounds(bounds)
        self.SetFlyModeToStaticEdges()
        self.GetXAxesLinesProperty().SetColor(self.colors['limits'])
        self.GetYAxesLinesProperty().SetColor(self.colors['limits'])
        self.GetZAxesLinesProperty().SetColor(self.colors['limits'])
        self.SetXAxisTickVisibility(0)
        self.SetYAxisTickVisibility(0)
        self.SetZAxisTickVisibility(0)
        self.XAxisMinorTickVisibilityOff()
        self.YAxisMinorTickVisibilityOff()
        self.ZAxisMinorTickVisibilityOff()

        self.SetXAxisLabelVisibility(False)
        self.SetYAxisLabelVisibility(False)
        self.SetZAxisLabelVisibility(False)

    def show_extents(self, show):
        self.SetXAxisVisibility(show)
        self.SetYAxisVisibility(show)
        self.SetZAxisVisibility(show)
        
    def update(self, actor):
        bounds = actor.GetBounds()
        self.SetBounds(bounds)
        self.Modified()

# this draws the dimension lines and texts
class Dimensions(vtk.vtkActor):
    def __init__(self):
        self.colors = COLOR_MAP
        self.g5x_offset = (0.0, 0.0, 0.0)
        # the first 3 items are for the line text colors, the next 6 are for bar text colors
        self.text_colors = [self.colors['label_ok'], self.colors['label_ok'], self.colors['label_ok'],
                            self.colors['label_ok'], self.colors['label_ok'], self.colors['label_ok'],
                            self.colors['label_ok'], self.colors['label_ok'], self.colors['label_ok']]

        self.append_filter = vtk.vtkAppendPolyData()
        self.combined_mapper = vtk.vtkPolyDataMapper()
        self.SetMapper(self.combined_mapper)

    def update(self, path, machine):
        bounds = path.GetBounds()
        limits = machine.GetBounds()
        # check if any program bounds are outside of machine limits
        for i in range(0, 6, 2):
            if bounds[i] < limits[i]:
                self.text_colors[i+3] = self.colors['label_limit']
            if bounds[i+1] > limits[i+1]:
                self.text_colors[i+4] = self.colors['label_limit']
        self.num_pts = []
        self.append_filter.RemoveAllInputs()
        self.draw_dimensions(bounds)
        self.create_line_text(bounds)
        self.create_bar_text(bounds)
        scalars = vtk.vtkIntArray()
        scalars.SetNumberOfComponents(1)
        scalars.SetName("ColorID")
        for idx in range(len(self.num_pts)):
            for i in range(self.num_pts[idx]):
                scalars.InsertNextValue(idx)

        self.append_filter.GetOutput().GetPointData().SetScalars(scalars)
        self.append_filter.Update()

        lut = vtk.vtkLookupTable()
        lut.SetNumberOfTableValues(len(self.text_colors))
        lut.Build()
        for i, color in enumerate(self.text_colors):
            lut.SetTableValue(i, *color, 1)
        mapper = self.GetMapper()
        mapper.SetInputData(self.append_filter.GetOutput())
        mapper.SetScalarRange(0, len(self.text_colors) - 1)
        mapper.SetLookupTable(lut)
        mapper.ScalarVisibilityOn()
        mapper.Update()

    def draw_dimensions(self, bounds):
        xmin, xmax = bounds[0], bounds[1]
        ymin, ymax = bounds[2], bounds[3]
        zmin, zmax = bounds[4], bounds[5]
        self.offset = 8
        # dimension line points
        points = vtk.vtkPoints()
        points.InsertNextPoint(xmin, ymin - self.offset, zmin)
        points.InsertNextPoint(xmax, ymin - self.offset, zmin)
        points.InsertNextPoint(xmin - self.offset, ymin, zmin)
        points.InsertNextPoint(xmin - self.offset, ymax, zmin)
        points.InsertNextPoint(xmin - self.offset, ymin - self.offset, zmin)
        points.InsertNextPoint(xmin - self.offset, ymin - self.offset, zmax)
        # end bar points X axis
        points.InsertNextPoint(xmin, ymin - self.offset + 4, zmin)
        points.InsertNextPoint(xmin, ymin - self.offset - 4, zmin)
        points.InsertNextPoint(xmax, ymin - self.offset + 4, zmin)
        points.InsertNextPoint(xmax, ymin - self.offset - 4, zmin)
        # end bar points Y axis
        points.InsertNextPoint(xmin - self.offset + 4, ymin, zmin)
        points.InsertNextPoint(xmin - self.offset - 4, ymin, zmin)
        points.InsertNextPoint(xmin - self.offset + 4, ymax, zmin)
        points.InsertNextPoint(xmin - self.offset - 4, ymax, zmin)
        # end bar points Z axis        
        points.InsertNextPoint(xmin - self.offset + 4, ymin - self.offset, zmin)
        points.InsertNextPoint(xmin - self.offset - 4, ymin - self.offset, zmin)
        points.InsertNextPoint(xmin - self.offset + 4, ymin - self.offset, zmax)
        points.InsertNextPoint(xmin - self.offset - 4, ymin - self.offset, zmax)

        lines = vtk.vtkCellArray()
        for i in range(9):
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, 2 * i)
            line.GetPointIds().SetId(1, (2 * i) + 1)
            lines.InsertNextCell(line)
            
        line_polydata = vtk.vtkPolyData()
        line_polydata.SetPoints(points)
        line_polydata.SetLines(lines)

        lineMapper = vtk.vtkPolyDataMapper()
        lineMapper.SetInputData(line_polydata)
        lineActor = vtk.vtkActor()
        lineActor.SetMapper(lineMapper)
        lineActor.GetProperty().SetColor(self.colors['label_ok'])
        self.append_filter.AddInputData(line_polydata)
        self.append_filter.Update()

    def create_line_text(self, bounds):
        xmin, xmax = bounds[0], bounds[1]
        ymin, ymax = bounds[2], bounds[3]
        zmin, zmax = bounds[4], bounds[5]
        text = [f'{(xmax - xmin):.3f}', f'{(ymax - ymin):.3f}', f'{(zmax - zmin):.3f}']
        # calculate midpoints of dimension lines
        position = [((xmin + xmax) / 2, ymin - self.offset, zmin),
                    (xmin - self.offset, (ymin + ymax) / 2, zmin),
                    (xmin - self.offset, ymin - self.offset, (zmin + zmax) / 2)]
        for idx in range(3):
            vector_text = vtk.vtkVectorText()
            vector_text.SetText(text[idx])
            vector_text.Update()
            bounds = vector_text.GetOutput().GetBounds()
            center_x = (bounds[0] + bounds[1]) / 2
            center_y = (bounds[2] + bounds[3]) / 2
            width = bounds[1] - bounds[0]
            height = bounds[3] - bounds[2]
            transform = vtk.vtkTransform()
            if idx == 0:
                transform.Translate(position[idx][0] - width, position[idx][1] - self.offset - height, position[idx][2])
            elif idx == 1:
                transform.Translate(position[idx][0] - center_x, position[idx][1] - width, position[idx][2])
                transform.RotateZ(90.0)
            elif idx == 2:
                transform.Translate(position[idx][0] - center_x, position[idx][1] - width, position[idx][2] - width)
                transform.RotateX(90.0)
                transform.RotateZ(90.0)
            transform.Scale(4.0, 4.0, 4.0)
            transform_filter = vtk.vtkTransformPolyDataFilter()
            transform_filter.SetTransform(transform)
            transform_filter.SetInputConnection(vector_text.GetOutputPort())
            transform_filter.Update()
            self.num_pts.append(transform_filter.GetOutput().GetNumberOfPoints())
            self.append_filter.AddInputData(transform_filter.GetOutput())
        self.append_filter.Update()

    def create_bar_text(self, bounds):
        xmin, xmax = bounds[0], bounds[1]
        ymin, ymax = bounds[2], bounds[3]
        zmin, zmax = bounds[4], bounds[5]
        text = (str(round(xmin - self.g5x_offset[0], 3)), str(round(xmax - self.g5x_offset[0], 3)),
                str(round(ymin - self.g5x_offset[1], 3)), str(round(ymax - self.g5x_offset[1], 3)),
                str(round(zmin - self.g5x_offset[2], 3)), str(round(zmax - self.g5x_offset[2], 3)))
        pos = [(xmin, ymin - self.offset - 4, zmin),
               (xmax, ymin - self.offset - 4, zmin),
               (xmin - self.offset - 4, ymin, zmin),
               (xmin - self.offset - 4, ymax, zmin),
               (xmin - self.offset - 4, ymin - self.offset, zmin),
               (xmin - self.offset - 4, ymin - self.offset, zmax)]
        for idx in range(6):
            vector_text = vtk.vtkVectorText()
            vector_text.SetText(text[idx])
            vector_text.Update()
            size = vector_text.GetOutput().GetBounds()
            width = size[1] - size[0]
            height = size[3] - size[2]
            transform = vtk.vtkTransform()
            if idx == 0 or idx == 1:
                transform.Translate(pos[idx][0] + height, pos[idx][1] - 4*width, pos[idx][2])
                transform.RotateZ(90)
            elif idx == 2 or idx == 3:
                transform.Translate(pos[idx][0] - 4*width - 4, pos[idx][1] - height, pos[idx][2])
            elif idx == 4 or idx == 5:
                transform.Translate(pos[idx][0] - 4*width - 4, pos[idx][1] - height, pos[idx][2] - height)
                transform.RotateX(90.0)
            transform.Scale(4.0, 4.0, 4.0)
            transform_filter = vtk.vtkTransformPolyDataFilter()
            transform_filter.SetTransform(transform)
            transform_filter.SetInputConnection(vector_text.GetOutputPort())
            transform_filter.Update()
            self.num_pts.append(transform_filter.GetOutput().GetNumberOfPoints())
            self.append_filter.AddInputData(transform_filter.GetOutput())
        self.append_filter.Update()

    def update_offset(self, offset):
        self.g5x_offset = offset

    def show_dimensions(self, show):
        self.SetVisibility(show)

# the path tracing tool movement
class PathCache:
    def __init__(self, current_position):
        self.colors = COLOR_MAP
        self.current_position = current_position
        self.index = 0
        self.num_points = 2
        self.points = vtk.vtkPoints()
        self.points.InsertNextPoint(current_position)
        self.lines = vtk.vtkCellArray()
        self.lines.InsertNextCell(1)  # number of points
        self.lines.InsertCellPoint(0)
        self.lines_polygon_data = vtk.vtkPolyData()
        self.polygon_mapper = vtk.vtkPolyDataMapper()
        self.actor = vtk.vtkActor()
        self.actor.GetProperty().SetColor(self.colors['path'])
        self.actor.GetProperty().SetLineWidth(2)
        self.actor.GetProperty().SetOpacity(0.6)
        self.actor.SetMapper(self.polygon_mapper)
        self.lines_polygon_data.SetPoints(self.points)
        self.lines_polygon_data.SetLines(self.lines)
        self.polygon_mapper.SetInputData(self.lines_polygon_data)
        self.polygon_mapper.Update()

    def add_line_point(self, point):
        self.index += 1
        self.points.InsertNextPoint(point)
        self.points.Modified()
        self.lines.InsertNextCell(self.num_points)
        self.lines.InsertCellPoint(self.index - 1)
        self.lines.InsertCellPoint(self.index)
        self.lines.Modified()

    def get_actor(self):
        return self.actor


# this draws the machine boundary outline
class Machine(vtk.vtkCubeAxesActor):
    def __init__(self, axis):
        xmax = axis[0]["max_position_limit"]
        xmin = axis[0]["min_position_limit"]
        ymax = axis[1]["max_position_limit"]
        ymin = axis[1]["min_position_limit"]
        zmax = axis[2]["max_position_limit"]
        zmin = axis[2]["min_position_limit"]
        self.SetBounds(xmin, xmax, ymin, ymax, zmin, zmax)
        self.SetFlyModeToStaticEdges()
        self.GetXAxesLinesProperty().SetColor(0.7, 0.0, 0.1)
        self.GetYAxesLinesProperty().SetColor(0.7, 0.0, 0.1)
        self.GetZAxesLinesProperty().SetColor(0.7, 0.0, 0.1)
        self.XAxisTickVisibilityOff()
        self.YAxisTickVisibilityOff()
        self.ZAxisTickVisibilityOff()
        self.XAxisLabelVisibilityOff()
        self.YAxisLabelVisibilityOff()
        self.ZAxisLabelVisibilityOff()
        self.XAxisMinorTickVisibilityOff()
        self.YAxisMinorTickVisibilityOff()
        self.ZAxisMinorTickVisibilityOff()


# this draws the origin symbol at 0,0,0
class Origin(vtk.vtkActor):
    def __init__(self):
        radius = 12 if MACHINE_UNITS == 'mm' else 0.8

        circle_xy = self.make_circle(radius)
        circle_xz = self.make_circle(radius, rotation = (90, 1, 0, 0))
        circle_yz = self.make_circle(radius, rotation = (90, 0, 1, 0))
        
        append = vtk.vtkAppendPolyData()
        append.AddInputData(circle_xy)
        append.AddInputData(circle_xz)
        append.AddInputData(circle_yz)
        append.Update()
        
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(append.GetOutputPort())
        
        self.SetMapper(mapper)
        self.GetProperty().SetColor(0.0, 0.8, 0.8)
        self.GetProperty().SetLineWidth(1)

    def make_circle(self, radius, rotation=None):
        num_sides = 100
        points = vtk.vtkPoints()
        lines = vtk.vtkCellArray()
        
        for i in range(num_sides):
            angle = 2 * math.pi * i / num_sides
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            points.InsertNextPoint(x, y, 0.0)
            
        polyline = vtk.vtkPolyLine()
        polyline.GetPointIds().SetNumberOfIds(num_sides + 1)
        for i in range(num_sides):
            polyline.GetPointIds().SetId(i, i)
        polyline.GetPointIds().SetId(num_sides, 0)
        lines.InsertNextCell(polyline)
        
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetLines(lines)
        
        if rotation:
            transform = vtk.vtkTransform()
            angle, ax, ay, az = rotation
            transform.RotateWXYZ(angle, ax, ay, az)
            transform_filter = vtk.vtkTransformPolyDataFilter()
            transform_filter.SetTransform(transform)
            transform_filter.SetInputData(polydata)
            transform_filter.Update()
            return transform_filter.GetOutput()
        else:
            return polydata
        
# this draws the XYZ axis icon
class Axes(vtk.vtkAxesActor):
    def __init__(self):
        self.units = MACHINE_UNITS
#        self.axis_mask = STATUS.stat.axis_mask
        self.length = 25.0 if self.units == 'mm' else 1.0

        transform = vtk.vtkTransform()
        transform.Translate(0.0, 0.0, 0.0)  # Z up
        self.SetUserTransform(transform)
        self.AxisLabelsOff()
        self.SetShaftTypeToLine()
        self.SetTipTypeToCone()
        self.GetXAxisShaftProperty().SetLineWidth(2)
        self.GetYAxisShaftProperty().SetLineWidth(2)
        self.GetZAxisShaftProperty().SetLineWidth(2)
        self.GetXAxisShaftProperty().SetColor(0, 1, 0)
        self.GetYAxisShaftProperty().SetColor(1, 0, 0)
        self.GetXAxisTipProperty().SetColor(0, 1, 0)
        self.GetYAxisTipProperty().SetColor(1, 0, 0)
        self.SetTotalLength(self.length, self.length, self.length)


class Tool:
    def __init__(self, tool):
        self.units = MACHINE_UNITS
        self.height = 50.0 if self.units == 'mm' else 2.0
        transform = vtk.vtkTransform()
        if tool['id'] == 0 or tool['diameter'] < .05:
            source = vtk.vtkConeSource()
            source.SetHeight(self.height / 2)
            source.SetCenter(-self.height / 4 - tool['zoffset'], -tool['yoffset'], -tool['xoffset'])
            source.SetRadius(self.height / 4)
            source.SetResolution(64)
            transform.RotateWXYZ(90, 0, 1, 0)
        else:
            source = vtk.vtkCylinderSource()
            source.SetHeight(self.height / 2)
            source.SetCenter(-tool['xoffset'], self.height / 4 - tool['zoffset'], tool['yoffset'])
            source.SetRadius(tool['diameter'] / 2)
            source.SetResolution(64)
            transform.RotateWXYZ(90, 1, 0, 0)
        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetTransform(transform)
        transform_filter.SetInputConnection(source.GetOutputPort())
        transform_filter.Update()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(transform_filter.GetOutputPort())
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(mapper)

    def get_actor(self):
        return self.actor
