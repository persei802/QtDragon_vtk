#!/usr/bin/env python
import os
import math
import linuxcnc
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from qtpy.QtGui import QColor
from qtpy.QtCore import Signal, Slot

# Fix polygons not drawing correctly on some GPU
# https://stackoverflow.com/questions/51357630/vtk-rendering-not-working-as-expected-inside-pyqt?rq=1
import vtk.qt
vtk.qt.QVTKRWIBase = "QGLWidget"
# Fix end

from qtvcp.qt_makegui import VCPWindow
from qtvcp.core import Info, Status, Tool
from .vtk_canon import VTKCanon

INFO = Info()
STATUS = Status()
TOOL = Tool()
MACHINE_UNITS = 'mm' if INFO.MACHINE_IS_METRIC else 'in'
# status message alert levels
DEFAULT =  0
WARNING =  1
ERROR = 2
# path colors
COLOR_MAP = {
    'traverse':    (76, 128, 128, 200),
    'arcfeed':     (255, 255, 255, 240),
    'feed':        (255, 255, 255, 240),
    'path':        (1.0, 1.0, 0.0),
    'dwell':       (255, 128, 128, 240),
    'limits':      (1.0, 0.0, 0.0),
    'label_ok':    (0.86, 0.64, 0.86),
    'label_limit': (1.00, 0.21, 0.23),
    'highlight':   (0, 1.0, 1.0),
    'user':        (220, 220, 220, 255)}


class VTKGraphics(QVTKRenderWindowInteractor):
    g5x_updated = Signal()
    g92_updated = Signal()
    rot_updated = Signal()

    def __init__(self, parent=None):
        super(VTKGraphics, self).__init__()
        self.parent = VCPWindow()
        self.current_position = None
        self.last_filename = None
        self.current_file = None
        self.gcode_properties = None
        self.units = MACHINE_UNITS
        self.inhibit_selection = True
        self.path_colors = COLOR_MAP
        self.path_actor = None
        self.highlight_actor = None
        self.max_speed = float(INFO.MAX_TRAJ_VELOCITY / 60 or 1)
        self.min_extents = [0.0, 0.0, 0.0]
        self.max_extents = [0.0, 0.0, 0.0]
        self.min_extents_zero_xy = [0.0, 0.0, 0.0]
        self.max_extents_zero_xy = [0.0, 0.0, 0.0]

        self.view_directions = {
            "x": ((1, 0, 0), (0, 0, 1), 1000),   # right
            "y": ((0, -1, 0), (0, 0, 1), 1000),  # front
            "z": ((0, 0, 1), (0, 1, 0), 1000),   # top
            "p": ((1, -1, 1), (0, 0, 1), 1000)}  # isometric

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
        self.axis = STATUS.stat.axis
        # background colours
        self._background_color = QColor(60, 60, 60, 255)
        self._background_color2 = QColor(10, 10, 10, 255)
        self.delay = 0
        self._last_filename = str()

        self.rotating = 0
        self.panning = 0
        self.zooming = 0
        self.pan_mode = True

    def init_vtkmodules(self):
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
        self.render_window = self.GetRenderWindow()
        self.render_window.AddRenderer(self.renderer)
        self.interactor = self.render_window.GetInteractor()
        self.interactor.SetInteractorStyle(None)
        self.interactor.SetRenderWindow(self.render_window)
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
        self.origin_actor = Axes()
        # set up path cache
        self.path_cache = PathCache(self.tooltip_position)
        self.path_cache_actor = self.path_cache.get_actor()
        # set up tool actor
        self.tool = Tool(self.get_tool_array())
        self.tool_actor = self.tool.get_actor()
        # set up path actors
        self.path_actor = PathActor()
        self.canon = VTKCanon(self.path_actor, self.path_colors)
        self.highlight_actor = PathActor()
        self.highlight_actor.GetProperty().SetColor(self.path_colors["highlight"])
        self.highlight_actor.GetProperty().SetLineWidth(2)
        # set up extents actor
        self.extents_actor = PathBoundaries(self.path_actor)
        self.extents_actor.SetCamera(self.camera)
        # set up dimensions actor
        self.dimensions_actor = Dimensions(self.machine_actor)
        # add all the actors to the renderer
        self.renderer.AddActor(self.machine_actor)
        self.renderer.AddActor(self.tool_actor)
        self.renderer.AddActor(self.axes_actor)
        self.renderer.AddActor(self.origin_actor)
        self.renderer.AddActor(self.path_cache_actor)
        self.renderer.AddActor(self.extents_actor)
        self.renderer.AddActor(self.dimensions_actor)
        self.renderer.AddActor(self.path_actor)
        self.renderer.AddActor(self.highlight_actor)
        self.renderer.ResetCamera()
        # Add observers to watch for particular events.
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
        self.render_window.Render()

    def _hal_init(self):
        STATUS.connect('file-loaded', lambda w, filename: self.load_program(filename))
        STATUS.connect('motion-mode-changed', lambda w, mode: self.motion_type(mode))
        STATUS.connect('user-system-changed', lambda w, data: self.update_g5x_index(data))
        STATUS.connect('periodic', self.periodic_check)
        STATUS.connect('tool-in-spindle-changed', lambda w, tool: self.update_tool(tool))
        STATUS.connect('gcode-line-selected', lambda w, line: self.highlight_line(line))
        self.g5x_updated.connect(self.update_g5x_offset)
        self.g92_updated.connect(self.update_g92_offset)
        self.rot_updated.connect(self.update_rotation)

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
        lastX, lastY = self.interactor.GetLastEventPosition()
        x, y = self.interactor.GetEventPosition()
        dx = lastX - x
        dy = lastY - y
        center = self.render_window.GetSize()
        centerX = center[0] / 2.0
        centerY = center[1] / 2.0

        if self.rotating:
            self.rotate(dx, dy)
        elif self.panning:
            self.pan(dx, dy, centerX, centerY)
        elif self.zooming:
            self.dolly(x, y, lastX, lastY)

    # Routines that translate the events into camera motions.
    def rotate(self, dx, dy):
        self.camera.Azimuth(dx * 0.5)
        self.camera.Elevation(dy * 0.5)
        self.camera.OrthogonalizeViewUp()
        self.renderer.ResetCameraClippingRange()
        self.render_window.Render()

    # Pan translates x-y motion into translation of the focal point and position.
    def pan(self, dx, dy, centerX, centerY):
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
        APoint0 = centerX - dx
        APoint1 = centerY - dy
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

        self.render_window.Render()

    # Dolly converts y-motion into a camera dolly commands.
    def dolly(self, x, y, lastX, lastY):
        dollyFactor = pow(1.02, (0.5 * (y - lastY)))
        if self.camera.GetParallelProjection():
            parallelScale = self.camera.GetParallelScale() * dollyFactor
            self.camera.SetParallelScale(parallelScale)
        else:
            self.camera.Dolly(dollyFactor)
            self.renderer.ResetCameraClippingRange()
        self.render_window.Render()

## STATUS messages
    def load_program(self, fname=None):
        if fname is None: return
        self.current_file = fname
        self.highlight_actor.points.Reset()
        self.highlight_actor.lines.Reset()
        rtn = self.canon.load_preview(fname)
        if rtn is None: return
        if type(rtn) is str:
            self.parent.add_status(rtn, WARNING)
            return
        # move the path to current offsets
        transform = vtk.vtkTransform()
        transform.Translate(*self.g5x_offset[:3])
        self.path_actor.SetUserTransform(transform)
        self.path_actor.Modified()
        self.calc_extents(False)
        transform.RotateWXYZ(*self.g5x_offset[5:9])
        transform.RotateZ(self.rotation_offset)
        self.path_actor.SetUserTransform(transform)
        self.path_actor.Modified()
        self.calc_extents(True)
        self.highlight_actor.SetUserTransform(transform)
        # update extents and dimensions actors
        self.extents_actor.update(self.path_actor)
        self.dimensions_actor.update(self.g5x_offset[:3], self.extents_actor)
        self.render_window.Render()
        self.calc_gcode_properties()

    def motion_type(self, value):
        if value == linuxcnc.MOTION_TYPE_TOOLCHANGE:
            self.update_tool()

    def update_g5x_index(self, index):
        self.g5x_index = int(index)

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
        self.render_window.Render()

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
            rotation_offset = STATUS.stat.rotation_xy
            if g5x_offset != self.g5x_offset:
                self.g5x_offset = g5x_offset
                self.g5x_updated.emit()
            if g92_offset != self.g92_offset:
                self.g92_offset = g92_offset
                self.g92_updated.emit()
            if rotation_offset != self.rotation_offset:
                self.rotation_offset = rotation_offset
                self.rot_updated.emit()
        return True

    def highlight_line(self, selected_line):
        if self.inhibit_selection: return
        cell_ids = self.canon.line_to_cells.get(selected_line, [])
        if not cell_ids:
            self.highlight_actor.poly_data.Initialize()
            self.highlight_actor.poly_data.Modified()
            self.render_window.Render()
            return
        ids = vtk.vtkIdList()
        for cell_id in cell_ids:
            ids.InsertNextId(cell_id)
        extract = vtk.vtkExtractCells()
        extract.SetInputData(self.path_actor.poly_data)
        extract.SetCellList(ids)
        extract.Update()
        geometry = vtk.vtkGeometryFilter()
        geometry.SetInputData(extract.GetOutput())
        geometry.Update()
        output = geometry.GetOutput()
        self.highlight_actor.poly_data.SetPoints(output.GetPoints())
        self.highlight_actor.poly_data.SetLines(output.GetLines())
        self.highlight_actor.poly_data.Modified()
        self.highlight_actor.data_mapper.Update()
        self.render_window.Render()

    def calc_extents(self, rotated=False):
        bounds = self.path_actor.GetBounds()
        if rotated:
            self.min_extents_rxy = (bounds[0], bounds[2], bounds[4])
            self.max_extents_rxy = (bounds[1], bounds[3], bounds[5])
        else:
            self.min_extents = (bounds[0], bounds[2], bounds[4])
            self.max_extents = (bounds[1], bounds[3], bounds[5])

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
        self.render_window.Render()

    def update_g5x_offset(self):
        offset = self.g5x_offset
        transform = vtk.vtkTransform()
        transform.Translate(*offset[:3])
        transform.RotateWXYZ(*offset[5:9])
        self.axes_actor.SetUserTransform(transform)
        self.path_actor.SetUserTransform(transform)
        self.highlight_actor.SetUserTransform(transform)
        self.extents_actor.update(self.path_actor)
        self.dimensions_actor.update(self.g5x_offset[:3], self.extents_actor)
        self.render_window.Render()

    def update_g92_offset(self):
#        if STATUS.is_mdi_mode() or STATUS.is_auto_mode():
        new_path_position = [self.g5x_offset[i] + self.g92_offset[i] for i in range(len(self.g5x_offset))]
        transform = vtk.vtkTransform()
        transform.Translate(*new_path_position[:3])
        self.axes_actor.SetUserTransform(transform)
        self.render_window.Render()

    def update_rotation(self):
        transform = vtk.vtkTransform()
        transform.Translate(*self.g5x_offset[:3])
        transform.RotateZ(self.rotation_offset)
        self.axes_actor.SetUserTransform(transform)
        self.path_actor.SetUserTransform(transform)
        self.highlight_actor.SetUserTransform(transform)
        self.extents_actor.update(self.path_actor)
        self.dimensions_actor.update(self.g5x_offset[:3], self.extents_actor)
        self.render_window.Render()

    def get_tool_array(self):
        tool_array = {}
        array = TOOL.GET_TOOL_INFO(self.tool_no)
        for i, val in enumerate(self.tool_keys):
            tool_array[val] = array[i]
        return tool_array

    def setview(self, view):
        if view not in self.view_directions: return
        center = self.g5x_offset[:3]
        view_dir, view_up, dist = self.view_directions[view]

        self.camera.SetFocalPoint(*center)
        self.camera.SetPosition(
            center[0] + view_dir[0] * dist,
            center[1] + view_dir[1] * dist,
            center[2] + view_dir[2] * dist)
        self.camera.SetViewUp(*view_up)
        self.renderer.ResetCameraClippingRange()
        self.render_window.Render()
        self.printView()

    def printView(self):
        fp = self.camera.GetFocalPoint()
        p = self.camera.GetPosition()
        vu = self.camera.GetViewUp()
        d = self.camera.GetDistance()

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
        self.render_window.Render()

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
        self.render_window.Render()

    def zoomout(self):
        camera = self.camera
        if camera.GetParallelProjection():
            parallelScale = camera.GetParallelScale() * 1.1
            camera.SetParallelScale(parallelScale)
        else:
            self.renderer.ResetCameraClippingRange()
            camera.Zoom(1.1)
        self.render_window.Render()

    def set_alpha_mode(self, alpha):
        val = 0.5 if alpha else 1.0
        self.path_actor.GetProperty().SetOpacity(val)
        self.render_window.Render()

    def showProgramBounds(self, show):
        if self.extents_actor is None: return
        self.extents_actor.SetXAxisVisibility(show)
        self.extents_actor.SetYAxisVisibility(show)
        self.extents_actor.SetZAxisVisibility(show)
        self.render_window.Render()

    def showDimensions(self, show):
        if self.dimensions_actor is None: return
        self.dimensions_actor.SetVisibility(show)
        self.render_window.Render()

    def showMachineBounds(self, show):
        self.machine_actor.SetVisibility(show)
        self.render_window.Render()

    def showMachineLabels(self, show):
        self.machine_actor.SetXAxisLabelVisibility(show)
        self.machine_actor.SetYAxisLabelVisibility(show)
        self.machine_actor.SetZAxisLabelVisibility(show)
        self.render_window.Render()

    def setBackgroundColor(self, color):
        self._background_color = color
        self.renderer.SetBackground(color.getRgbF()[:3])
        self.render_window.Render()

    def set_inhibit_selection(self, state):
        self.inhibit_selection = state

    def setBackgroundColor2(self, color):
        self._background_color2 = color
        self.renderer.GradientBackgroundOn()
        self.renderer.SetBackground2(color.getRgbF()[:3])
        self.render_window.Render()

    def calc_gcode_properties(self):
        props = {}
        loaded_file = self.current_file
        max_speed = self.max_speed
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
        if self.units == "mm":
            units = "mm"
            fmt = '.3f'
        else:
            units = "in"
            fmt = '.4f'
        g0 = self.canon.traverse
        g1 = self.canon.feed
        gt = (g0 / max_speed) + self.canon.motion_time + self.canon.dwell_time
        gt = round(gt)
        props['toollist'] = self.canon.tool_list
        if units == 'in':
            g0 = g0 / 25.4
            g1 = g1 / 25.4
            min_ext = [i / 25.4 for i in self.min_extents]
            max_ext = [i / 25.4 for i in self.max_extents]
            min_rxy = [i / 25.4 for i in self.min_extents_rxy]
            max_rxy = [i / 25.4 for i in self.max_extents_rxy]
        else:
            min_ext = self.min_extents
            max_ext = self.max_extents
            min_rxy = self.min_extents_rxy
            max_rxy = self.max_extents_rxy
            
        props['g0'] = f"{g0:{fmt}} {units}"
        props['g1'] = f"{g1:{fmt}} {units}"
        if gt > 120:
            props['run'] = f"{(gt/60):.1f} Minutes"
        else:
            props['run'] = f"{int(gt)} Seconds"
        for i, axis in enumerate('xyz'):
            props[axis] = f'{min_ext[i]:{fmt}} to {max_ext[i]:{fmt}} = {(max_ext[i] - min_ext[i]):{fmt}} {units}'
            props[axis + '_rxy'] = f'{min_rxy[i]:{fmt}} to {max_rxy[i]:{fmt}} = {(max_rxy[i] - min_rxy[i]):{fmt}} {units}'
        props['machine_unit_sys'] = 'Metric' if self.units == 'mm' else 'Imperial'
        props['gcode_units'] = units
        self.gcode_properties = props
        STATUS.emit('graphics-gcode-properties', self.gcode_properties)

# an actor for the toolpath preview
class PathActor(vtk.vtkActor):
    def __init__(self):
        super(PathActor, self).__init__()
        self.colors = vtk.vtkUnsignedCharArray()
        self.colors.SetNumberOfComponents(4)
        self.points = vtk.vtkPoints()
        self.lines = vtk.vtkCellArray()
        self.poly_data = vtk.vtkPolyData()
        self.data_mapper = vtk.vtkPolyDataMapper()

        self.poly_data.SetPoints(self.points)
        self.poly_data.SetLines(self.lines)
        self.data_mapper.SetInputData(self.poly_data)
        self.SetMapper(self.data_mapper)

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
    def __init__(self, machine):
        super(Dimensions, self).__init__()
        self.colors = COLOR_MAP
        self.limits = machine.GetBounds()
        self.g5x_offset = (0.0, 0.0, 0.0)
        # the first 3 items are for the line text colors, the next 6 are for bar text colors
        self.text_colors = [self.colors['label_ok'], self.colors['label_ok'], self.colors['label_ok'],
                            self.colors['label_ok'], self.colors['label_ok'], self.colors['label_ok'],
                            self.colors['label_ok'], self.colors['label_ok'], self.colors['label_ok']]

        self.append_filter = vtk.vtkAppendPolyData()
        self.combined_mapper = vtk.vtkPolyDataMapper()
        self.SetMapper(self.combined_mapper)

    def update(self, offset, path):
        self.g5x_offset = offset
        bounds = path.GetBounds()
        # check if any program bounds are outside of machine limits
        for i in range(0, 6, 2):
            if bounds[i] < self.limits[i]:
                self.text_colors[i+3] = self.colors['label_limit']
            else:
                self.text_colors[i+3] = self.colors['label_ok']
            if bounds[i+1] > self.limits[i+1]:
                self.text_colors[i+4] = self.colors['label_limit']
            else:
                self.text_colors[i+4] = self.colors['label_ok']
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


# this draws the XYZ axis icon
class Axes(vtk.vtkAxesActor):
    def __init__(self):
        self.units = MACHINE_UNITS
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
