"""Qt view only. ROS hands immutable display dictionaries and copied QImages to slots."""
import math
from collections import deque
import time
from pathlib import Path
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QPolygonF, QImage, QFontDatabase
from PyQt5.QtWidgets import (QWidget, QGridLayout, QVBoxLayout, QHBoxLayout, QLabel,
                             QPushButton, QCheckBox, QSlider, QListWidget, QFileDialog)
from .model import DisplayModel, STATES, MODES

COLOURS = {'observed': '#5ade9b', 'memory': '#41dce6', 'planned': '#6499ff', 'candidates': '#f4cf63'}
HISTORY_COLOURS = {'detected': '#2f8a5c', 'planned': '#3d5fa8'}


class MapPanel(QWidget):
    def __init__(self, model):
        super().__init__()
        self.model = model
        self.scale = 350.
        self.follow = True
        self.show_trail = True
        self.show_history = True
        self.layer_counts = {}
        self.setMinimumSize(300, 250)

    def wheelEvent(self, event):
        self.scale = max(30, min(3000, self.scale * (1.15 if event.angleDelta().y() > 0 else 1/1.15)))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#111d2d'))
        pose = self.model.current.get('pose') if self.model.current else None
        cx, cy = pose[:2] if self.follow and pose else (0., 0.)
        def point(x, y):
            return QPointF(self.width()/2 + (x-cx)*self.scale, self.height()/2 - (y-cy)*self.scale)
        p.setPen(QPen(QColor('#22334a'), 1))
        extent = max(self.width(), self.height())/self.scale
        step = .1 if self.scale > 150 else .5
        for n in range(math.floor((cx-extent)/step), math.ceil((cx+extent)/step)+1):
            x = point(n*step, cy).x()
            p.drawLine(QPointF(x, 0), QPointF(x, self.height()))
        for n in range(math.floor((cy-extent)/step), math.ceil((cy+extent)/step)+1):
            y = point(cx, n*step).y()
            p.drawLine(QPointF(0, y), QPointF(self.width(), y))
        p.setPen(QPen(QColor('#415572'), 1))
        p.drawLine(point(-100, 0), point(100, 0))
        p.drawLine(point(0, -100), point(0, 100))
        if self.show_history:
            for name, colour in HISTORY_COLOURS.items():
                points = self.model.history_points(name)
                if points:
                    p.setPen(QPen(QColor(colour), 3 if name == 'detected' else 2, Qt.SolidLine, Qt.RoundCap))
                    p.drawPoints(QPolygonF([point(*xy) for xy in points]))
        if self.show_trail and len(self.model.trail) > 1:
            p.setPen(QPen(QColor('#71839b'), 1, Qt.DotLine))
            p.drawPolyline(QPolygonF([point(*xy) for xy in self.model.trail]))
        self.layer_counts = {}
        for name, colour in COLOURS.items():
            lines = self.model.layers.get(name, [])
            self.layer_counts[name] = len(lines)
            p.setPen(QPen(QColor(colour), 3 if name != 'planned' else 2,
                          Qt.DashLine if name == 'candidates' else Qt.SolidLine))
            if name == 'candidates' and self.model.current and self.model.current.get('candidate_quality'):
                continue
            for line in lines:
                if len(line) > 1:
                    p.drawPolyline(QPolygonF([point(*xy) for xy in line]))
                elif line:
                    p.drawEllipse(point(*line[0]), 4, 4)
        qualities = self.model.current.get('candidate_quality', []) if self.model.current else []
        finite = [q['rule_score'] for q in qualities if math.isfinite(q['rule_score'])]
        low, high = (min(finite), max(finite)) if finite else (0., 1.)
        for quality in qualities:
            score = quality['rule_score']
            relative = (score-low)/max(.001, high-low) if math.isfinite(score) else 0.
            colour = QColor(COLOURS['candidates'])
            colour.setAlphaF(.4+.6*max(0., min(1., relative)))
            p.setPen(QPen(colour, 4 if quality['selected'] else 1.5+relative, Qt.DashLine))
            for line in quality['lines']:
                if len(line)>1:
                    p.drawPolyline(QPolygonF([point(*xy) for xy in line]))
                if line:
                    mlp = quality['mlp_score']
                    label = f"#{quality['id']} R:{score:.2f} M:{mlp:.2f}" if math.isfinite(mlp) else f"#{quality['id']} R:{score:.2f} M:n/a"
                    if quality['selected']:
                        label += ' SELECTED'
                    p.drawText(point(*line[-1])+QPointF(8,-8), label)
        if pose:
            x, y, yaw = pose
            shape = []
            for dx, dy in [(.065, 0), (-.045, .035), (-.025, 0), (-.045, -.035)]:
                shape.append(point(x + dx*math.cos(yaw)-dy*math.sin(yaw),
                                   y + dx*math.sin(yaw)+dy*math.cos(yaw)))
            p.setPen(QPen(QColor('#e1eaff'), 2))
            p.setBrush(QColor('#8b9dff'))
            p.drawPolygon(QPolygonF(shape))
        p.setPen(QColor('#c6d4e8'))
        p.drawText(20, 30, 'LOCAL MAP  /  odom')
        p.drawText(20, self.height()-18, f'Wheel to zoom  •  grid {step:.1f} m  •  {"FOLLOW" if self.follow else "ORIGIN"}')
        for i, (name, colour) in enumerate(COLOURS.items()):
            p.setPen(QPen(QColor(colour), 3))
            p.drawLine(25+i*145, 55, 45+i*145, 55)
            p.drawText(52+i*145, 60, name.title())
        if not pose:
            p.drawText(self.rect(), Qt.AlignCenter, 'Waiting for rover snapshot…')


class CameraPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.image = QImage()
        self.setMinimumSize(200, 150)

    def paintEvent(self, event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor('#0c1421'))
        if self.image.isNull():
            p.setPen(QColor('#8798b1'))
            p.drawText(self.rect(), Qt.AlignCenter, 'CAMERA  /  waiting for RGB')
        else:
            size = self.image.size().scaled(self.size(), Qt.KeepAspectRatio)
            rect = QRectF((self.width()-size.width())/2, (self.height()-size.height())/2,
                          size.width(), size.height())
            p.drawImage(rect, self.image)


class EventSlider(QSlider):
    def __init__(self, model):
        super().__init__(Qt.Horizontal)
        self.model = model
        self.extent = None
        self.setRange(0, 1000)
        self.setValue(1000)
        self.setEnabled(False)

    def paintEvent(self, event):
        super().paintEvent(event)
        if len(self.model.events) < 2:
            return
        p = QPainter(self)
        lo, hi = self.extent or (self.model.events[0]['time'], self.model.events[-1]['time'])
        for event in self.model.events:
            x = 10 + (self.width()-20)*(event['time']-lo)/max(1, hi-lo)
            p.setPen(QPen(QColor('#f4cf63' if event['kind'] == 'safety' else '#6499ff'), 2))
            p.drawLine(QPointF(x, 0), QPointF(x, self.height()/2))


class GcsWindow(QWidget):
    command_requested = pyqtSignal(str)
    bag_requested = pyqtSignal(str)
    seek_requested = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        QFontDatabase.addApplicationFont(str(Path(__file__).parent / 'fonts/NotoSansCJK-UI.otf'))
        self.setWindowTitle('Line Follower · Development Ground Station')
        self.resize(1600, 1000)
        self.model = DisplayModel()
        self.replay_mode = False
        self.replay_duration = 0.
        self._live_snapshot = self._live_image = None
        self._archive_events = []
        self.snapshot_count = self.camera_count = 0
        self.first_snapshot = self.first_camera = None
        self.setStyleSheet('''
            QWidget { background: #0c1421; color: #dce7f7; font-family: "Noto Sans CJK KR", sans-serif; font-size: 13px; }
            QLabel { background: transparent; }
            QPushButton { background: #233650; border: 1px solid #395170; border-radius: 6px; padding: 11px; }
            QPushButton:hover { background: #304c72; }
            QPushButton:disabled { color: #58687e; border-color: #253247; }
            QListWidget { background: #111d2d; border: 0; padding: 5px; }
            QCheckBox { padding: 5px; }
        ''')
        self.grid = QGridLayout(self)
        self.grid.setContentsMargins(16, 16, 16, 16)
        self.grid.setSpacing(12)
        for i in range(4):
            self.grid.setColumnStretch(i, 1)
            self.grid.setRowStretch(i, 1)
        self.map_panel = MapPanel(self.model)
        self.command_panel = QWidget()
        panel = QVBoxLayout(self.command_panel)
        title = QLabel('ROVER CONTROL')
        title.setStyleSheet('font-size: 21px; font-weight: bold; color: #a6bcff;')
        panel.addWidget(title)
        self.state_label = QLabel('WAITING FOR ROVER')
        self.state_label.setWordWrap(True)
        panel.addWidget(self.state_label)
        self.link_label = QLabel('LINK  LOST  /  no snapshots')
        panel.addWidget(self.link_label)
        self.recording_label = QLabel('BAG / IDLE')
        self.recording_label.setWordWrap(True)
        panel.addWidget(self.recording_label)
        self.buttons = {}
        for command, text in [('start', '미션 시작  ·  START'), ('bag_start', 'ROS Bag 시작'),
                              ('bag_stop', 'ROS Bag 종료'), ('kill', 'KILL  ·  소프트웨어 정지')]:
            button = QPushButton(text)
            button.clicked.connect(lambda checked, c=command: self.command_requested.emit(c))
            if command == 'kill':
                button.setStyleSheet('background: #773044; border: 1px solid #e1667e; font-weight: bold;')
            panel.addWidget(button)
            self.buttons[command] = button
        note = QLabel('KILL requires a working link.\nSoftware stop only — not a physical E-STOP.')
        note.setStyleSheet('color: #98a8bd; font-size: 11px;')
        panel.addWidget(note)
        self.command_label = QLabel('No commands sent')
        self.command_label.setWordWrap(True)
        panel.addWidget(self.command_label)
        follow = QCheckBox('Follow robot')
        follow.setChecked(True)
        follow.toggled.connect(self.set_follow)
        panel.addWidget(follow)
        trail = QCheckBox('Analysis trail (display only)')
        trail.setChecked(True)
        trail.toggled.connect(self.set_trail)
        panel.addWidget(trail)
        history = QCheckBox('Confirmed map: passed path only (display only)')
        history.setChecked(True)
        history.toggled.connect(self.set_history)
        panel.addWidget(history)
        clear = QPushButton('Clear confirmed map')
        clear.clicked.connect(self.clear_history)
        panel.addWidget(clear)
        panel.addStretch()
        self.camera_panel = CameraPanel()
        self.timeline_panel = QWidget()
        timeline = QVBoxLayout(self.timeline_panel)
        modes = QHBoxLayout()
        modes.addWidget(QLabel('ROS BAG  /  EVENT TIMELINE'))
        modes.addStretch()
        self.live_button = QPushButton('● LIVE')
        self.replay_button = QPushButton('OPEN BAG / REPLAY')
        self.replay_button.clicked.connect(self.choose_bag)
        self.live_button.clicked.connect(self.resume_live)
        modes.addWidget(self.live_button)
        modes.addWidget(self.replay_button)
        timeline.addLayout(modes)
        self.time_label = QLabel('LIVE')
        modes.insertWidget(1, self.time_label)
        self.slider = EventSlider(self.model)
        self.slider.valueChanged.connect(self.seek)
        timeline.addWidget(self.slider)
        self.event_list = QListWidget()
        self.event_list.itemClicked.connect(self.seek_event)
        timeline.addWidget(self.event_list)
        self.grid.addWidget(self.map_panel, 0, 0, 3, 3)
        self.grid.addWidget(self.command_panel, 0, 3, 2, 1)
        self.grid.addWidget(self.camera_panel, 2, 3, 1, 1)
        self.grid.addWidget(self.timeline_panel, 3, 0, 1, 4)

    def set_follow(self, value):
        self.map_panel.follow = value
        self.map_panel.update()

    def set_trail(self, value):
        self.map_panel.show_trail = value
        self.map_panel.update()

    def set_history(self, value):
        self.map_panel.show_history = value
        self.map_panel.update()

    def clear_history(self):
        self.model.clear_history()
        self.map_panel.update()

    def update_events(self):
        self.event_list.clear()
        for event in list(self.model.events)[-100:][::-1]:
            self.event_list.addItem(f'{event["time"]:8.1f}s   {event["kind"].upper():8}   {event["text"]}')
            self.event_list.item(self.event_list.count()-1).setData(Qt.UserRole, event['time'])
        self.slider.update()

    def on_snapshot(self, data, replay=False):
        if not replay:
            self._live_snapshot = data
            if self.replay_mode:
                return
        now = time.monotonic()
        self.first_snapshot = self.first_snapshot or now
        self.snapshot_count += 1
        self.model.snapshot(data, now)
        s = data['mission']
        self.state_label.setText(f'{STATES[s["state"]]}  /  drive={s["drive_enabled"]}\n'
                                 f'{s["reason"]}\nRun: {s["run_id"]}\nElapsed: {s["elapsed"]:.1f} s')
        self.map_panel.update()
        self.update_events()

    def on_link(self, data):
        if self.replay_mode:
            return
        self.link_label.setText(f'LINK  {data["state"]}  /  {data["rate"]:.2f} Hz  /  age {data["age"]:.1f}s')
        self.link_label.setStyleSheet('color: ' + ('#5ade9b' if data['state'] == 'OK' else '#ef6b7d' if data['state'] == 'LOST' else '#f4cf63'))

    def on_camera(self, image, replay=False):
        if not replay:
            self._live_image = image
            if self.replay_mode:
                return
        self.first_camera = self.first_camera or time.monotonic()
        self.camera_count += 1
        self.camera_panel.image = image
        self.camera_panel.update()

    def on_command(self, data):
        if self.replay_mode:
            return
        self.model.command(data, time.monotonic())
        self.command_label.setText(f'{data["command"]}: {data["outcome"]}\n{data["reason"]}')
        self.update_events()


    def choose_bag(self):
        path = QFileDialog.getExistingDirectory(self, 'Open rosbag2 directory')
        if path:
            self.begin_replay()
            self.command_label.setText('Loading recorded bag…')
            self.bag_requested.emit(path)

    def begin_replay(self):
        self.replay_mode = True
        for button in self.buttons.values():
            button.setEnabled(False)
        self.link_label.setText('REPLAY / offline display · commands disabled')
        self.link_label.setStyleSheet('color: #6499ff')

    def archive_loaded(self, data):
        self.begin_replay()
        self.replay_duration = data['duration']
        self._archive_events = data['events']
        self.model = DisplayModel()
        self.map_panel.model = self.slider.model = self.model
        self.model.events = deque(self._archive_events, maxlen=10000)
        self.slider.extent = (0., self.replay_duration)
        self.slider.setEnabled(True)
        self.slider.setRange(0, max(1, math.ceil(self.replay_duration*1000)))
        self.slider.setValue(0)
        self.camera_panel.image = QImage()
        self.state_label.setText('REPLAY / no snapshot at selected time')
        self.command_label.setText('Recorded bag: '+data['path'])
        self.update_events()
        self.map_panel.update()
        self.camera_panel.update()

    def replay_frame(self, data):
        if not self.replay_mode:
            return
        if data['snapshot']:
            self.on_snapshot(data['snapshot'], replay=True)
        else:
            self.model.current = None
            self.model.layers = {}
            self.state_label.setText('REPLAY / no snapshot at selected time')
        if data['camera'] is not None:
            self.on_camera(data['camera'], replay=True)
        else:
            self.camera_panel.image = QImage()
            self.camera_panel.update()
        self.model.events = deque(self._archive_events, maxlen=10000)
        self.time_label.setText(f'{data["seconds"]:.2f} / {self.replay_duration:.2f} s')
        self.update_events()
        self.map_panel.update()

    def seek(self, value):
        if self.replay_mode:
            self.seek_requested.emit(value/1000)

    def seek_event(self, item):
        if self.replay_mode and self.replay_duration:
            seconds = item.data(Qt.UserRole)
            self.slider.setValue(math.ceil(seconds*1000))

    def replay_error(self, text):
        self.resume_live()
        self.command_label.setText('Bag error: '+text)

    def resume_live(self):
        self.replay_mode = False
        self.model = DisplayModel()
        self.map_panel.model = self.slider.model = self.model
        self.slider.extent = None
        self.slider.setEnabled(False)
        self.slider.setRange(0, 1000)
        self.slider.setValue(1000)
        self.time_label.setText('LIVE')
        for button in self.buttons.values():
            button.setEnabled(True)
        if self._live_snapshot:
            self.on_snapshot(self._live_snapshot)
        if self._live_image is not None:
            self.on_camera(self._live_image)
        self.update_events()


    def on_recording(self, data):
        if not self.replay_mode:
            self.recording_label.setText(f'BAG / {data["state"]} · {data["bytes"]/1048576:.1f} MiB')
            self.recording_label.setToolTip(data['path']+'\n'+data['reason'])
