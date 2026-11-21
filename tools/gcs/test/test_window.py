"""D-12 offscreen layout and layer/camera update acceptance, without a ROS graph."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtGui import QImage
from PyQt5.QtWidgets import QApplication
from line_follower_gcs.window import GcsWindow


def test_four_by_four_and_updates():
    app = QApplication.instance() or QApplication([])
    window = GcsWindow()
    window.resize(1600, 1000)
    window.show()
    expected = [(window.map_panel, (0, 0, 3, 3)), (window.command_panel, (0, 3, 2, 1)),
                (window.camera_panel, (2, 3, 1, 1)), (window.timeline_panel, (3, 0, 1, 4))]
    for widget, placement in expected:
        assert window.grid.getItemPosition(window.grid.indexOf(widget)) == placement
    window.on_snapshot({'mission': {'state': 3, 'drive_enabled': True, 'run_id': 'synthetic',
                                    'reason': 'test', 'elapsed': 21},
                        'pose': (.1, .2, .3), 'observed': [[(0, 0), (.2, .2)]],
                        'memory': [[(.2, .2), (.3, .3)]], 'planned': [[(0, 0), (.4, .4)]],
                        'candidates': [[(0, 0), (.2, -.2)]],
                        'candidate_quality': [{'id':7,'rule_score':.8,'mlp_score':float('nan'),'selected':True,'lines':[[(0,0),(.2,-.2)]]}], 'mode': 3, 'stop_reason': ''})
    image = QImage(640, 480, QImage.Format_RGB888)
    image.fill(0x228844)
    window.on_camera(image)
    window.on_link({'state': 'OK', 'rate': 2., 'age': .1})
    app.processEvents()
    assert all(window.map_panel.layer_counts[name] > 0 for name in ('observed', 'memory', 'planned', 'candidates'))
    assert not window.camera_panel.image.isNull()
    assert window.snapshot_count == 1 and window.camera_count == 1
    assert 'RUNNING' in window.state_label.text()
    assert len(window.model.events) >= 2
    window.on_link({'state':'LOST','rate':0.,'age':3.})
    assert '#ef6b7d' in window.link_label.styleSheet()
    assert window.replay_button.isEnabled() is True
    assert window.size().width() == 1600 and window.size().height() == 1000
    sent = []
    window.command_requested.connect(sent.append)
    for command in ('start', 'bag_start', 'bag_stop', 'kill'):
        window.buttons[command].click()
    assert sent == ['start', 'bag_start', 'bag_stop', 'kill']
    window.close()


def test_replay_switch_scrub_and_live_isolation():
    app = QApplication.instance() or QApplication([])
    window = GcsWindow()
    assert window.replay_button.isEnabled()
    window.archive_loaded({'path': '/test/bag', 'duration': 10., 'events': [
        {'time': 2., 'kind': 'mission', 'text': 'RUNNING', 'run_id': 'r'}]})
    assert window.replay_mode and window.slider.isEnabled()
    assert all(not b.isEnabled() for b in window.buttons.values())
    seeks = []
    window.seek_requested.connect(seeks.append)
    window.slider.setValue(5000)
    assert seeks[-1] == 5.
    image = QImage(640, 480, QImage.Format_RGB888)
    image.fill(0x112233)
    data = {'mission': {'state': 3, 'drive_enabled': True, 'run_id': 'bag', 'reason': 'recorded', 'elapsed': 22},
            'pose': (1., 2., 0.), 'observed': [], 'memory': [], 'planned': [], 'candidates': []}
    window.replay_frame({'snapshot': data, 'camera': image, 'seconds': 5.})
    assert window.model.current['mission']['run_id'] == 'bag'
    assert not window.camera_panel.image.isNull()
    live = dict(data, mission=dict(data['mission'], run_id='live'))
    window.on_snapshot(live)
    assert window.model.current['mission']['run_id'] == 'bag'
    window.replay_frame({'snapshot':None,'camera':None,'seconds':0.})
    assert 'no snapshot' in window.state_label.text()
    assert window.camera_panel.image.isNull()
    window.resume_live()
    assert window.model.current['mission']['run_id'] == 'live'
    assert not window.slider.isEnabled()
    window.close()
