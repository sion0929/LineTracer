#!/usr/bin/env python3
"""Offscreen acceptance using the real reader worker and a recorded bag."""
import os
import math
os.environ['QT_QPA_PLATFORM']='offscreen'
import sys
import time
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QImage
from cv_bridge import CvBridge
from line_follower_gcs.bag_archive import BagArchive
from line_follower_gcs.gcs_mission_manager import Signals
from line_follower_gcs.replay_worker import ReplayWorker
from line_follower_gcs.window import GcsWindow


def main():
    archive = BagArchive(sys.argv[1])
    running = next(i for i,s in enumerate(archive.snapshots) if s.mission.state==3)
    target = (archive.snapshot_stamps[running]-archive.start_ns)/1e9+.001
    app=QApplication([])
    window=GcsWindow()
    window.show()
    signals=Signals()
    errors=[]
    def convert(msg):
        rgb=CvBridge().imgmsg_to_cv2(msg,'rgb8')
        return QImage(rgb.data,rgb.shape[1],rgb.shape[0],rgb.strides[0],QImage.Format_RGB888).copy()
    worker=ReplayWorker(signals,convert)
    signals.archive.connect(window.archive_loaded)
    signals.replay_frame.connect(window.replay_frame)
    signals.replay_error.connect(errors.append)
    window.seek_requested.connect(lambda seconds:worker.request('seek',seconds))
    def wait(predicate):
        end=time.monotonic()+10
        while time.monotonic()<end:
            app.processEvents()
            if errors:
                raise RuntimeError(str(errors))
            if predicate():
                return
            time.sleep(.01)
        raise RuntimeError('Qt bag worker deadline')
    worker.start()
    try:
        worker.request('open',sys.argv[1])
        wait(lambda: window.replay_mode)
        window.slider.setValue(math.ceil(target*1000))
        wait(lambda: window.model.current and window.model.current['mission']['state']==3 and
             not window.camera_panel.image.isNull())
        assert window.model.layers['observed'] and window.model.layers['planned']
        assert all(not b.isEnabled() for b in window.buttons.values())
        print(f'PASS | UI bag RUNNING scrub | t={target:.3f}s; map observed/planned; camera={window.camera_panel.image.width()}x{window.camera_panel.image.height()}; commands disabled')
        window.slider.setValue(window.slider.maximum())
        wait(lambda: window.model.current and window.model.current['mission']['state']==8)
        assert any(e['text']=='E_STOP' for e in window.model.events)
        print(f'PASS | UI bag end scrub | state=STOPPED; events={len(window.model.events)}; same shared extractor')
        window.resume_live()
        assert not window.replay_mode and not window.slider.isEnabled()
        print('PASS | UI LIVE return | replay ended; seek disabled')
    finally:
        worker.close()
        window.close()
    return 0


if __name__=='__main__':
    raise SystemExit(main())
