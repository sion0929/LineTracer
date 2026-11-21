"""One worker owns the reader; coalesced seek requests keep disk I/O off the Qt thread."""
import threading
from .bag_archive import BagArchive
from .adapter import display_snapshot


class ReplayWorker(threading.Thread):
    def __init__(self, signals, image_converter):
        super().__init__(name='gcs-bag-reader', daemon=True)
        self.signals, self.image_converter = signals, image_converter
        self.condition = threading.Condition()
        self.pending = None
        self.stopping = False
        self.archive = None

    def request(self, action, value):
        with self.condition:
            self.pending = (action, value)
            self.condition.notify()

    def close(self):
        with self.condition:
            self.stopping = True
            self.condition.notify()
        self.join(timeout=3.)

    def run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.pending or self.stopping)
                if self.stopping:
                    return
                action, value = self.pending
                self.pending = None
            try:
                if action == 'open':
                    self.archive = BagArchive(value)
                    self.signals.archive.emit({'path': self.archive.path, 'duration': self.archive.duration,
                                               'events': self.archive.events, 'counts': self.archive.counts})
                    value = 0.
                if self.archive:
                    snapshot, camera = self.archive.at(value)
                    self.signals.replay_frame.emit({
                        'snapshot': display_snapshot(snapshot) if snapshot else None,
                        'camera': self.image_converter(camera) if camera else None, 'seconds': value})
            except Exception as exc:
                self.signals.replay_error.emit(str(exc))
