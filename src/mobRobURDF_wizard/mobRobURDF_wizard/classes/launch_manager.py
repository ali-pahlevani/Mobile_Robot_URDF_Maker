"""Run the Gazebo simulation as a managed child process.

Launches ``ros2 launch mobRobURDF_launch gazebo_test.launch.py`` in its own
process group so the whole tree (gz sim, controllers, bridges, RViz) can be
torn down cleanly with a single signal — gracefully (SIGINT, which ros2 launch
forwards to its children), escalating to SIGTERM/SIGKILL only if needed.
"""

import os
import signal
import subprocess
import logging

from PyQt5.QtCore import QObject, QThread, QTimer, pyqtSignal

logger = logging.getLogger(__name__)

LAUNCH_PACKAGE = "mobRobURDF_launch"
LAUNCH_FILE = "gazebo_test.launch.py"


class _OutputReader(QThread):
    """Reads merged stdout/stderr line-by-line, then reports the exit code."""
    line = pyqtSignal(str)
    finished_rc = pyqtSignal(int)

    def __init__(self, proc):
        super().__init__()
        self._proc = proc

    def run(self):
        try:
            for line in iter(self._proc.stdout.readline, ""):
                if not line:
                    break
                self.line.emit(line.rstrip("\n"))
        except Exception:
            pass
        rc = self._proc.wait()
        self.finished_rc.emit(rc)


class LaunchManager(QObject):
    started = pyqtSignal()
    stopped = pyqtSignal(int)   # process return code
    output = pyqtSignal(str)     # one log line

    def __init__(self, parent=None):
        super().__init__(parent)
        self._proc = None
        self._reader = None

    def is_running(self):
        return self._proc is not None and self._proc.poll() is None

    # ── Start ────────────────────────────────────────────────────────────
    def start(self):
        if self.is_running():
            return
        cmd = ["ros2", "launch", LAUNCH_PACKAGE, LAUNCH_FILE]
        logger.info("Launching simulation: %s", " ".join(cmd))
        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,  # own process group for clean teardown
        )
        self._reader = _OutputReader(self._proc)
        self._reader.line.connect(self.output)
        self._reader.finished_rc.connect(self._on_finished)
        self._reader.start()
        self.started.emit()

    def _on_finished(self, rc):
        logger.info("Simulation process exited with code %s", rc)
        self._proc = None
        self.stopped.emit(rc)

    # ── Stop (non-blocking, escalates over time) ─────────────────────────
    def stop(self):
        if not self.is_running():
            return
        logger.info("Stopping simulation (SIGINT)…")
        self._signal_group(signal.SIGINT)
        QTimer.singleShot(8000, self._escalate_term)

    def _escalate_term(self):
        if self.is_running():
            logger.warning("Simulation still running; sending SIGTERM…")
            self._signal_group(signal.SIGTERM)
            QTimer.singleShot(3000, self._escalate_kill)

    def _escalate_kill(self):
        if self.is_running():
            logger.warning("Simulation still running; sending SIGKILL…")
            self._signal_group(signal.SIGKILL)

    # ── Blocking teardown for application exit ───────────────────────────
    def shutdown(self):
        """Synchronously stop the simulation — used on window close."""
        if not self.is_running():
            return
        for sig, wait in ((signal.SIGINT, 6), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            self._signal_group(sig)
            try:
                self._proc.wait(timeout=wait)
                break
            except subprocess.TimeoutExpired:
                continue
        self._proc = None

    # ── Helpers ──────────────────────────────────────────────────────────
    def _signal_group(self, sig):
        if self._proc is None:
            return
        try:
            os.killpg(os.getpgid(self._proc.pid), sig)
        except ProcessLookupError:
            pass
        except Exception as e:
            logger.warning("Failed to signal simulation process group: %s", e)
