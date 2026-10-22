"""CPU / RAM / temperature read from /proc once per second on a background thread (same method as CoVip's SysStats)."""
import os
import threading
import time


class SysStats(threading.Thread):
    """cpu_sys: whole machine 0-100 %. cpu_node: this process in % of ONE core (0-400 on a Pi 4), like htop."""
    def __init__(self):
        super().__init__(daemon=True)
        self.cpu_sys = self.cpu_node = self.temp = self.rss_mb = None
        self._tick = os.sysconf('SC_CLK_TCK')
        self._page_mb = os.sysconf('SC_PAGE_SIZE') / 1e6

    @staticmethod
    def _cpu_total():
        with open('/proc/stat') as f:
            v = [int(x) for x in f.readline().split()[1:]]
        return sum(v) - v[3] - v[4], sum(v)

    def _self_ticks(self):
        with open('/proc/self/stat') as f:
            p = f.read().rsplit(')', 1)[1].split()
        with open('/proc/self/statm') as f:
            self.rss_mb = int(f.read().split()[1]) * self._page_mb
        return int(p[11]) + int(p[12])

    def run(self):
        busy0, tot0 = self._cpu_total()
        me0, w0 = self._self_ticks(), time.time()
        while True:
            time.sleep(1.0)
            try:
                busy, tot = self._cpu_total()
                me, w = self._self_ticks(), time.time()
                self.cpu_sys = (busy-busy0) / max(tot-tot0, 1) * 100
                self.cpu_node = (me-me0) / self._tick / max(w-w0, 1e-3) * 100
                busy0, tot0, me0, w0 = busy, tot, me, w
                with open('/sys/class/thermal/thermal_zone0/temp') as f:
                    self.temp = int(f.read()) / 1000
            except (OSError, ValueError, IndexError):
                pass
