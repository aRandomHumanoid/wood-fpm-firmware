from __future__ import annotations

import threading
from collections import deque
from typing import Deque, Dict, List


class SerialConsoleBuffer:
    def __init__(self, max_entries: int = 500):
        self._entries: Deque[Dict[str, object]] = deque(maxlen=max_entries)
        self._lock = threading.Lock()

    def append(self, entry: Dict[str, object]):
        with self._lock:
            self._entries.append(dict(entry))

    def snapshot(self) -> List[Dict[str, object]]:
        with self._lock:
            return [dict(entry) for entry in self._entries]

    def clear(self):
        with self._lock:
            self._entries.clear()