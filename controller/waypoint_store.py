from __future__ import annotations

import json
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from threading import RLock
from typing import Any


@dataclass(frozen=True)
class WaypointRecord:
    name: str
    joint_pos: list[float]
    saved_at: str


class WaypointStore:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._lock = RLock()
        self._records: dict[str, dict[str, Any]] = {}

    def load(self) -> None:
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            if not self.path.exists():
                self._records = {}
                self._flush()
                return

            raw = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(raw, dict):
                raise ValueError("waypoint file must contain a JSON object")

            records: dict[str, dict[str, Any]] = {}
            for name, value in raw.items():
                records[name] = self._validate_record(name, value)
            self._records = records

    def get(self, name: str) -> WaypointRecord | None:
        with self._lock:
            record = self._records.get(name)
            if record is None:
                return None
            return self._to_record(name, record)

    def save(
        self,
        name: str,
        joint_pos: list[float],
    ) -> WaypointRecord:
        if not name:
            raise ValueError("name must not be empty")
        record = {
            "joint_pos": self._validate_vector("joint_pos", joint_pos, 6),
            "saved_at": datetime.now(timezone.utc).isoformat(),
        }
        with self._lock:
            self._records[name] = record
            self._flush()
            return self._to_record(name, record)

    def _flush(self) -> None:
        data = {
            name: deepcopy(record)
            for name, record in sorted(self._records.items(), key=lambda item: item[0])
        }
        self.path.write_text(
            json.dumps(data, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _to_record(self, name: str, record: dict[str, Any]) -> WaypointRecord:
        return WaypointRecord(
            name=name,
            joint_pos=list(record["joint_pos"]),
            saved_at=str(record["saved_at"]),
        )

    def _validate_record(self, name: str, value: Any) -> dict[str, Any]:
        if not isinstance(value, dict):
            raise ValueError(f"waypoint {name!r} must be an object")
        return {
            "joint_pos": self._validate_vector("joint_pos", value.get("joint_pos"), 6),
            "saved_at": str(value.get("saved_at") or ""),
        }

    @staticmethod
    def _validate_vector(field: str, value: Any, length: int) -> list[float]:
        if not isinstance(value, list) or len(value) != length:
            raise ValueError(f"{field} must be a list of length {length}")
        return [float(item) for item in value]
