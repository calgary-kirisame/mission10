from __future__ import annotations

from enum import Enum


class MissionPhase(str, Enum):
    HOVER = "hover"
    ORBIT = "orbit"
    RETURN = "return"
