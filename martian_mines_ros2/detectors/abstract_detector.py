from abc import ABC, abstractmethod
from typing import List
import numpy as np
from martian_mines_ros2.msg import BoundingBoxLabeled


class AbstractDetector(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def detect(self, frame: np.ndarray) -> List[BoundingBoxLabeled]:
        raise NotImplementedError("Detect not implemented yet")

    @abstractmethod
    def draw_markers(self, frame: np.ndarray) -> np.ndarray:
        raise NotImplementedError("Draw markers not implemented yet")