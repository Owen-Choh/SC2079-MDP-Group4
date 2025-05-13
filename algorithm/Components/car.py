from typing import List
from Components.grid import GridState
from Helpers.motion import Heading


class Car:
    def __init__(self, start_x: int, start_y: int, initial_direction: Heading):
        self.path_history: List[GridState] = [GridState(start_x, start_y, initial_direction)]

    def start_state(self):
        return self.path_history[0]
