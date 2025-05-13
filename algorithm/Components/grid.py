from Helpers.constants import (
    EXPANDED_CELL_SIZE,
    CAPTURE_COST,
    CLOSE_OBSTACLE_COST,
    TURN_PADDING_DISTANCE,
    MID_TURN_PADDING_DISTANCE,
)
from Helpers.motion import Heading

import math

from typing import List
from warnings import warn


class GridState:
    def __init__(
        self,
        pos_x,
        pos_y,
        facing_direction: Heading = Heading.NORTH,
        target_obstacle_ids: List = None,
        movement_penalty=0,
    ):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.facing_direction = facing_direction
        self.target_obstacle_ids = target_obstacle_ids if target_obstacle_ids is not None else []
        self.movement_penalty = movement_penalty

    def __repr__(self):
        return "GridState(x: {}, y: {}, direction: {}, screenshot: {})".format(
            self.pos_x, self.pos_y, self.facing_direction, self.target_obstacle_ids
        )
    
    def mark_target(self, obstacle_id: str):
        self.target_obstacle_ids.append(obstacle_id)

    def matches_state(self, pos_x, pos_y, facing_direction):
        return self.pos_x == pos_x and self.pos_y == pos_y and self.facing_direction == facing_direction

    def matches_position(self, pos_x, pos_y) -> bool:
        return self.pos_x == pos_x and self.pos_y == pos_y

    def serialize(self):
        return {
            "x": self.pos_x,
            "y": self.pos_y,
            "d": self.facing_direction,
            "s": self.target_obstacle_ids
        }


class Object(GridState):
    def __init__(self, pos_x: int, pos_y: int, facing_direction: Heading, obstacle_id: int):
        super().__init__(pos_x, pos_y, facing_direction)
        self.obstacle_id = obstacle_id

    def __eq__(self, other):
        return (
            self.pos_x == other.pos_x
            and self.pos_y == other.pos_y
            and self.facing_direction == other.facing_direction
        )

    def get_obstacle_id(self):
        return self.obstacle_id

    def generate_view_states(self) -> List[GridState]:
        view_states = []
        offset = 3 * EXPANDED_CELL_SIZE

        if self.facing_direction == Heading.NORTH:
            positions = [
                (self.pos_x, self.pos_y + offset),
                (self.pos_x - 1, self.pos_y + 2 + offset),
                (self.pos_x + 1, self.pos_y + 2 + offset),
                (self.pos_x, self.pos_y + 1 + offset),
                (self.pos_x, self.pos_y + 2 + offset),
            ]
            directions = [Heading.SOUTH] * 5

        elif self.facing_direction == Heading.SOUTH:
            positions = [
                (self.pos_x, self.pos_y - offset),
                (self.pos_x + 1, self.pos_y - 2 - offset),
                (self.pos_x - 1, self.pos_y - 2 - offset),
                (self.pos_x, self.pos_y - 1 - offset),
                (self.pos_x, self.pos_y - 2 - offset),
            ]
            directions = [Heading.NORTH] * 5

        elif self.facing_direction == Heading.EAST:
            positions = [
                (self.pos_x + offset, self.pos_y),
                (self.pos_x + 2 + offset, self.pos_y + 1),
                (self.pos_x + 2 + offset, self.pos_y - 1),
                (self.pos_x + 1 + offset, self.pos_y),
                (self.pos_x + 2 + offset, self.pos_y),
            ]
            directions = [Heading.WEST] * 5

        elif self.facing_direction == Heading.WEST:
            positions = [
                (self.pos_x - offset, self.pos_y),
                (self.pos_x - 2 - offset, self.pos_y + 1),
                (self.pos_x - 2 - offset, self.pos_y - 1),
                (self.pos_x - 1 - offset, self.pos_y),
                (self.pos_x - 2 - offset, self.pos_y),
            ]
            directions = [Heading.EAST] * 5

        costs = [
            CLOSE_OBSTACLE_COST,
            CAPTURE_COST,
            CAPTURE_COST,
            CLOSE_OBSTACLE_COST // 2,
            0,
        ]

        for idx, pos in enumerate(positions):
            if Grid.is_within_grid(*pos):
                view_states.append(
                    GridState(pos[0], pos[1], directions[idx], [self.obstacle_id], costs[idx])
                )

        return view_states


class Grid:
    size_x: int = 20
    size_y: int = 20

    def __init__(self, size_x: int, size_y: int):
        self.size_x = size_x
        self.size_y = size_y
        self.obstacle_list: List[Object] = []

    def list_obs(self):
        return self.obstacle_list

    def insert_obs(self, obstacle: Object):
        if obstacle not in self.obstacle_list:
            self.obstacle_list.append(obstacle)

    def clear_obs(self):
        self.obstacle_list = []

    def is_position_reachable(self, pos_x: int, pos_y: int) -> bool:
        if not self.is_valid_position(pos_x, pos_y):
            return False

        for obstacle in self.obstacle_list:
            if abs(obstacle.pos_x - pos_x) + abs(obstacle.pos_y - pos_y) <= 2:
                return False
            if max(abs(obstacle.pos_x - pos_x), abs(obstacle.pos_y - pos_y)) < 2:
                return False

        return True

    def is_turn_possible(self, from_x: int, from_y: int, to_x: int, to_y: int, direction: Heading) -> bool:
        path_points = self.calculate_turn_points(from_x, from_y, to_x, to_y, direction)

        if not self.is_valid_position(from_x, from_y) or not self.is_valid_position(to_x, to_y):
            return False

        for obstacle in self.obstacle_list:
            pre_dist = math.dist((from_x, from_y), (obstacle.pos_x, obstacle.pos_y))
            post_dist = math.dist((to_x, to_y), (obstacle.pos_x, obstacle.pos_y))
            if pre_dist < TURN_PADDING_DISTANCE or post_dist < TURN_PADDING_DISTANCE:
                return False

            for point in path_points:
                if math.dist(point, (obstacle.pos_x, obstacle.pos_y)) < MID_TURN_PADDING_DISTANCE:
                    return False

        return True

    def is_half_turn_possible(self, from_x: int, from_y: int, to_x: int, to_y: int) -> bool:
        if not self.is_valid_position(from_x, from_y) or not self.is_valid_position(to_x, to_y):
            return False

        padding = 2 * EXPANDED_CELL_SIZE
        min_x, max_x = sorted([from_x, to_x])
        min_y, max_y = sorted([from_y, to_y])

        for obs in self.obstacle_list:
            if abs(to_x - from_x) > abs(to_y - from_y):
                if min_x <= obs.pos_x <= max_x and min_y - padding <= obs.pos_y <= max_y + padding:
                    return False
            else:
                if min_x - padding <= obs.pos_x <= max_x + padding and min_y <= obs.pos_y <= max_y:
                    return False

        return True
    
    def is_valid_state(self, state: GridState) -> bool:
        return self.is_valid_position(state.pos_x, state.pos_y)

    def is_valid_position(self, x: int, y: int) -> bool:
        return 1 <= x < self.size_x - 1 and 1 <= y < self.size_y - 1

    def generate_valid_viewpoints(self) -> List[List[GridState]]:
        view_positions = []
        for obstacle in self.obstacle_list:
            if obstacle.facing_direction == Heading.SKIP:
                continue
            valid_states = [s for s in obstacle.generate_view_states() if self.is_position_reachable(s.pos_x, s.pos_y)]
            view_positions.append(valid_states)
        return view_positions

    def get_obstacle_id(self, obstacle_id: int) -> Object:
        for obstacle in self.obstacle_list:
            if obstacle.obstacle_id == obstacle_id:
                return obstacle
        return None

    @staticmethod
    def calculate_turn_points(x: int, y: int, new_x: int, new_y: int, direction: Heading):
        mid_x, mid_y = (x + new_x) / 2, (y + new_y) / 2

        if direction in [Heading.NORTH, Heading.SOUTH]:
            tr_x, tr_y = x, new_y
            p1 = ((x + mid_x) / 2, mid_y)
            p2 = ((tr_x + mid_x) / 2, (tr_y + mid_y) / 2)
            p3 = (mid_x, (new_y + mid_y) / 2)
        elif direction in [Heading.EAST, Heading.WEST]:
            tr_x, tr_y = new_x, y
            p1 = (mid_x, (y + mid_y) / 2)
            p2 = ((tr_x + mid_x) / 2, (tr_y + mid_y) / 2)
            p3 = ((new_x + mid_x) / 2, mid_y)
        else:
            raise ValueError("Invalid direction")

        return [p1, p2, p3]

    @classmethod
    def is_within_grid(cls, x: int, y: int):
        return 0 <= x < cls.size_x and 0 <= y < cls.size_y
