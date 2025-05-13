import heapq
import math
import numpy as np
import time

from python_tsp.exact import solve_tsp_dynamic_programming

from Components.grid import GridState, Object, Grid
from Components.car import Car
from Helpers.motion import Heading, MovementType
from Helpers.constants import (
    MOVE_DIRECTIONS,
    TURN_COST_FACTOR,
    HALF_TURN_COST_FACTOR,
    MAX_ITERATIONS,
    TURN_RADIUS,
    SAFETY_COST,
    FULL_TURN_RADIUS,
    HALF_TURN_RADIUS,
    REVERSE_COST_FACTOR,
)

class PathPlanner:

    def __init__(
        self,
        grid_width: int = 20,
        grid_height: int = 20,
        robot_obj: Car = None,
        start_x: int = 1,
        start_y: int = 1,
        start_facing: Heading = Heading.NORTH,
    ):
        self.grid = Grid(grid_width, grid_height)

        self.robot = robot_obj if robot_obj else Car(start_x, start_y, start_facing)

        self.path_table = dict()
        self.cost_table = dict()
        self.motion_table = dict()

    def reset_object(self):
        self.grid.clear_obs()

    def insert_obs(self, obs_x, obs_y, obs_direction, obs_id):
        self.grid.insert_obs(Object(obs_x, obs_y, obs_direction, obs_id))

    def prepare_all_paths(self, state_list) -> int:
        path_counter = 0
        for idx_start in range(len(state_list) - 1):
            for idx_end in range(idx_start + 1, len(state_list)):
                path_counter += 1
                self.search_astar_path(state_list[idx_start], state_list[idx_end])
                if path_counter % 100 == 0:
                    print(f"Checked {path_counter} paths so far.")

    def get_best_path(self):
        start_time = time.time()
        best_total_cost = 1e9
        best_path = []
        attempts_checked = 0

        view_positions = self.grid.generate_valid_viewpoints()
        num_views = len(view_positions)

        step_start = time.time()

        for binary_str in self.generate_visit_flags(num_views):
            attempts_checked += 1
            if attempts_checked % 100 == 0:
                print(f"Checked {attempts_checked} path combinations so far...")

            states_to_visit = [self.robot.start_state()]
            selected_viewpoints = []

            for i in range(num_views):
                if binary_str[i] == "1":
                    selected_viewpoints.append(view_positions[i])
                    states_to_visit.extend(view_positions[i])

            self.prepare_all_paths(states_to_visit)

            all_combinations = PathPlanner.build_combinations(selected_viewpoints, 0, [], [], 2000)

            for combo in all_combinations:
                visit_sequence = [0]
                pointer, travel_cost = 1, 0

                for i, vp in enumerate(selected_viewpoints):
                    visit_sequence.append(pointer + combo[i])
                    travel_cost += vp[combo[i]].movement_penalty
                    pointer += len(vp)

                dist_matrix = np.zeros((len(visit_sequence), len(visit_sequence)))

                for a in range(len(visit_sequence) - 1):
                    for b in range(a + 1, len(visit_sequence)):
                        src = states_to_visit[visit_sequence[a]]
                        dst = states_to_visit[visit_sequence[b]]

                        if (src, dst) in self.cost_table:
                            dist_matrix[a, b] = self.cost_table[(src, dst)]
                        else:
                            dist_matrix[a, b] = 1e9

                        dist_matrix[b, a] = dist_matrix[a, b]

                dist_matrix[:, 0] = 0

                perm, tsp_cost = solve_tsp_dynamic_programming(dist_matrix)

                if tsp_cost + travel_cost >= best_total_cost:
                    continue

                best_total_cost = tsp_cost + travel_cost
                best_path = [states_to_visit[0]]

                for k in range(len(perm) - 1):
                    source = states_to_visit[visit_sequence[perm[k]]]
                    target = states_to_visit[visit_sequence[perm[k + 1]]]
                    partial_path = self.path_table[(source, target)]

                    for i in range(1, len(partial_path)):
                        best_path.append(
                            GridState(
                                partial_path[i][0],
                                partial_path[i][1],
                                partial_path[i][2],
                            )
                        )

                    target_id = target.target_obstacle_ids[0]
                    matched_obs = self.grid.get_obstacle_id(target_id)

                    if matched_obs:
                        rel_pos = PathPlanner.capture_position_relative(best_path[-1], matched_obs)
                        formatted_id = f"{target_id}_{rel_pos}"
                        best_path[-1].mark_target(formatted_id)
                    else:
                        raise ValueError(f"Obstacle with id {target_id} not found")

            if best_path:
                break

        if not best_path:
            print("WARNING: No valid path found!")

        return best_path, best_total_cost


    def search_astar_path(self, state_start: GridState, state_end: GridState) -> None:
        if (state_start, state_end) in self.path_table:
            return

        g_cost = {(state_start.pos_x, state_start.pos_y, state_start.facing_direction): 0}

        heap = [
            (self.estimate_dist(state_start, state_end), state_start.pos_x, state_start.pos_y, state_start.facing_direction)
        ]

        explored = set()
        parent_trace = dict()

        while heap:
            _, curr_x, curr_y, curr_dir = heapq.heappop(heap)

            if (curr_x, curr_y, curr_dir) in explored:
                continue

            if state_end.matches_state(curr_x, curr_y, curr_dir):
                self.store_computed_path(state_start, state_end, parent_trace, g_cost[(curr_x, curr_y, curr_dir)])
                return

            explored.add((curr_x, curr_y, curr_dir))
            base_cost = g_cost[(curr_x, curr_y, curr_dir)]

            for new_x, new_y, new_dir, env_cost, move in self.explore_neighbors(curr_x, curr_y, curr_dir):
                if (new_x, new_y, new_dir) in explored:
                    continue

                if (
                    (curr_x, curr_y, curr_dir, new_x, new_y, new_dir) not in self.motion_table and
                    (new_x, new_y, new_dir, curr_x, curr_y, curr_dir) not in self.motion_table
                ):
                    self.motion_table[(curr_x, curr_y, curr_dir, new_x, new_y, new_dir)] = move

                rotate_cost = TURN_COST_FACTOR * Heading.calculate_turn_cost(curr_dir, new_dir)
                if rotate_cost == 0:
                    rotate_cost = 1

                back_cost = REVERSE_COST_FACTOR * move.get_reverse_cost()
                if back_cost == 0:
                    back_cost = 1

                swing_cost = HALF_TURN_COST_FACTOR * move.get_half_turn_cost()
                if swing_cost == 0:
                    swing_cost = 1

                move_total = back_cost * swing_cost * rotate_cost
                total_move_cost = move_total + env_cost

                if state_end.matches_state(new_x, new_y, new_dir):
                    screenshot_penalty = state_end.movement_penalty
                else:
                    screenshot_penalty = 0

                full_cost = (
                    base_cost
                    + total_move_cost
                    + screenshot_penalty
                    + self.estimate_dist(GridState(new_x, new_y, new_dir), state_end)
                )

                if (new_x, new_y, new_dir) not in g_cost or g_cost[(new_x, new_y, new_dir)] > base_cost + total_move_cost:
                    g_cost[(new_x, new_y, new_dir)] = base_cost + total_move_cost + screenshot_penalty
                    heapq.heappush(heap, (full_cost, new_x, new_y, new_dir))
                    parent_trace[(new_x, new_y, new_dir)] = (curr_x, curr_y, curr_dir)

    def compute_safety_penalty(self, pos_x: int, pos_y: int) -> int:
        padding = 2
        for obstacle in self.grid.list_obs():
            if abs(obstacle.pos_x - pos_x) <= padding and abs(obstacle.pos_y - pos_y) <= padding:
                return SAFETY_COST
            if abs(obstacle.pos_y - pos_y) <= padding and abs(obstacle.pos_x - pos_x) <= padding:
                return SAFETY_COST
        return 0

    def explore_neighbors(self, x_coord, y_coord, curr_dir):
        next_states = []

        for dx, dy, intended_dir in MOVE_DIRECTIONS:
            if intended_dir == curr_dir:
                if self.grid.is_position_reachable(x_coord + dx, y_coord + dy):
                    cost = self.compute_safety_penalty(x_coord + dx, y_coord + dy)
                    next_states.append((x_coord + dx, y_coord + dy, intended_dir, cost, MovementType.FORWARD))

                if self.grid.is_position_reachable(x_coord - dx, y_coord - dy):
                    cost = self.compute_safety_penalty(x_coord - dx, y_coord - dy)
                    next_states.append((x_coord - dx, y_coord - dy, intended_dir, cost, MovementType.REVERSE))

                offset_dx, offset_dy = self.compute_half_turn_shift(curr_dir)

                if curr_dir in [Heading.NORTH, Heading.SOUTH]:
                    if self.grid.is_half_turn_possible(x_coord, y_coord, x_coord + offset_dx, y_coord + offset_dy):
                        cost = self.compute_safety_penalty(x_coord + offset_dx, y_coord + offset_dy)
                        next_states.append((x_coord + offset_dx, y_coord + offset_dy, intended_dir, cost, MovementType.FORWARD_OFFSET_RIGHT))

                    if self.grid.is_half_turn_possible(x_coord, y_coord, x_coord - offset_dx, y_coord + offset_dy):
                        cost = self.compute_safety_penalty(x_coord - offset_dx, y_coord + offset_dy)
                        next_states.append((x_coord - offset_dx, y_coord + offset_dy, intended_dir, cost, MovementType.FORWARD_OFFSET_LEFT))

                    if self.grid.is_half_turn_possible(x_coord, y_coord, x_coord + offset_dx, y_coord - offset_dy):
                        cost = self.compute_safety_penalty(x_coord + offset_dx, y_coord - offset_dy)
                        next_states.append((x_coord + offset_dx, y_coord - offset_dy, intended_dir, cost, MovementType.REVERSE_OFFSET_RIGHT))

                    if self.grid.is_half_turn_possible(x_coord, y_coord, x_coord - offset_dx, y_coord - offset_dy):
                        cost = self.compute_safety_penalty(x_coord - offset_dx, y_coord - offset_dy)
                        next_states.append((x_coord - offset_dx, y_coord - offset_dy, intended_dir, cost, MovementType.REVERSE_OFFSET_LEFT))
                else:
                    if self.grid.is_half_turn_possible(x_coord, y_coord, x_coord + offset_dx, y_coord - offset_dy):
                        cost = self.compute_safety_penalty(x_coord + offset_dx, y_coord - offset_dy)
                        next_states.append((x_coord + offset_dx, y_coord - offset_dy, intended_dir, cost, MovementType.FORWARD_OFFSET_RIGHT))

                    if self.grid.is_half_turn_possible(x_coord, y_coord, x_coord + offset_dx, y_coord + offset_dy):
                        cost = self.compute_safety_penalty(x_coord + offset_dx, y_coord + offset_dy)
                        next_states.append((x_coord + offset_dx, y_coord + offset_dy, intended_dir, cost, MovementType.FORWARD_OFFSET_LEFT))

                    if self.grid.is_half_turn_possible(x_coord, y_coord, x_coord - offset_dx, y_coord - offset_dy):
                        cost = self.compute_safety_penalty(x_coord - offset_dx, y_coord - offset_dy)
                        next_states.append((x_coord - offset_dx, y_coord - offset_dy, intended_dir, cost, MovementType.REVERSE_OFFSET_RIGHT))

                    if self.grid.is_half_turn_possible(x_coord, y_coord, x_coord - offset_dx, y_coord + offset_dy):
                        cost = self.compute_safety_penalty(x_coord - offset_dx, y_coord + offset_dy)
                        next_states.append((x_coord - offset_dx, y_coord + offset_dy, intended_dir, cost, MovementType.REVERSE_OFFSET_LEFT))

            else:
                turn_dx, turn_dy = FULL_TURN_RADIUS
                delta_rl_x, delta_rl_y = 4, 2

                if curr_dir == Heading.NORTH and intended_dir == Heading.EAST:
                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord + turn_dx, y_coord + turn_dy, curr_dir):
                        cost = self.compute_safety_penalty(x_coord + turn_dx, y_coord + turn_dy)
                        next_states.append((x_coord + turn_dx, y_coord + turn_dy, intended_dir, cost + 10, MovementType.FORWARD_RIGHT_TURN))

                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord - delta_rl_y, y_coord - delta_rl_x, curr_dir):
                        cost = self.compute_safety_penalty(x_coord - delta_rl_y, y_coord - delta_rl_x)
                        next_states.append((x_coord - delta_rl_y, y_coord - delta_rl_x, intended_dir, cost + 10, MovementType.REVERSE_LEFT_TURN))

                if curr_dir == Heading.EAST and intended_dir == Heading.NORTH:
                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord + turn_dy, y_coord + turn_dx, curr_dir):
                        cost = self.compute_safety_penalty(x_coord + turn_dy, y_coord + turn_dx)
                        next_states.append((x_coord + turn_dy, y_coord + turn_dx, intended_dir, cost + 10, MovementType.FORWARD_LEFT_TURN))

                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord - turn_dx, y_coord - turn_dy, curr_dir):
                        cost = self.compute_safety_penalty(x_coord - turn_dx, y_coord - turn_dy)
                        next_states.append((x_coord - turn_dx, y_coord - turn_dy, intended_dir, cost + 10, MovementType.REVERSE_RIGHT_TURN))

                if curr_dir == Heading.EAST and intended_dir == Heading.SOUTH:
                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord + turn_dy, y_coord - turn_dx, curr_dir):
                        cost = self.compute_safety_penalty(x_coord + turn_dy, y_coord - turn_dx)
                        next_states.append((x_coord + turn_dy, y_coord - turn_dx, intended_dir, cost + 10, MovementType.FORWARD_RIGHT_TURN))

                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord - delta_rl_x, y_coord + delta_rl_y, curr_dir):
                        cost = self.compute_safety_penalty(x_coord - delta_rl_x, y_coord + delta_rl_y)
                        next_states.append((x_coord - delta_rl_x, y_coord + delta_rl_y, intended_dir, cost + 10, MovementType.REVERSE_LEFT_TURN))

                if curr_dir == Heading.SOUTH and intended_dir == Heading.EAST:
                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord + turn_dx, y_coord - turn_dy, curr_dir):
                        cost = self.compute_safety_penalty(x_coord + turn_dx, y_coord - turn_dy)
                        next_states.append((x_coord + turn_dx, y_coord - turn_dy, intended_dir, cost + 10, MovementType.FORWARD_LEFT_TURN))

                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord - turn_dy, y_coord + turn_dx, curr_dir):
                        cost = self.compute_safety_penalty(x_coord - turn_dy, y_coord + turn_dx)
                        next_states.append((x_coord - turn_dy, y_coord + turn_dx, intended_dir, cost + 10, MovementType.REVERSE_RIGHT_TURN))

                if curr_dir == Heading.SOUTH and intended_dir == Heading.WEST:
                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord - turn_dx, y_coord - turn_dy, curr_dir):
                        cost = self.compute_safety_penalty(x_coord - turn_dx, y_coord - turn_dy)
                        next_states.append((x_coord - turn_dx, y_coord - turn_dy, intended_dir, cost + 10, MovementType.FORWARD_RIGHT_TURN))

                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord + delta_rl_y, y_coord + delta_rl_x, curr_dir):
                        cost = self.compute_safety_penalty(x_coord + delta_rl_y, y_coord + delta_rl_x)
                        next_states.append((x_coord + delta_rl_y, y_coord + delta_rl_x, intended_dir, cost + 10, MovementType.REVERSE_LEFT_TURN))

                if curr_dir == Heading.WEST and intended_dir == Heading.SOUTH:
                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord - turn_dy, y_coord - turn_dx, curr_dir):
                        cost = self.compute_safety_penalty(x_coord - turn_dy, y_coord - turn_dx)
                        next_states.append((x_coord - turn_dy, y_coord - turn_dx, intended_dir, cost + 10, MovementType.FORWARD_LEFT_TURN))

                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord + turn_dx, y_coord + turn_dy, curr_dir):
                        cost = self.compute_safety_penalty(x_coord + turn_dx, y_coord + turn_dy)
                        next_states.append((x_coord + turn_dx, y_coord + turn_dy, intended_dir, cost + 10, MovementType.REVERSE_RIGHT_TURN))

                if curr_dir == Heading.WEST and intended_dir == Heading.NORTH:
                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord - turn_dy, y_coord + turn_dx, curr_dir):
                        cost = self.compute_safety_penalty(x_coord - turn_dy, y_coord + turn_dx)
                        next_states.append((x_coord - turn_dy, y_coord + turn_dx, intended_dir, cost + 10, MovementType.FORWARD_RIGHT_TURN))

                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord + delta_rl_x, y_coord - delta_rl_y, curr_dir):
                        cost = self.compute_safety_penalty(x_coord + delta_rl_x, y_coord - delta_rl_y)
                        next_states.append((x_coord + delta_rl_x, y_coord - delta_rl_y, intended_dir, cost + 10, MovementType.REVERSE_LEFT_TURN))

                if curr_dir == Heading.NORTH and intended_dir == Heading.WEST:
                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord - turn_dx, y_coord + turn_dy, curr_dir):
                        cost = self.compute_safety_penalty(x_coord - turn_dx, y_coord + turn_dy)
                        next_states.append((x_coord - turn_dx, y_coord + turn_dy, intended_dir, cost + 10, MovementType.FORWARD_LEFT_TURN))

                    if self.grid.is_turn_possible(x_coord, y_coord, x_coord + turn_dy, y_coord - turn_dx, curr_dir):
                        cost = self.compute_safety_penalty(x_coord + turn_dy, y_coord - turn_dx)
                        next_states.append((x_coord + turn_dy, y_coord - turn_dx, intended_dir, cost + 10, MovementType.REVERSE_RIGHT_TURN))

        return next_states

    def store_computed_path(self, start_state: GridState, goal_state: GridState, parent_map: dict, path_cost: int):
        self.cost_table[(start_state, goal_state)] = path_cost
        self.cost_table[(goal_state, start_state)] = path_cost

        path_trace = []
        trace_pointer = (goal_state.pos_x, goal_state.pos_y, goal_state.facing_direction)
        while trace_pointer in parent_map:
            path_trace.append(trace_pointer)
            trace_pointer = parent_map[trace_pointer]
        path_trace.append(trace_pointer)

        self.path_table[(start_state, goal_state)] = path_trace[::-1]
        self.path_table[(goal_state, start_state)] = path_trace

    @staticmethod
    def generate_visit_flags(num_bits):
        max_length = bin(2**num_bits - 1).count("1")
        bit_strings = [bin(i)[2:].zfill(max_length) for i in range(2**num_bits)]
        bit_strings.sort(key=lambda s: s.count("1"), reverse=True)
        return bit_strings

    @staticmethod
    def estimate_dist(start_state: GridState, goal_state: GridState, level=0) -> int:
        dx = start_state.pos_x - goal_state.pos_x
        dy = start_state.pos_y - goal_state.pos_y
        if level == 1:
            return math.sqrt(dx**2 + dy**2)
        return abs(dx) + abs(dy)

    @staticmethod
    def compute_half_turn_shift(facing: Heading):
        if facing == Heading.NORTH:
            return HALF_TURN_RADIUS[1], HALF_TURN_RADIUS[0]
        elif facing == Heading.SOUTH:
            return -HALF_TURN_RADIUS[1], -HALF_TURN_RADIUS[0]
        elif facing == Heading.EAST:
            return HALF_TURN_RADIUS[0], HALF_TURN_RADIUS[1]
        elif facing == Heading.WEST:
            return -HALF_TURN_RADIUS[0], -HALF_TURN_RADIUS[1]
        else:
            raise ValueError(f"Invalid direction {facing}. This should never happen.")

    @staticmethod
    def capture_position_relative(cell: GridState, obs: Object) -> str:
        pos_x, pos_y, heading = cell.pos_x, cell.pos_y, cell.facing_direction
        obs_x, obs_y = obs.pos_x, obs.pos_y

        if heading == Heading.NORTH:
            if obs_x == pos_x and obs_y > pos_y:
                return "C"
            elif obs_x < pos_x:
                return "L"
            else:
                return "R"
        elif heading == Heading.SOUTH:
            if obs_x == pos_x and obs_y < pos_y:
                return "C"
            elif obs_x < pos_x:
                return "R"
            else:
                return "L"
        elif heading == Heading.EAST:
            if obs_y == pos_y and obs_x > pos_x:
                return "C"
            elif obs_y < pos_y:
                return "R"
            else:
                return "L"
        elif heading == Heading.WEST:
            if obs_y == pos_y and obs_x < pos_x:
                return "C"
            elif obs_y < pos_y:
                return "L"
            else:
                return "R"
        else:
            raise ValueError(f"Invalid direction {heading}.")

    @staticmethod
    def build_combinations(view_sets, idx: int, temp: list, final: list, max_iter: int):
        if idx == len(view_sets):
            final.append(temp.copy())
            return final
        if max_iter == 0:
            return final
        max_iter -= 1
        for j in range(len(view_sets[idx])):
            temp.append(j)
            final = PathPlanner.build_combinations(view_sets, idx + 1, temp, final, max_iter)
            temp.pop()
        return final

    def convert_path_to_motion(self, path):
        motion_list = []
        capture_ids = []

        for i in range(len(path) - 1):
            curr = path[i]
            nxt = path[i + 1]

            x1, y1, d1 = curr.pos_x, curr.pos_y, curr.facing_direction
            x2, y2, d2 = nxt.pos_x, nxt.pos_y, nxt.facing_direction

            if (x2, y2, d2, x1, y1, d1) in self.motion_table:
                motion = self.motion_table[(x2, y2, d2, x1, y1, d1)].get_opposite_movement()
            elif (x1, y1, d1, x2, y2, d2) in self.motion_table:
                motion = self.motion_table[(x1, y1, d1, x2, y2, d2)]
            else:
                raise ValueError(f"Invalid path from {curr} to {nxt}.")

            motion_list.append(motion)

            if nxt.target_obstacle_ids:
                for obs_id in nxt.target_obstacle_ids:
                    motion_list.append(MovementType.CAPTURE)
                    capture_ids.append(obs_id)

        return motion_list, capture_ids