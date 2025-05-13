from Algorithm.pathplanner import PathPlanner
from Components.grid import GridState, Object
from Helpers.motion import Heading, MovementType

import os
import json
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
import numpy as np

from typing import List, Tuple, Literal


class Simulator:

    debug_file = os.path.join(os.path.dirname(__file__), "../debug", "obstacles.json")
    debug = False
    debug_save = 0

    def __init__(
        self,
        maze_solver: PathPlanner = None,
        grid_width: int = None,
        grid_height: int = None,
        start_x: int = None,
        start_y: int = None,
        start_direction: int = None,
    ):
        self.maze_solver = (
            maze_solver
            if maze_solver
            else PathPlanner(
                grid_width=grid_width,
                grid_height=grid_height,
                start_x=start_x,
                start_y=start_y,
                start_facing=start_direction,
            )
        )

    def add_object(self, obstacle_list):
        for obs in obstacle_list:
            self.maze_solver.insert_obs(*obs)

        if self.debug:
            print(
                f"Debug mode enabled. Storing obstacles to file {os.path.realpath(self.debug_file)}"
            )
            self.save_objects(obstacle_list, save_number=self.debug_save)

    def reset_object(self):
        self.maze_solver.reset_object()

    def load_object(self, slot=0):
        if slot == 0:
            loaded_data = self.load_objects(option="last")["last"]
        elif slot in [1, 2, 3]:
            loaded_data = self.load_objects(option=f"{slot}")[f"{slot}"]
        else:
            return

        for entry in loaded_data:
            x, y, facing, obs_id = entry["x"], entry["y"], entry["direction"], entry["id"]
            self.maze_solver.insert_obs(x, y, facing, obs_id)

        return loaded_data

    def generate_random_object(self, num_obstacles: int):
        grid_width = self.maze_solver.grid.size_x
        grid_height = self.maze_solver.grid.size_y
        padding = 2
        initial_state = self.maze_solver.robot.start_state()
        current_obstacles = self.maze_solver.grid.list_obs()

        obstacle_ids = [int(obs.get_obstacle_id()) for obs in current_obstacles]
        last_obstacle_id = max(obstacle_ids) if obstacle_ids else 0

        blocked_area = self.get_restricted_zone(
            [(initial_state.pos_x, initial_state.pos_y)], padding=padding
        )

        new_obstacles = []
        for i in range(num_obstacles):
            rand_x = random.randint(0 + padding, grid_width - 1 - padding)
            rand_y = random.randint(0 + padding, grid_height - 1 - padding)

            direction = None
            while (rand_x, rand_y) in blocked_area or direction is None:
                rand_x = random.randint(0 + padding, grid_width - 1 - padding)
                rand_y = random.randint(0 + padding, grid_height - 1 - padding)
                if (rand_x, rand_y) in blocked_area:
                    continue

                direction = self.choose_viable_direction(
                    rand_x, rand_y, current_obstacles
                )

            blocked_area.update(self.get_restricted_zone([(rand_x, rand_y)]))

            obs_id = last_obstacle_id + i + 1

            new_obstacles.append((rand_x, rand_y, direction, obs_id))
            self.maze_solver.insert_obs(rand_x, rand_y, direction, obs_id)

        if self.debug:
            print(
                f"Debug mode enabled. Storing obstacles to file {os.path.realpath(self.debug_file)}"
            )
            self.save_objects(new_obstacles, save_number=self.debug_save)

        return new_obstacles

    def get_best_path(self):
        print("Calculating optimal path...")
        return self.maze_solver.get_best_path(False)

    def show_custom_path_animation(self, path_sequence, verbose=False):
        num_interp_points = 3
        start_state = self.maze_solver.robot.start_state()
        obstacle_list = self.maze_solver.grid.list_obs()

        print("Plotting optimal path animation")
        fig, ax = plt.subplots()

        obs_x, obs_y = [[], [], [], []], [[], [], [], []]
        markers = ["^", ">", "v", "<"]

        for obs in obstacle_list:
            idx = markers.index(self.direction_symbol(obs.facing_direction))
            obs_x[idx].append(obs.pos_x)
            obs_y[idx].append(obs.pos_y)

        path_x, path_y = [[], [], [], []], [[], [], [], []]
        screenshot_x, screenshot_y = [], []
        angle_x, angle_y = [[], [], [], []], [[], [], [], []]
        marker_angles = [45, 135, 225, 315]

        prev = start_state
        timestamp = 0
        for step in path_sequence:
            if abs(prev.pos_x - step.pos_x) > 1 or abs(prev.pos_y - step.pos_y) > 1:
                turn_angle = self.compute_turn_angle(prev.facing_direction, step.facing_direction)
                if turn_angle is None:
                    raise ValueError(f"Invalid turn from {prev.facing_direction} to {step.facing_direction} at ({prev.pos_x}, {prev.pos_y})")
                elif turn_angle == 0:
                    turn_angle = self.determine_half_turn_angle(prev.pos_x, prev.pos_y, step.pos_x, step.pos_y, prev.facing_direction)

                angle_idx = marker_angles.index(turn_angle)
                dx = step.pos_x - prev.pos_x
                dy = step.pos_y - prev.pos_y
                for i in range(1, 1 + num_interp_points):
                    angle_x[angle_idx].append((prev.pos_x + i * dx / (1 + num_interp_points), timestamp))
                    angle_y[angle_idx].append((prev.pos_y + i * dy / (1 + num_interp_points), timestamp))
                    timestamp += 1

            if step.target_obstacle_ids:
                screenshot_x.append((step.pos_x, timestamp))
                screenshot_y.append((step.pos_y, timestamp))

            idx = markers.index(self.direction_symbol(step.facing_direction))
            path_x[idx].append((step.pos_x, timestamp))
            path_y[idx].append((step.pos_y, timestamp))
            prev = step
            timestamp += 1

        ax.set(xlim=(0, 20), ylim=(0, 20))
        ax.set_xticks(range(0, 21))
        ax.set_yticks(range(0, 21))

        def update(frame_num):
            ax.clear()
            ax.set(xlim=(-1, 20), ylim=(-1, 20))
            ax.set_xticks(range(0, 21))
            ax.set_yticks(range(0, 21))
            ax.grid()

            # Draw robot's initial state
            ax.scatter(
                start_state.pos_x,
                start_state.pos_y,
                marker=self.direction_symbol(start_state.facing_direction),
                color="red",
                s=100,
            )

            trail_x, trail_y = [[], [], [], []], [[], [], [], []]
            turn_x, turn_y = [[], [], [], []], [[], [], [], []]

            for i in range(4):
                path_steps_x = path_x[i]
                path_steps_y = path_y[i]
                angle_steps_x = angle_x[i]
                angle_steps_y = angle_y[i]

                for k in range(len(path_steps_x)):
                    pos_x, t = path_steps_x[k]
                    pos_y, _ = path_steps_y[k]
                    if frame_num - 2 <= t <= frame_num:
                        trail_x[i].append(pos_x)
                        trail_y[i].append(pos_y)

                for k in range(len(angle_steps_x)):
                    angle_pos_x, t = angle_steps_x[k]
                    angle_pos_y, _ = angle_steps_y[k]
                    if frame_num - 2 <= t <= frame_num:
                        turn_x[i].append(angle_pos_x)
                        turn_y[i].append(angle_pos_y)

            snap_x = [x for x, t in screenshot_x if frame_num > t]
            snap_y = [y for y, t in screenshot_y if frame_num > t]

            if screenshot_x and screenshot_y:
                ax.scatter(snap_x, snap_y, color="green", s=100, marker="*")

            for i in range(len(markers)):
                if obs_x[i] and obs_y[i]:
                    ax.scatter(obs_x[i], obs_y[i], marker=markers[i], color="black", s=300)
                if trail_x[i] and trail_y[i]:
                    ax.scatter(trail_x[i], trail_y[i], marker=markers[i], color="blue", s=80)
                if turn_x[i] and turn_y[i]:
                    ax.scatter(turn_x[i], turn_y[i], marker=(3, 0, marker_angles[i]), color="green", s=120)

        if verbose:
            print("Animating optimal path...")

        ani = animation.FuncAnimation(fig, update, frames=timestamp + 5, interval=300)
        output_path = os.path.realpath(os.path.join(os.path.dirname(__file__), "../Animations", "robot_animation.gif"))

        print(f"Saving animation to {output_path}")
        ani.save(output_path, writer="pillow")

    def show_optimal_path_animation(self, verbose=False):
        num_interp_points = 3
        start_state = self.maze_solver.robot.start_state()
        obstacle_list = self.maze_solver.grid.list_obs()

        print("Calculating optimal path...")
        final_path, total_cost = self.maze_solver.get_best_path(False)

        if verbose:
            print(f"Optimal path with cost = {total_cost} calculated: ")
            for step in final_path:
                print(f"({step.pos_x}, {step.pos_y}) {step.facing_direction.name}", end=" ->  ") if step != final_path[-1] else print(f"({step.pos_x}, {step.pos_y}) {step.facing_direction.name}")

        print("Plotting optimal path animation")
        fig, ax = plt.subplots()

        obs_x, obs_y = [[], [], [], []], [[], [], [], []]
        markers = ["^", ">", "v", "<"]

        for obs in obstacle_list:
            idx = markers.index(self.direction_symbol(obs.facing_direction))
            obs_x[idx].append(obs.pos_x)
            obs_y[idx].append(obs.pos_y)

        path_x, path_y = [[], [], [], []], [[], [], [], []]
        screenshot_x, screenshot_y = [], []
        angle_x, angle_y = [[], [], [], []], [[], [], [], []]
        marker_angles = [45, 135, 225, 315]

        prev = start_state
        timestamp = 0
        for step in final_path:
            if abs(prev.pos_x - step.pos_x) > 1 or abs(prev.pos_y - step.pos_y) > 1:
                turn_angle = self.compute_turn_angle(prev.facing_direction, step.facing_direction)
                if turn_angle is None:
                    raise ValueError(f"Invalid turn from {prev.facing_direction} to {step.facing_direction} at ({prev.pos_x}, {prev.pos_y})")
                elif turn_angle == 0:
                    turn_angle = self.determine_half_turn_angle(prev.pos_x, prev.pos_y, step.pos_x, step.pos_y, prev.facing_direction)

                angle_idx = marker_angles.index(turn_angle)
                dx = step.pos_x - prev.pos_x
                dy = step.pos_y - prev.pos_y
                for i in range(1, 1 + num_interp_points):
                    angle_x[angle_idx].append((prev.pos_x + i * dx / (1 + num_interp_points), timestamp))
                    angle_y[angle_idx].append((prev.pos_y + i * dy / (1 + num_interp_points), timestamp))
                    timestamp += 1

            if step.target_obstacle_ids:
                screenshot_x.append((step.pos_x, timestamp))
                screenshot_y.append((step.pos_y, timestamp))

            idx = markers.index(self.direction_symbol(step.facing_direction))
            path_x[idx].append((step.pos_x, timestamp))
            path_y[idx].append((step.pos_y, timestamp))
            prev = step
            timestamp += 1

        ax.set(xlim=(0, 20), ylim=(0, 20))
        ax.set_xticks(range(0, 21))
        ax.set_yticks(range(0, 21))

        def update(frame):
            ax.clear()
            ax.set(xlim=(0, 20), ylim=(0, 20))
            ax.set_xticks(range(0, 21))
            ax.set_yticks(range(0, 21))
            ax.grid()

            ax.scatter(start_state.pos_x, start_state.pos_y, marker=self.direction_symbol(start_state.facing_direction), color="red", s=100)

            dx, dy = [[], [], [], []], [[], [], [], []]
            dx_turn, dy_turn = [[], [], [], []], [[], [], [], []]

            for j in range(4):
                for k, (x, t) in enumerate(path_x[j]):
                    if frame - 2 <= t <= frame:
                        dx[j].append(x)
                        dy[j].append(path_y[j][k][0])

                for k, (x, t) in enumerate(angle_x[j]):
                    if frame - 2 <= t <= frame:
                        dx_turn[j].append(x)
                        dy_turn[j].append(angle_y[j][k][0])

            snap_x = [x for x, t in screenshot_x if frame > t]
            snap_y = [y for y, t in screenshot_y if frame > t]

            if screenshot_x and screenshot_y:
                ax.scatter(snap_x, snap_y, color="green", s=100, marker="*")

            for j in range(4):
                if obs_x[j] and obs_y[j]:
                    ax.scatter(obs_x[j], obs_y[j], marker=markers[j], color="black", s=300)
                if dx[j] and dy[j]:
                    ax.scatter(dx[j], dy[j], marker=markers[j], color="blue", s=80)
                if dx_turn[j] and dy_turn[j]:
                    ax.scatter(dx_turn[j], dy_turn[j], marker=(3, 0, marker_angles[j]), color="green", s=120)

        if verbose:
            print("Animating optimal path...")

        ani = animation.FuncAnimation(fig, update, frames=timestamp + 5, interval=300)
        output_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "../Animations", "robot_animation.gif"))

        print(f"Saving animation to {output_file}")
        ani.save(output_file, writer="pillow")

        return final_path, total_cost

    def start_debug(self, save_number=0):
        self.debug_save = save_number
        self.debug = True
        if not os.path.exists(os.path.dirname(self.debug_file)):
            os.makedirs(os.path.dirname(self.debug_file))
        if not os.path.exists(self.debug_file):
            with open(self.debug_file, "w") as f:
                json.dump({"save_1": [], "save_2": [], "save_3": [], "last": []}, f)

    def stop_debug(self):
        self.debug = False
        self.debug_save = 0

    def load_objects(self, option="last") -> dict:
        try:
            with open(self.debug_file, "r") as f:
                all_obstacle_data = json.load(f)
        except IOError:
            return {"save_1": [], "save_2": [], "save_3": [], "last": []}

        if option == "all":
            return all_obstacle_data
        else:
            return {f"{option}": all_obstacle_data[option]}

    def save_objects(self, obstacle_list, save_slot=0):
        existing_data = self.load_objects(option="all")
        serialized_list = []
        for obstacle in obstacle_list:
            obs_entry = {"x": obstacle[0], "y": obstacle[1], "direction": obstacle[2], "id": obstacle[3]}
            serialized_list.append(obs_entry)

        if save_slot == 0:
            pass
        elif save_slot in [1, 2, 3]:
            existing_data[f"{save_slot}"] = serialized_list
        else:
            return

        existing_data["last"] = serialized_list

        try:
            with open(self.debug_file, "w") as f:
                json.dump(existing_data, f, indent=4)
        except IOError:
            pass

    @staticmethod
    def direction_symbol(direction):
        if direction == Heading.NORTH:
            return "^"
        elif direction == Heading.EAST:
            return ">"
        elif direction == Heading.SOUTH:
            return "v"
        elif direction == Heading.WEST:
            return "<"

    @staticmethod
    def compute_turn_angle(current_dir, next_dir):
        if next_dir == current_dir:
            return 0

        if current_dir == Heading.NORTH:
            if next_dir == Heading.EAST:
                return 315
            elif next_dir == Heading.WEST:
                return 45
        elif current_dir == Heading.EAST:
            if next_dir == Heading.SOUTH:
                return 225
            elif next_dir == Heading.NORTH:
                return 315
        elif current_dir == Heading.SOUTH:
            if next_dir == Heading.WEST:
                return 135
            elif next_dir == Heading.EAST:
                return 225
        elif current_dir == Heading.WEST:
            if next_dir == Heading.NORTH:
                return 45
            elif next_dir == Heading.SOUTH:
                return 135

        return None
    
    def determine_half_turn_angle(self, cur_x: int, cur_y: int, target_x: int, target_y: int, facing: Heading) -> int:
        if facing == Heading.NORTH:
            if target_x > cur_x:
                if target_y > cur_y:
                    return self.compute_turn_angle(Heading.NORTH, Heading.EAST)
                elif target_y < cur_y:
                    return self.compute_turn_angle(Heading.NORTH, Heading.WEST)
            else:
                if target_y > cur_y:
                    return self.compute_turn_angle(Heading.NORTH, Heading.WEST)
                elif target_y < cur_y:
                    return self.compute_turn_angle(Heading.NORTH, Heading.EAST)

        elif facing == Heading.EAST:
            if target_y > cur_y:
                if target_x > cur_x:
                    return self.compute_turn_angle(Heading.EAST, Heading.SOUTH)
                elif target_x < cur_x:
                    return self.compute_turn_angle(Heading.EAST, Heading.NORTH)
            else:
                if target_x > cur_x:
                    return self.compute_turn_angle(Heading.EAST, Heading.NORTH)
                elif target_x < cur_x:
                    return self.compute_turn_angle(Heading.EAST, Heading.SOUTH)

        elif facing == Heading.SOUTH:
            if target_x > cur_x:
                if target_y > cur_y:
                    return self.compute_turn_angle(Heading.SOUTH, Heading.WEST)
                elif target_y < cur_y:
                    return self.compute_turn_angle(Heading.SOUTH, Heading.EAST)
            else:
                if target_y > cur_y:
                    return self.compute_turn_angle(Heading.SOUTH, Heading.EAST)
                elif target_y < cur_y:
                    return self.compute_turn_angle(Heading.SOUTH, Heading.WEST)

        else:  # WEST
            if target_y > cur_y:
                if target_x > cur_x:
                    return self.compute_turn_angle(Heading.WEST, Heading.NORTH)
                elif target_x < cur_x:
                    return self.compute_turn_angle(Heading.WEST, Heading.SOUTH)
            else:
                if target_x > cur_x:
                    return self.compute_turn_angle(Heading.WEST, Heading.SOUTH)
                elif target_x < cur_x:
                    return self.compute_turn_angle(Heading.WEST, Heading.NORTH)
        return None
    
    @staticmethod
    def get_restricted_zone(position_list, padding=1) -> set:
        forbidden_squares = set()
        for pos in position_list:
            x, y = pos
            for i in range(-padding, padding + 1):
                for j in range(-padding, padding + 1):
                    forbidden_squares.add((x + i, y + j))
        return forbidden_squares

    @staticmethod
    def is_approach_blocked(x: int, y: int, facing: Heading, obstacle_list: List[Object], padding: int = 5) -> bool:
        pad_half = padding // 2
        if facing == Heading.NORTH:
            for obs in obstacle_list:
                if x - pad_half < obs.pos_x < x + pad_half and y < obs.pos_y < y + padding:
                    return True
        elif facing == Heading.EAST:
            for obs in obstacle_list:
                if y - pad_half < obs.pos_y < y + pad_half and x < obs.pos_x < x + padding:
                    return True
        elif facing == Heading.SOUTH:
            for obs in obstacle_list:
                if x - pad_half < obs.pos_x < x + pad_half and y - padding < obs.pos_y < y:
                    return True
        elif facing == Heading.WEST:
            for obs in obstacle_list:
                if y - pad_half < obs.pos_y < y + pad_half and x - padding < obs.pos_x < x:
                    return True
        else:
            raise ValueError(f"Invalid direction {facing}")

        return False
    
    def is_location_unreachable(self, pos_x: int, pos_y: int, direction: Heading, margin: int = 6) -> bool:
        if direction == Heading.NORTH and pos_y > self.maze_solver.grid.size_y - margin:
            return True
        elif direction == Heading.EAST and pos_x > self.maze_solver.grid.size_x - margin:
            return True
        elif direction == Heading.SOUTH and pos_y < margin:
            return True
        elif direction == Heading.WEST and pos_x < margin:
            return True
        elif direction not in Heading:
            raise ValueError(f"Invalid direction {direction}")
        return False

    def choose_viable_direction(self, grid_x: int, grid_y: int, nearby_obstacles: List[Object]) -> Heading:
        viable_directions = []
        for heading in [Heading.NORTH, Heading.EAST, Heading.SOUTH, Heading.WEST]:
            if self.is_approach_blocked(grid_x, grid_y, heading, nearby_obstacles):
                continue
            if self.is_location_unreachable(grid_x, grid_y, heading):
                continue
            viable_directions.append(heading)

        if not viable_directions:
            return None
        return random.choice(viable_directions)