from Algorithm.pathplanner import PathPlanner
from Components.grid import Object
from Helpers.motion import Heading, RobotCommand, MovementType
from Algorithm.Simulator import Simulator
from Helpers.motion import convert_command_list

simulator = Simulator(
    grid_width=20,
    grid_height=20,
    start_x=1,
    start_y=1,
    start_direction=Heading.NORTH,
)


simulator.stop_debug()

# simulator.load_object(0)  # load obstacles from file

#simulator.generate_random_object(8)  # uncomment to generate random obstacles

"""
obstacles = [
    (0, 17, Heading.EAST, 1),
    (5, 12, Heading.SOUTH, 2),
    (7, 5, Heading.NORTH, 3),
    (15, 2, Heading.WEST, 4),
    (15, 14, Heading.EAST, 5),
]
"""

obstacles = [(1, 16, Heading.EAST, 1),
             (5, 12, Heading.SOUTH, 2),
             (8, 5, Heading.NORTH, 3),
             (12, 14, Heading.EAST, 4),
             (15, 2, Heading.WEST, 5),
             (16, 19, Heading.SOUTH, 6),
             (19, 9, Heading.WEST, 7)]

simulator.add_object(obstacles)

best_path, cost = simulator.maze_solver.get_best_path()


motion_sequence, obstacle_ids = simulator.maze_solver.convert_path_to_motion(best_path)
robot_command = RobotCommand()
raw_commands = robot_command.compile_command_list(motion_sequence, obstacle_ids)
new_commands = convert_command_list(raw_commands)

print("Optimal Path: ", best_path)

print("Motion Path: ", motion_sequence)

print("Obstacle IDs: ", obstacle_ids)

print("Commands: ", raw_commands)

print("New Commands: ", new_commands)

simulator.show_custom_path_animation(best_path)