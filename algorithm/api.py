from flask import Flask, request, jsonify
from Algorithm.pathplanner import PathPlanner
from Components.grid import Object
from Helpers.motion import Heading, RobotCommand
from Algorithm.Simulator import Simulator
from Helpers.motion import convert_command_list
import time

app = Flask(__name__)

sim = Simulator(
    grid_size_x=20,
    grid_size_y=20,
    robot_x=1,
    robot_y=1,
    robot_direction=Heading.NORTH,
)

server_start_time = time.time()

last_generated_path = None
last_visited_order = None
last_commands = None

def str_to_direction(direction_str):
    direction_map = {
        "NORTH": Heading.NORTH,
        "SOUTH": Heading.SOUTH,
        "EAST": Heading.EAST,
        "WEST": Heading.WEST,
    }
    return direction_map.get(direction_str.upper())

@app.route('/generate_path', methods=['POST'])
def generate_path():
    global last_generated_path, last_visited_order

    data = request.json
    obstacles_data = data.get('obstacles', [])

    print(f"Received {len(obstacles_data)} obstacles for path computation.")

    sim.clear_obs()
    for obs in obstacles_data:
        direction = str_to_direction(obs['direction'])
        if direction is None:
            print(f"Invalid direction: {obs['direction']}")
            return jsonify({"error": f"Invalid direction: {obs['direction']}"}), 400
        sim.add_object([(obs['x'], obs['y'], direction, obs['id'])])

    start_time = time.time()
    print("Starting path computation...")

    last_generated_path, cost = sim.maze_solver.get_best_path()

    elapsed_time = round(time.time() - start_time, 3)
    print(f"Path Computation Time: {elapsed_time} seconds")

    if not last_generated_path:
        print("ERROR: Path computation returned an empty path.")
        return jsonify({"error": "Failed to compute a valid path"}), 500

    print(f"Computed path: {last_generated_path}")

    # Extract visitation order
    visited_obstacle_order = []
    for cell in last_generated_path:
        for screenshot_id in cell.screenshot_id:
            obstacle_id = screenshot_id.split('_')[0]
            if obstacle_id not in visited_obstacle_order:
                visited_obstacle_order.append(obstacle_id)

    last_visited_order = visited_obstacle_order

    sim.show_custom_path_animation(last_generated_path)

    path_data = [{"x": cell.x, "y": cell.y, "direction": cell.direction.name} for cell in last_generated_path]

    return jsonify({
        "optimal_path": path_data,
        "cost": cost,
        "visited_obstacle_order": visited_obstacle_order
    })

@app.route('/commands', methods=['POST'])
def compile_command_list():
    global last_generated_path, last_commands

    if last_generated_path is None:
        return jsonify({"error": "No path available. Please call /generate_path first."}), 400

    motions, obstacle_ids = sim.maze_solver.convert_path_to_motion(last_generated_path)

    command_generator = RobotCommand()
    commands = command_generator.compile_command_list(motions, obstacle_ids)
    new_commands = convert_command_list(commands)

    last_commands = new_commands

    last_commands = ["Z0000000"] + last_commands

    print("Default Commands: ", commands)
    print("New Commands: ", new_commands)
    print("obstacle order: ", obstacle_ids)

    return jsonify({
        "commands": new_commands,
        "visited_obstacle_order": obstacle_ids
    })

@app.route('/status', methods=['GET'])
def status():
    uptime = round(time.time() - server_start_time, 2)
    obstacle_count = len(sim.maze_solver.grid.list_obs())

    response = {
        "status": "running",
        "uptime_seconds": uptime,
        "api_version": "1.0.0",
        "obstacles_loaded": obstacle_count,
        "last_generated_path": bool(last_generated_path),
        "last_visited_order": last_visited_order if last_visited_order else [],
        "last_commands_generated": bool(last_commands)
    }
    return jsonify(response)

@app.route('/plot', methods=['GET'])
def plot_animation():
    optimal_path, _ = sim.maze_solver.get_best_path()
    sim.show_custom_path_animation(optimal_path)
    return jsonify({"message": "Animation plotted and saved."})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
