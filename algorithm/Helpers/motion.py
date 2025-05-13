from enum import Enum
import re

class Heading(int, Enum):

    NORTH = 0
    SOUTH = 1
    EAST = 2
    WEST = 3
    SKIP = 4

    def __int__(self):
        return self.value

    @staticmethod
    def calculate_turn_cost(start_direction, end_direction):
        if start_direction == Heading.NORTH:
            if end_direction in [Heading.EAST, Heading.WEST]:
                diff = 1
            elif end_direction == Heading.NORTH:
                return 0
            else:
                raise ValueError("Robot cannot turn from north to south")

        elif start_direction == Heading.SOUTH:
            if end_direction in [Heading.EAST, Heading.WEST]:
                diff = 1
            elif end_direction == Heading.SOUTH:
                return 0
            else:
                raise ValueError("Robot cannot turn from south to north")

        elif start_direction == Heading.EAST:
            if end_direction in [Heading.NORTH, Heading.SOUTH]:
                diff = 1
            elif end_direction == Heading.EAST:
                return 0
            else:
                raise ValueError("Robot cannot turn from east to west")

        elif start_direction == Heading.WEST:
            if end_direction in [Heading.NORTH, Heading.SOUTH]:
                diff = 1
            elif end_direction == Heading.WEST:
                return 0
            else:
                raise ValueError("Robot cannot turn from west to east")

        else:
            raise ValueError(f"direction {start_direction} is not a valid direction.")

        return diff

    def __repr__(self):
        return self.name

    def __str__(self):
        return self.name


class MovementType(int, Enum):

    FORWARD_LEFT_TURN = 0
    FORWARD_OFFSET_LEFT = 1
    FORWARD = 2
    FORWARD_OFFSET_RIGHT = 3
    FORWARD_RIGHT_TURN = 4

    REVERSE_LEFT_TURN = 10
    REVERSE_OFFSET_RIGHT = 9
    REVERSE = 8
    REVERSE_OFFSET_LEFT = 7
    REVERSE_RIGHT_TURN = 6

    CAPTURE = 1000

    def __int__(self):
        return self.value

    def __repr__(self):
        return self.name

    def __str__(self):
        return self.name

    def __eq__(self, other: "MovementType"):
        return self.value == other.value
    
    def can_combine(self):
        return self.value in [2, 8]

    def get_opposite_movement(self):
        if self == MovementType.CAPTURE:
            return MovementType.CAPTURE

        opp_val = 10 - self.value
        if opp_val == 5 or opp_val < 0 or opp_val > 10:
            raise ValueError(f"Invalid motion {self}. This should never happen.")

        return MovementType(opp_val)
    
    def get_half_turn_cost(self):
        if self in [
            MovementType.FORWARD_OFFSET_LEFT,
            MovementType.FORWARD_OFFSET_RIGHT,
            MovementType.REVERSE_OFFSET_LEFT,
            MovementType.REVERSE_OFFSET_RIGHT,
        ]:
            return 1
        else:
            return 0

    def get_reverse_cost(self):
        if self == MovementType.CAPTURE:
            raise ValueError("Capture motion does not have a reverse cost")

        if self in [
            MovementType.REVERSE_OFFSET_RIGHT,
            MovementType.REVERSE_OFFSET_LEFT,
            MovementType.REVERSE_LEFT_TURN,
            MovementType.REVERSE_RIGHT_TURN,
            MovementType.REVERSE,
        ]:
            return 1
        else:
            return 0


class RobotCommand:

    SEP = "|"
    END = ""
    RCV = "r"
    FIN = "FIN"
    INFO_MARKER = "M"
    INFO_DIST = "D"

    FORWARD_DIST_TARGET = "T"
    FORWARD_DIST_AWAY = "W"
    BACKWARD_DIST_TARGET = "t"
    BACKWARD_DIST_AWAY = "w"

    FORWARD_IR_DIST_L = "L"
    FORWARD_IR_DIST_R = "R"
    BACKWARD_IR_DIST_L = "l"
    BACKWARD_IR_DIST_R = "r"

    UNIT_DIST = 10

    FORWARD_TURN_ANGLE_LEFT = 25
    FORWARD_TURN_ANGLE_RIGHT = 25
    BACKWARD_TURN_ANGLE_LEFT = 25
    BACKWARD_TURN_ANGLE_RIGHT = 25

    FORWARD_RIGHT_FINAL_ANGLE = 86
    FORWARD_LEFT_FINAL_ANGLE = 87
    BACKWARD_RIGHT_FINAL_ANGLE = 89
    BACKWARD_LEFT_FINAL_ANGLE = 88

    def __init__(self, straight_speed: int = 50, turn_speed: int = 50):
        self.straight_speed = straight_speed
        self.turn_speed = turn_speed

    def create_command(self, motion: MovementType, num_motions: int = 1):
        if num_motions > 1:
            dist = num_motions * self.UNIT_DIST
        else:
            dist = self.UNIT_DIST

        if motion == MovementType.FORWARD:
            return [
                f"{self.FORWARD_DIST_TARGET}{self.straight_speed}{self.SEP}0{self.SEP}{dist}{self.END}"
            ]
        elif motion == MovementType.REVERSE:
            return [
                f"{self.BACKWARD_DIST_TARGET}{self.straight_speed}{self.SEP}0{self.SEP}{dist}{self.END}"
            ]
        elif motion == MovementType.FORWARD_LEFT_TURN:
            cmd1 = f"{self.FORWARD_DIST_TARGET}{self.turn_speed}{self.SEP}-{self.FORWARD_TURN_ANGLE_LEFT}{self.SEP}{self.FORWARD_LEFT_FINAL_ANGLE}{self.END}"
            cmd2 = f"{self.FORWARD_DIST_TARGET}{self.straight_speed}{self.SEP}0{self.SEP}{6}{self.END}"
            return [cmd1]

        elif motion == MovementType.FORWARD_RIGHT_TURN:
            cmd1 = f"{self.FORWARD_DIST_TARGET}{self.straight_speed}{self.SEP}0{self.SEP}{5}{self.END}"
            cmd2 = f"{self.FORWARD_DIST_TARGET}{self.turn_speed}{self.SEP}{self.FORWARD_TURN_ANGLE_RIGHT}{self.SEP}{self.FORWARD_RIGHT_FINAL_ANGLE}{self.END}"
            cmd3 = f"{self.FORWARD_DIST_TARGET}{self.straight_speed}{self.SEP}0{self.SEP}{12}{self.END}"
            return [cmd2]

        elif motion == MovementType.REVERSE_LEFT_TURN:
            cmd1 = f"{self.BACKWARD_DIST_TARGET}{self.straight_speed}{self.SEP}0{self.SEP}{6}{self.END}"
            cmd2 = f"{self.BACKWARD_DIST_TARGET}{self.turn_speed}{self.SEP}-{self.BACKWARD_TURN_ANGLE_LEFT}{self.SEP}{self.BACKWARD_LEFT_FINAL_ANGLE}{self.END}"
            return [cmd2]

        elif motion == MovementType.REVERSE_RIGHT_TURN:
            cmd1 = f"{self.BACKWARD_DIST_TARGET}{self.straight_speed}{self.SEP}0{self.SEP}{7}{self.END}"
            cmd2 = f"{self.BACKWARD_DIST_TARGET}{self.turn_speed}{self.SEP}{self.BACKWARD_TURN_ANGLE_RIGHT}{self.SEP}{self.BACKWARD_RIGHT_FINAL_ANGLE}{self.END}"
            cmd3 = f"{self.BACKWARD_DIST_TARGET}{self.straight_speed}{self.SEP}0{self.SEP}{4}{self.END}"
            return [cmd2]

        elif motion == MovementType.FORWARD_OFFSET_LEFT:
            cmd1 = f"{self.FORWARD_DIST_TARGET}{self.turn_speed}{self.SEP}{-14}{self.SEP}{21}{self.END}"
            cmd2 = f"{self.FORWARD_DIST_TARGET}{self.turn_speed}{self.SEP}{17}{self.SEP}{21}{self.END}"
        elif motion == MovementType.FORWARD_OFFSET_RIGHT:
            cmd1 = f"{self.FORWARD_DIST_TARGET}{self.turn_speed}{self.SEP}{14}{self.SEP}{21}{self.END}"
            cmd2 = f"{self.FORWARD_DIST_TARGET}{self.straight_speed}{self.SEP}{-17}{self.SEP}{21}{self.END}"
        elif motion == MovementType.REVERSE_OFFSET_LEFT:
            cmd1 = f"{self.BACKWARD_DIST_TARGET}{self.turn_speed}{self.SEP}{-15}{self.SEP}{24}{self.END}"
            cmd2 = f"{self.BACKWARD_DIST_TARGET}{self.turn_speed}{self.SEP}{25}{self.SEP}{25}{self.END}"
        elif motion == MovementType.REVERSE_OFFSET_RIGHT:
            cmd1 = f"{self.BACKWARD_DIST_TARGET}{self.turn_speed}{self.SEP}{15}{self.SEP}{24}{self.END}"
            cmd2 = f"{self.BACKWARD_DIST_TARGET}{self.turn_speed}{self.SEP}{-25}{self.SEP}{25}{self.END}"
        else:
            raise ValueError(f"Invalid motion {motion}. This should never happen.")
        return [cmd1, cmd2]

    def compile_command_list(self, motions, obstacle_ids, testing=False):
        snap_count = 0
        if not motions:
            return []
        commands = []
        prev_motion = motions[0]
        num_motions = 1
        for motion in motions[1:]:
            if motion == prev_motion and motion.can_combine():
                num_motions += 1
            else:
                if prev_motion == MovementType.CAPTURE:
                    commands.append(f"M0|0|0")
                    commands.append(f"SNAP{obstacle_ids[snap_count]}")
                    snap_count += 1
                    prev_motion = motion
                    continue
                if testing:
                    raise ValueError("This function is DEPRECATED!!")
                else:
                    cur_cmd = self.create_command(prev_motion, num_motions)
                commands.extend(cur_cmd)
                num_motions = 1

            prev_motion = motion

        if testing:
            raise ValueError("This function is DEPRECATED!!")
        else:
            if prev_motion == MovementType.CAPTURE:
                commands.append(f"M0|0|0")
                commands.append(f"SNAP{obstacle_ids[snap_count]}")
            else:
                cur_cmd = self.create_command(prev_motion, num_motions)
                commands.extend(cur_cmd)

        commands.append(f"{self.FIN}")
        commands = 	RobotCommand.merge_all(commands)
        return commands
    
    @staticmethod
    def try_merge(cmd1: str, cmd2: str):
        result1, result2 = cmd1.split("|"), cmd2.split("|")

        angle1, angle2 = int(result1[1]), int(result2[1])
        if angle1 != angle2 or angle1 != 0:
            return None

        dist1, dist2 = int(result1[2]), int(result2[2])
        motion1, motion2 = result1[0][0], result2[0][0]

        if motion1 != motion2:
            if dist1 > dist2:
                used_motion = motion1
                used_dist = dist1 - dist2
            else:
                used_motion = motion2
                used_dist = dist2 - dist1
        else:
            used_motion = motion1
            used_dist = dist1 + dist2

        speed = result1[0][1:]
        return f"{used_motion}{speed}|{0}|{used_dist}"

    @staticmethod
    def merge_all(commands: list):
        merged_commands = []
        prev_cmd = None
        for cmd in commands:
            if prev_cmd:
                if cmd == "FIN" or cmd.startswith("SNAP") or cmd == "M0|0|0":
                    merged_commands.append(prev_cmd)
                    prev_cmd = None
                    merged_commands.append(cmd)
                    continue

                merged_cmd = RobotCommand.try_merge(prev_cmd, cmd)
                if merged_cmd:
                    prev_cmd = merged_cmd
                else:
                    merged_commands.append(prev_cmd)
                    prev_cmd = cmd
            else:
                if cmd == "FIN" or cmd.startswith("SNAP") or cmd == "M0|0|0":
                    merged_commands.append(cmd)
                    continue
                prev_cmd = cmd
        return merged_commands


MOTION_MAP = {
    'T': {'straight': 'F', 'left_turn': 'L', 'right_turn': 'R', 'offset_left': 'O', 'offset_right': 'P'},
    't': {'straight': 'B', 'left_turn': 'l', 'right_turn': 'r', 'offset_left': 'o', 'offset_right': 'p'}
}

NORMAL_TURN_MAPPING = {86: 87, 87: 87, 88: 87, 89: 87}
OFFSET_TURN_VALUES = {21, 24, 25}

def convert_to_new_format(old_command: str) -> list:
    old_command = old_command.strip()
    if old_command == "FIN":
        return ["FIN"]
    elif old_command == "M0|0|0":
        return ["Z0000000"]
    elif old_command.startswith("SNAP"):
        return [old_command + "a", "Z0000000"]

    match = re.match(r"([Tt])50\|(-?\d+)\|(-?\d+)", old_command)
    if not match:
        raise ValueError(f"Unrecognized command format: {old_command}")

    motion_letter, angle_str, value_str = match.groups()
    angle, value = int(angle_str), int(value_str)
    motion_map = MOTION_MAP[motion_letter]

    if angle == 0:
        new_letter = motion_map['straight']
        return [f"{new_letter}50|{value:03d}a"]

    if abs(angle) == 25 and value in NORMAL_TURN_MAPPING:
        adjusted_value = NORMAL_TURN_MAPPING[value]
        new_letter = motion_map['right_turn'] if angle > 0 else motion_map['left_turn']
        return [f"{new_letter}50|{adjusted_value:03d}a"]

    if value in OFFSET_TURN_VALUES:
        new_letter = motion_map['offset_right'] if angle > 0 else motion_map['offset_left']
        return [f"{new_letter}20|{value:03d}a"]

    raise ValueError(f"Unrecognized command format: {old_command}")

def convert_command_list(old_commands: list) -> list:
    new_commands = []
    for cmd in old_commands:
        new_commands.extend(convert_to_new_format(cmd))
    return new_commands