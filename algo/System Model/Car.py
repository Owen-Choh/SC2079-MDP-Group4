import numpy as np


class Car:
    def __init__(self, grid_map, x=20.0, y=20.0, theta=0.0, wheelbase=5.0, max_steering_angle=np.pi/6):
        """Initialize the Car.

        Args:
            grid_map: Reference to the grid map.
            x (float): Initial x-coordinate (cm).
            y (float): Initial y-coordinate (cm).
            theta (float): Initial orientation (radians, 0 = east).
            wheelbase (float): Distance between the car's front and rear axles (cm).
            max_steering_angle (float): Maximum steering angle (radians).
        """
        self.grid_map = grid_map
        self.x = x
        self.y = y
        self.theta = theta  # Orientation in radians
        self.wheelbase = wheelbase  # Distance between axles
        self.max_steering_angle = max_steering_angle  # Max steering angle
        self.linear_velocity = 0.0  # Speed (cm/s)
        self.angular_velocity = 0.0  # Angular velocity (rad/s)
        self.dt = 0.1  # Time step (seconds)

    def compute_turning_radius(self, speed):
        """Compute the turning radius based on the car's speed and steering geometry."""
        if speed == 0:
            return float('inf')  # Infinite radius for no movement
        steering_angle = min(self.max_steering_angle, np.arctan(speed / self.wheelbase))
        return self.wheelbase / np.tan(steering_angle)

    def move_straight(self, distance):
        """Move in a straight line for a given distance."""
        self.x += distance * np.cos(self.theta)
        self.y += distance * np.sin(self.theta)

    def move_arc(self, speed, angle, direction):
        """Move in an arc with a speed-dependent turning radius.

        Args:
            speed (float): Speed of the car (cm/s).
            angle (float): Angle to turn (radians).
            direction (str): Direction of the turn ('left' or 'right').
        """
        radius = self.compute_turning_radius(speed)
        if direction == "left":
            self.theta += angle
        elif direction == "right":
            self.theta -= angle
        else:
            raise ValueError("Invalid direction. Use 'left' or 'right'.")

        # Update position based on the arc
        self.x += radius * (np.sin(self.theta) - np.sin(self.theta - angle))
        self.y += radius * (-np.cos(self.theta) + np.cos(self.theta - angle))

    def turn_to_angle(self, target_theta, speed):
        """Turn the car to face a specific target angle using arcs."""
        delta_theta = target_theta - self.theta
        delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-π, π]

        if delta_theta > 0:
            # Turn left
            self.move_arc(speed, delta_theta, direction="left")
        elif delta_theta < 0:
            # Turn right
            self.move_arc(speed, -delta_theta, direction="right")

    def navigate_to(self, target_x, target_y, speed):
        """Navigate to a target position.

        Args:
            target_x (float): Target x-coordinate (cm).
            target_y (float): Target y-coordinate (cm).
            speed (float): Speed of the car (cm/s).
        """
        # Calculate straight-line distance and target angle
        dx = target_x - self.x
        dy = target_y - self.y
        distance = np.sqrt(dx**2 + dy**2)
        target_theta = np.arctan2(dy, dx)

        # Turn to face the target
        self.turn_to_angle(target_theta, speed)

        # Move straight to the target
        self.move_straight(distance)

    def update_position(self):
        """Update the car's position based on its velocities."""
        if self.angular_velocity == 0.0:
            # Straight-line movement
            self.move_straight(self.linear_velocity * self.dt)
        else:
            # Arc movement
            radius = self.linear_velocity / self.angular_velocity
            angle = self.angular_velocity * self.dt
            direction = "left" if self.angular_velocity > 0 else "right"
            self.move_arc(self.linear_velocity, angle, direction)

    def check_collision(self):
        """Check for collisions using the grid map."""
        return self.grid_map.is_collision(self.x, self.y)

    def perform_image_recognition(self):
        """Simulate image recognition at the current position."""
        print(f"Performing image recognition at ({self.x:.2f}, {self.y:.2f}).")
