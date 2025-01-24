import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple

class GridMap:
    def __init__(self, obstacles: List[object] = [], resolution: int = 1):
        self.x_min, self.x_max = 0.0, 200.0
        self.y_min, self.y_max = 0.0, 200.0
        self.resolution = resolution
        self.grid = np.zeros((int(self.x_max/resolution), int(self.y_max/resolution)))

        self.objects = []
        self.add_object(obstacles)

    def add_object(self, obj):
        """Adds an object to the grid."""
        for obstacle in obj:
            self.objects.append(obstacle)
            i, j = self.grid_index(obstacle.x, obstacle.y)
            size = obstacle.size
            self.grid[i:i+size, j:j+size] = 1  # Mark obstacle
            self.mark_buffer_zone(obstacle)

    def remove_object(self, obj):
        """Removes an object from the grid."""
        if obj in self.objects:
            self.objects.remove(obj)
            i, j = self.grid_index(obj.x, obj.y)
            size = obj.size
            self.grid[i:i+size, j:j+size] = 0  # Clear obstacle
            self.mark_buffer_zone(obj, clear=True)

    def mark_buffer_zone(self, obj, buffer_size=3, clear=False):
        """Marks or clears buffer zones around an object."""
        i, j = self.grid_index(obj.x, obj.y)
        size = obj.size
        buffer_val = 0 if clear else 1
        self.grid[max(0, i-buffer_size):i+size+buffer_size,
                  max(0, j-buffer_size):j+size+buffer_size] = buffer_val

    def is_collision(self, x, y):
        """Checks if a point collides with an object or buffer zone."""
        i, j = self.grid_index(x, y)
        if i < 0 or i >= self.grid.shape[0] or j < 0 or j >= self.grid.shape[1]:
            return True  # Out of bounds
        return self.grid[i, j] == 1

    def grid_index(self, x, y):
        """Converts world coordinates to grid indices."""
        i = int(x / self.resolution)
        j = int(y / self.resolution)
        return i, j

    def display(self):
        """Displays the current grid."""
        plt.imshow(self.grid, origin='lower', cmap='Greys')
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Grid Map")
        plt.show()
