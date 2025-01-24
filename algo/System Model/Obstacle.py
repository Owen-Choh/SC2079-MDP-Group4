import pygame

class Obstacle:
    def __init__(self, x: int, y: int, image_facing: str, width: int = 10, height: int = 10, id: int = -1):
        self.x = x
        self.y = y
        self.image_facing = image_facing
        self.width = width
        self.height = height
        self.id = id

    def grid_coordinates(self, resolution: int = 1):
        """Convert world coordinates to grid coordinates."""
        i = int(self.x / resolution)
        j = int(self.y / resolution)
        return i, j

    def add_virtual_wall(self, grid: np.ndarray, buffer: int = 3):
        """Add a buffer zone around the obstacle."""
        i, j = self.grid_coordinates()
        grid[max(0, i - buffer):i + self.width + buffer, max(0, j - buffer):j + self.height + buffer] = 1

    def remove_virtual_wall(self, grid: np.ndarray, buffer: int = 3):
        """Remove a buffer zone around the obstacle."""
        i, j = self.grid_coordinates()
        grid[max(0, i - buffer):i + self.width + buffer, max(0, j - buffer):j + self.height + buffer] = 0

    def visualize(self, pygame_surface):
        """Render the obstacle on the grid using Pygame."""
        image = pygame.image.load("obstacle.png")
        if self.image_facing == 'E':
            image = pygame.transform.rotate(image, 270)
        elif self.image_facing == 'S':
            image = pygame.transform.rotate(image, 180)
        elif self.image_facing == 'W':
            image = pygame.transform.rotate(image, 90)
        pygame_surface.blit(image, (self.x, self.y))
