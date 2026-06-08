import pygame
import sys

# Initialize Pygame
pygame.init()

# Constants
WINDOW_WIDTH, WINDOW_HEIGHT = 800, 600
GRID_SIZE = 20  # Size of each grid square

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# Initialize the screen
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Pathfinding Visualizer")

# Create a 2D grid
grid = [[0 for _ in range(WINDOW_WIDTH // GRID_SIZE)] for _ in range(WINDOW_HEIGHT // GRID_SIZE)]

# Draw the grid
def draw_grid():
    for y in range(len(grid)):
        for x in range(len(grid[0])):
            rect = pygame.Rect(x * GRID_SIZE, y * GRID_SIZE, GRID_SIZE, GRID_SIZE)
            color = WHITE
            if grid[y][x] == 1:
                color = BLACK  # Obstacle
            elif grid[y][x] == 2:
                color = GREEN  # Path
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, BLACK, rect, 1)  # Grid lines

# Example pathfinding algorithm (placeholder)
def find_path():
    # Clear previous path
    for y in range(len(grid)):
        for x in range(len(grid[0])):
            if grid[y][x] == 2:
                grid[y][x] = 0

    # Placeholder: Draw a straight path from top-left to bottom-right
    for i in range(min(len(grid), len(grid[0]))):
        if grid[i][i] == 0:
            grid[i][i] = 2

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Handle mouse clicks
        if event.type == pygame.MOUSEBUTTONDOWN:
            x, y = pygame.mouse.get_pos()
            grid_x, grid_y = x // GRID_SIZE, y // GRID_SIZE

            # Toggle obstacles
            if grid[grid_y][grid_x] == 0:
                grid[grid_y][grid_x] = 1  # Add obstacle
            elif grid[grid_y][grid_x] == 1:
                grid[grid_y][grid_x] = 0  # Remove obstacle

        # Handle key presses
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                find_path()  # Run pathfinding algorithm

    # Clear the screen
    screen.fill(WHITE)

    # Draw the grid
    draw_grid()

    # Update the display
    pygame.display.flip()

# Quit Pygame
pygame.quit()
sys.exit()
