import pygame
import sys
from classGrid import Grid

pygame.init()
window_width, window_height, toolbar_width = 900, 600, 300
myGrid = Grid(window_width - toolbar_width, window_height, 50)
screen = pygame.display.set_mode((window_width, window_height))
clock = pygame.time.Clock()

algorithms = ["Random Tree", "RRT", "RRT*", "RRT#"]
selected_algorithm = algorithms[1]
speed = 1
show_distance_colors = False

font = pygame.font.Font(None, 36)

def draw_button(screen, x, y, width, height, text, selected):
    """Draws a button with text."""
    color = (255, 0, 0) if selected else (200, 200, 200)
    text_color = (0, 0, 0) if not selected else (255, 255, 255)
    pygame.draw.rect(screen, color, (x, y, width, height), border_radius=5)
    pygame.draw.rect(screen, (0, 0, 0), (x, y, width, height), 2, border_radius=5)
    button_text = font.render(text, True, text_color)
    text_rect = button_text.get_rect(center=(x + width // 2, y + height // 2))
    screen.blit(button_text, text_rect)

slider_x, slider_y, slider_width, slider_height = 15, 315, 270, 20
def draw_slider(screen, x, y, width, height, value, max_value):
    """Draw a slider with a hollow rectangle background and a black border."""
    pygame.draw.rect(screen, (255, 255, 255), (x, y, width, height), 1)  # Background
    pygame.draw.rect(screen, (0, 0, 0), (x, y, width, height), 1)  # Border
    handle_width = int(width * (value / max_value))
    pygame.draw.rect(screen, (100, 100, 255), (x, y, handle_width, height))  # Handle
    return value

toggle_x, toggle_y, toggle_width, toggle_height = 210, 362, 60, 30
def draw_toggle_switch(screen, x, y, width, height, state):
    """Draw a toggle switch with circular edges and a movable circle."""
    border_color = (150, 150, 150)
    pygame.draw.rect(screen, border_color, (x, y, width, height), border_radius=height // 2)
    pygame.draw.rect(screen, (0, 0, 0), (x, y, width, height), 2, border_radius=height // 2)

    # Circle for the toggle
    circle_x = x + (width - height if state else 0)
    circle_color = (0, 200, 0) if state else (200, 0, 0)
    pygame.draw.circle(screen, circle_color, (circle_x + height // 2, y + height // 2), height // 2 - 2)
    return state

def draw_toolbar():
    # Algorithm Buttons
    pygame.draw.rect(screen, (200, 200, 200), (0, 0, toolbar_width, 600))  # Toolbar background
    title = font.render("Algorithms", True, (0, 0, 0))
    screen.blit(title, (10, 10))
    for i, alg in enumerate(algorithms):
        draw_button(screen, 10, 60 + i * 50, 280, 40, alg, alg == selected_algorithm)

    pygame.draw.line(screen, (0, 0, 0), (10, 260), (290, 260), 2)

    # Speed Slider
    speed_text = font.render(f"Speed: {speed} FPS", True, (0, 0, 0))
    screen.blit(speed_text, (10, 280))
    draw_slider(screen, slider_x, slider_y, slider_width, slider_height, speed, 60)

    pygame.draw.line(screen, (0, 0, 0), (10, slider_y + 30), (290, slider_y + 30), 2)

    # Distance Colors Toggle
    toggle_text = font.render("Distance Colors", True, (0, 0, 0))
    screen.blit(toggle_text, (10, 365))
    draw_toggle_switch(screen, toggle_x, toggle_y, toggle_width, toggle_height, show_distance_colors)

    # Shortest Path
    shortest_path_text = font.render(f"Shortest Path: {'INF' if myGrid.goal.lmc == float('inf') else int(myGrid.goal.lmc)}", True, (0, 0, 0))
    screen.blit(shortest_path_text, (10, 550))

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:  # Exit the loop if the window is closed
            running = False

        # Mouse Events
        if event.type == pygame.MOUSEBUTTONDOWN:
            mouse_x, mouse_y = pygame.mouse.get_pos()

            # Check if algorithm is clicked
            for i, alg in enumerate(algorithms):
                button_x, button_y, button_width, button_height = 10, 60 + i * 50, 280, 40
                if button_x <= mouse_x <= button_x + button_width and button_y <= mouse_y <= button_y + button_height:
                    selected_algorithm = alg
                    myGrid.reset(window_width - toolbar_width, window_height, 50)

            # Check if slider is clicked
            if slider_x <= mouse_x <= slider_x + slider_width and slider_y <= mouse_y <= slider_y + slider_height:
                slider_pos = mouse_x - slider_x
                speed = max(1, min(60, int((slider_pos / slider_width) * 60)))  # Clamp speed between 0 and 60

            # Check if toggle switch is clicked
            if toggle_x <= mouse_x <= toggle_x + toggle_width and toggle_y <= mouse_y <= toggle_y + toggle_height:
                show_distance_colors = not show_distance_colors


    screen.fill((255, 255, 255))  # Clear Screen

    # Execute Algorithm
    if selected_algorithm == "Random Tree":
        myGrid.randomTree()
    if selected_algorithm == "RRT":
        myGrid.RRT()
    if selected_algorithm == "RRT*":
        myGrid.RRTstar()
    if selected_algorithm == "RRT#":
        myGrid.RRTsharp()

    # Render the grid and its nodes within the workspace
    pygame.draw.rect(screen, (240, 240, 240), (toolbar_width, 0, 600, 600))  # Workspace background
    myGrid.render(screen, toolbar_width, 0, show_distance_colors)

    draw_toolbar()

    pygame.display.flip()  # Update the display
    clock.tick(speed)  # Limit the frame rate

pygame.quit()
sys.exit()