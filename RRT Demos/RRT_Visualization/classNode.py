import pygame

# Constants
inf = float('inf')
maxDist = 600

class Node:
    def __init__(self, x, y, parent = None, goal = None):
        self.pos = pygame.Vector2(x, y)  # Position as a 2D vector
        self.g = inf  # Cost-to-go: cost to reach this node from the start
        self.lmc = inf  # Least marginal cost: lower bound cost for the node
        self.h = self.pos.distance_to(goal.pos) if goal else inf  # Heuristic value: estimated cost to the goal
        self.updateKey()  # Priority queue key based on cost and heuristic

        self.parent = parent  # Reference to the parent node
        self.successors = []  # List of successor nodes

        self.diam = 3  # Diameter of the node when rendered
        self.edgeWeight = 1  # Thickness of the edge connecting to the parent

        # If a parent is provided, calculate the least marginal cost and update the queue key
        if parent:
            self.lmc = parent.g + self.pos.distance_to(parent.pos)  # Update lmc based on parent
            self.updateKey()  # Recalculate the queue key

    def updateKey(self):
        # Update the priority queue key: min(g, lmc) + heuristic value (h)
        self.queueKey = min(self.g, self.lmc) + self.h

    def render(self, screen, offset_x, offset_y, showDist):
        if showDist:
            hue = 1 - (self.g / maxDist) if self.g < maxDist else 0  # Normalize g for color gradient
            color = pygame.Color(0)  # Initialize color object
            color.hsva = (hue * 360, 100, 100, 100)  # Set color in HSV mode for smooth gradient
        else:
            color = pygame.Color(0)  # Initialize color object

        # Draw the node as a circle
        pygame.draw.circle(screen, color, (int(self.pos.x + offset_x), int(self.pos.y + offset_y)), self.diam)

        # If the node has a parent, draw an edge connecting to it
        if self.parent:
            pygame.draw.line(screen, color, (int(self.pos.x + offset_x), int(self.pos.y + offset_y)), 
                             (int(self.parent.pos.x + offset_x), int(self.parent.pos.y + offset_y)), self.edgeWeight)
