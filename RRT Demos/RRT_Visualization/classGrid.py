import random
from classNode import Node

class Grid:
    def __init__(self, width, height, padding):
        self.nodes = []  # List of all nodes in the grid
        self.queue = []  # Priority queue for RRT#
        
        self.branchLength = 20  # Maximum step size for new nodes
        self.error = self.branchLength / 2  # Error threshold for reaching the goal
        self.searchRad = 2*self.branchLength  # Search radius for finding neighbors

        self.source = Node(width / 2, height / 2)  # Starting node
        self.source.g = 0
        self.goal = Node(random.uniform(padding, width - padding), random.uniform(padding, height - padding))  # Goal node     
        self.goal.h = 0  
        self.goal.diam = 2* self.error - 5

        print(f"Euclidian Distance to Goal: {self.goal.pos.distance_to(self.source.pos)}")

    def reset(self, width, height, padding):
        self.nodes = []  # Clear all existing nodes
        self.queue = []  # Priority queue for RRT#
        self.goal.lmc = float('inf')

    def randomTree(self):
        if len(self.nodes) == 0:
            self.nodes.append(self.source)
        self.x_rand = Node(random.uniform(0, 600), random.uniform(0, 600))
        node_rand = self.nodes[int(random.uniform(0, len(self.nodes) - 1))]
        v_steer = self.steer(self.x_rand, node_rand, self.branchLength)
        x_new = Node(node_rand.pos.x + v_steer.x, node_rand.pos.y + v_steer.y, node_rand)
        x_new.g = x_new.lmc
        self.nodes.append(x_new)

        if x_new.pos.distance_to(self.goal.pos) < self.error:
            self.goal.lmc = x_new.lmc

    def RRT(self):
        # Rapidly-Exploring Random Tree algorithm
        self.x_rand = Node(random.uniform(0, 600), random.uniform(0, 600))
        x_near = self.findNear(self.x_rand)
        v_steer = self.steer(self.x_rand, x_near, self.branchLength)
        x_new = Node(x_near.pos.x + v_steer.x, x_near.pos.y + v_steer.y, x_near)
        x_new.g = x_new.lmc
        self.nodes.append(x_new)

        if x_new.pos.distance_to(self.goal.pos) < self.error:
            self.goal.lmc = x_new.lmc
            print(f"Solved with cost: {self.goal.lmc}")

    def RRTstar(self):
        self.x_rand = Node(random.uniform(0, 600), random.uniform(0, 600))
        self.extend(self.x_rand)
        self.nodes[-1].g = self.nodes[-1].lmc

        if self.nodes[-1].pos.distance_to(self.goal.pos) < self.error:
            self.goal.lmc = self.nodes[-1].lmc


    def RRTsharp(self):
        self.x_rand = Node(random.uniform(0, 600), random.uniform(0, 600))
        self.extend(self.x_rand, True)
        self.replan()

    def render(self, screen, offset_x, offset_y, showDist):
        self.source.render(screen, offset_x, offset_y, showDist)  # Render the source node
        self.goal.render(screen, offset_x, offset_y, False)  # Render the goal node
        self.x_rand.render(screen, offset_x, offset_y, False)
        for node in self.nodes:
            node.render(screen, offset_x, offset_y, showDist)  # Render all other nodes

    def findNear(self, x):
        # Find the closest node to a given position
        nearest = self.source
        for node in self.nodes:
            if node.pos.distance_to(x.pos) < nearest.pos.distance_to(x.pos):
                nearest = node
        return nearest

    def findNeighbors(self, x, radius):
        # Find all nodes within a given radius
        neighbors = []
        for node in self.nodes:
            if node.pos.distance_to(x.pos) < radius:
                neighbors.append(node)
        return neighbors

    def steer(self, near, rand, length):
        if(rand.pos.distance_to(near.pos) < length):
            length = rand.pos.distance_to(near.pos)
        direction = (near.pos - rand.pos).normalize() * length
        return direction
    
    def extend(self, x_rand, rejectSample = False):
        x_near = self.findNear(x_rand)
        v_steer = self.steer(self.x_rand, x_near, self.branchLength)
        x_new = Node(x_near.pos.x + v_steer.x, x_near.pos.y + v_steer.y, x_near, goal = self.goal)

        neighbors = self.findNeighbors(x_new, self.searchRad)
        x_new.successors = []

        for neighbor in neighbors:
            cost = neighbor.g + neighbor.pos.distance_to(x_new.pos)
            if cost < x_new.lmc:
                x_new.lmc = cost
                x_new.parent = neighbor
            x_new.successors.append(neighbor)
            neighbor.successors.append(x_new)

        if x_new.queueKey < self.goal.queueKey and rejectSample: #rejection sampling
            self.nodes.append(x_new)
            self.updateQueue(x_new)
            if x_new.pos.distance_to(self.goal.pos) < self.error:
                self.goal.lmc = x_new.lmc
                print(f"Solved with cost: {self.goal.lmc}")

        elif rejectSample == False:
            self.nodes.append(x_new)
            # self.updateQueue(x_new)
            if x_new.pos.distance_to(self.goal.pos) < self.error:
                self.goal.lmc = x_new.lmc
                print(f"Solved with cost: {self.goal.lmc}")

    def replan(self):
        self.goal.updateKey()
        while self.queue:
            x = self.queue.pop(0)
            x.g = x.lmc
            for successor in x.successors:
                cost = x.g + x.pos.distance_to(successor.pos)
                if cost < successor.lmc:
                    successor.lmc = cost
                    successor.parent = x
                    self.updateQueue(successor)

    def updateQueue(self, x):
        x.updateKey()
        if x.g != x.lmc and x in self.queue:
            self.queue.remove(x)
            self.queue.append(x)
            self.queue.sort(key=lambda node: node.queueKey)  # Keep the queue sorted
        elif x.g != x.lmc and x not in self.queue:
            self.queue.append(x)
            self.queue.sort(key=lambda node: node.queueKey)  # Sort after adding
        elif x.g == x.lmc and x in self.queue:
            self.queue.remove(x)