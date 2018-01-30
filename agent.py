from gameobjects import GameObject
from move import Direction
from move import Move
from board import *
import heapq

class Graph:
    def __init__(self, board):
        self.board = board
        self.width = len(board)
        self.height = len(board[0])
        self.walls = []
        for x in range(self.width):
            for y in range(self.height):
                if board[x][y]==GameObject.WALL or board[x][y]==GameObject.SNAKE_BODY:
                    self.walls.append((x, y))

    def findHead(self):
        board = self.board
        for x in range(self.width):
            for y in range(self.height):
                if self.board[x][y]==GameObject.SNAKE_HEAD:
                    return (x, y)

    def findFood(self):
        food = []
        for x in range(self.width):
            for y in range(self.height):
                if self.board[x][y]==GameObject.FOOD:
                    food.append((x, y))
        return food

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        return id not in self.walls
    
    def neighbors(self, id, first, direction):
        (x, y) = id
        results = []
        if first and direction==Direction.NORTH:
            results = [(x+1, y),(x, y-1),(x-1, y)]
        elif direction==Direction.EAST:
            results = [(x+1,y),(x,y-1), (x,y+1)]
        elif direction==Direction.SOUTH:
            results = [(x+1, y),(x-1, y),(x, y+1)]
        elif direction==Direction.WEST:
            results = [(x, y-1),(x-1, y),(x, y+1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results 
    def cost(self, id, id2):
        (x1, y1) = id
        (x2, y2) = id2
        return (abs(x2-x1)+abs(y2-y1))

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path

def heuristic(goals, b):
    (x2, y2) = b
    heuristics = []
    for goal in goals:
        (x1, y1) = goal
        heuristics.append(abs(x1 - x2) + abs(y1 - y2))
    return min(heuristics)

def a_star_search(graph, start, goal, direction):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    first = True
    while not frontier.empty():
        current = frontier.get()
        
        if current in goal:
            break
        for next in graph.neighbors(current, first, direction):
            first = False
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far, current

class Agent:
        def __init__(self):
            self.path = []
            self.body = []

        def findDirection(self, head, direction, path):
            if self.path == []:
                return 0
            (x1, y1) = self.path.pop(0)
            self.body.append((x1, y1))
            (x2, y2) = head
            print("Head: "+str(head))
            mutation = (x1-x2, y1-y2)
            print("Muta: "+str(mutation))
            if mutation==(1,0):
               newdirection = 1
            elif mutation==(0,1):
               newdirection = 2
            elif mutation==(-1,0):
               newdirection = 3
            elif mutation==(0,-1):
               newdirection = 0
            if direction == Direction.NORTH:
                direction = 0
            elif direction == Direction.EAST:
                direction = 1
            elif direction == Direction.SOUTH:
                direction = 2
            elif direction == Direction.WEST:
                direction = 3
            move = newdirection-direction
            if move == -3:
                move = 1
            elif move == 3:
                move = -1
            return move

        def get_move(self, board, score, turns_alive, turns_to_starve, direction):
            self.graph = Graph(board)          #move these three lines
            self.head = self.graph.findHead()  #...
            self.food = self.graph.findFood()  #...
            if self.path == []:

                #replace them here
                came_from, cost_so_far, goal = a_star_search(self.graph, self.head, self.food, direction)
                self.path = reconstruct_path(came_from, self.head, goal)
                self.path.pop(0)
                print("Head: "+str(self.head))
                print("Path: "+str(self.path))
                nextmove = self.findDirection(self.head, direction, self.path)
            else:
                self.head = self.graph.findHead()
                nextmove = self.findDirection(self.head, direction, self.path)
             
            if nextmove==-1:
                return Move.LEFT
            elif nextmove==0:
                return Move.STRAIGHT
            elif nextmove==1:
                return Move.RIGHT
            return null

        def on_die(self):
            self.path = []
            self.body = []
            pass
