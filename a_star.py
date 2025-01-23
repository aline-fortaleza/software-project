import heapq
import math

class AStarPlanner:
    def __init__(self, field_width, field_height, obstacles):
        self.field_width = field_width
        self.field_height = field_height
        self.obstacles = set(obstacles)  # verify obstacles more quickly

    def heuristic(self, node, goal):
        # using the usual distance formula (because we don't have a "real" grid)
        return math.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)

    def is_valid(self, node):
        x, y = node
        # check if the node is inside the field and isn't a obstacle
        return ( (-self.field_width/2) <= x <= (self.field_width/2) and  
                (-self.field_height/2) <= y <= (self.field_height/2) and 
                node not in self.obstacles)

    def get_neighbors(self, node):
        x, y = node
        neighbors = [ #I tested some values and by checking the neighboor by 0.25 it didn't crash. I don't know why
            (x - 0.25, y), (x + 0.25, y), (x, y - 0.25), (x, y + 0.25),  # right, left, up and down
            (x - 0.25, y - 0.25), (x + 0.25, y - 0.25), (x - 0.25, y + 0.25), (x + 0.25, y + 0.25)  # diagonal
        ]
        return [n for n in neighbors if self.is_valid(n)] 

    def plan(self, start, goal):
        # Priority queue for open nodes
        open_set = []
        heapq.heappush(open_set, (0, start))  # (value of f(n), node)

        came_from = {}  # get the path
        g_score = {start: 0}  # Cost from beggining until now
        f_score = {start: self.heuristic(start, goal)}  # Total estimation

        while open_set:
            _, current = heapq.heappop(open_set)

            # check if got into the end point
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # update the score
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                    # Add to the priority queue
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # If it doesn't find a answer it will return void

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
def main():
    field_width = 6.0
    field_height = 4.0
    obstacles = [(-2, 0), (-1.5, 1), (0, -1)]
    
    planner = AStarPlanner(field_width, field_height, obstacles)
    
    start = (-2.5, -1.5)  # Começa no canto inferior esquerdo
    goal = (2.5, 1.5)  # Termina no canto superior direito
    
    path = planner.plan(start, goal)
    print("Caminho:", path)
main()