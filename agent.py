from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from a_star import AStarPlanner

from utils.Point import Point

class ExampleAgent(BaseAgent): 
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow) 
        self.path = [] # store the path given by the A*

    def decision(self):
        if len(self.targets) == 0: #check if we still have targets
            return

        target = self.targets[0] #gets the first target
        #print(target)
        
        if not self.path: #just does it when we don't have a path yet 
            if self.pos.dist_to(target) > 0.1: #it didn't get into the target yet
                planner = AStarPlanner(6.0, 4.0, obstacles= self.get_obstacles()) # width = 6 and height = 4
                self.path = planner.plan((self.pos.x, self.pos.y), (target.x,  target.y))
                #print(self.path)
                if not self.path:
                    print("Nenhum caminho encontrado")
                    self.targets.pop(0)
                    return
                
            # else: #when the robot get in the target
            #     self.targets.pop(0)
            #     return
            
        
        next_point = Point(*self.path[0]) 
        #print(next_point)
        
        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, next_point)
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)
        #print(f"Target velocity: {target_velocity}, Target angle velocity: {target_angle_velocity}")

        #checks if the robot got into the next point with 0.1 of tolerance
        if self.pos.dist_to(next_point) < 0.1:
            self.path.pop(0) #pops out the atual point from the PQ
            
        if not self.path and self.pos.dist_to(target) < 0.1:
            self.targets.pop(0) 
            
     

        return
    
 
    
    def get_obstacles(self):
        #get all the obstacles in the format needed for the A*
        obstacles = []
        for robot_id, robot in self.opponents.items():
            obstacles.append((robot.x, robot.y)) # add the opponents as obstacles
        for robot_id, robot in self.teammates.items():
            if robot_id != self.id: #doesn't count with the robot being used right now
                obstacles.append((robot.x, robot.y))
        
        return obstacles

    def post_decision(self):
        pass