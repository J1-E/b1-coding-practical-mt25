# Import the relevant classes from the dynamic module
from .dynamic import Submarine, Mission 

class Controller:
    def __init__(self, submarine: Submarine, mission: Mission, Kp: float, Kd: float):
        self.submarine = submarine
        self.mission = mission
        self.Kp = Kp
        self.Kd = Kd

    #Find error in a separate method to keep compute_action cleaner (using observation as input because it makes sense to name it that way within this controller method)
    def find_error(self, observation: float, time_step: int):
        reference_depth = self.mission.reference[time_step]
        error = reference_depth - observation
        return error

    def compute_action(self, observation: float, time_step: int) -> float:
        error = self.find_error(observation, time_step)
        
        #compute derivative using the difference in reference subtracted by the difference in y position which will by y velocity since we always have both contained within this class
        if time_step > 0:
            derivative = (self.mission.reference[time_step] - self.mission.reference[time_step - 1]) - self.submarine.vel_y
        else:
            derivative = 0
        action = self.Kp * error - self.Kd * derivative
        return action