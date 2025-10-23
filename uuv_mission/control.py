# Import the relevant classes from the dynamic module
from .dynamic import Submarine, Mission 
import numpy as np

class Controller:
    # Include control gains in initialisation so that we can tune them when creating the controller (and have Ki default to 0)
    def __init__(self, submarine: Submarine, mission: Mission, Kp: float, Kd: float, Ki: float = 0.0):
        self.submarine = submarine
        self.mission = mission
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

    #Find error in a separate method to keep compute_action cleaner (using observation as input because it makes sense to name it that way within this controller method)
    def find_error(self, observation: float, time_step: int) -> float:
        reference_depth = self.mission.reference[time_step]
        error = reference_depth - observation
        return error

    def PD_action(self, observation: float, time_step: int) -> float:
        error = self.find_error(observation, time_step)
        
        #compute derivative using the difference in reference subtracted by the difference in y position which will = y velocity since we always have both contained within this class
        if time_step > 0:
            derivative = (self.mission.reference[time_step] - self.mission.reference[time_step - 1]) - self.submarine.vel_y
        else:
            derivative = 0
        PD_action = self.Kp * error + self.Kd * derivative
        return PD_action
    
    def integral_action(self, y_positions: np.ndarray, time_step: int) -> float: # Need to feed the position array to the method to integrate error over time
        # TODO Placeholder for integral action if needed in future
        current_error=self.mission.reference[time_step]-y_positions[time_step]
        for i in range(time_step):
            previous_error=self.mission.reference[i]-y_positions[i]
            current_error+=previous_error
        integral_action=self.Ki*current_error
        return integral_action