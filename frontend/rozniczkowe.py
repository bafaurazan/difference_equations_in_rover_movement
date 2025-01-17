from scipy.integrate import odeint
import math
import time

class RoverSimulation:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation angle
        self.d = 0.037  # Wheelbase (calibration)
        
        self.state_times = [2.3, 3.0]  # Durations for left and right motion
        self.timer_period = 0.1  # Update interval (seconds)
        self.current_time = 0.0
        self.state = 0

    def diff_drive_ode(self, state, t, v, omega):
        """Differential equations for robot motion."""
        x, y, theta = state
        dxdt = v * math.cos(theta)
        dydt = v * math.sin(theta)
        dthetadt = omega
        return [dxdt, dydt, dthetadt]

    def run(self):
        state_durations = [0, self.state_times[0], sum(self.state_times)]
        while self.current_time <= state_durations[-1]:
            if self.current_time < state_durations[1]:  # Moving left
                v = 2.0
                omega = -2.0 / self.d
                print("State 0: Moving left")
            elif self.current_time < state_durations[-1]:  # Moving right
                v = 2.0
                omega = 2.0 / self.d
                print("State 1: Moving right")
            else:  # Stopped
                v = 0.0
                omega = 0.0
                print("State 2: Stopped")

            # Solve ODEs
            t = [0, self.timer_period]
            state = [self.x, self.y, self.theta]
            new_state = odeint(self.diff_drive_ode, state, t, args=(v, omega))[-1]
            
            # Update position and angle
            self.x, self.y, self.theta = new_state
            print(f"Position: x = {self.x:.2f}, y = {self.y:.2f}, theta = {self.theta:.2f}")

            # Increment time
            self.current_time += self.timer_period
            time.sleep(self.timer_period)

if __name__ == '__main__':
    simulation = RoverSimulation()
    simulation.run()
