from scipy.integrate import odeint
import math
import time

class RoverSimulation:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation angle
        self.d = 0.037  # Wheelbase (calibration)
        
        self.state_times = [3.3, 3.0]  # Durations for left and right motion
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

    def compute_velocity(self, current_time, start_time, end_time):
        """
        Computes the velocity based on the current time, smoothly transitioning
        from 0 to 2 over the specified duration.
        """
        duration = end_time - start_time
        if current_time < start_time:
            return 0.0
        elif current_time > end_time:
            return 2.0
        else:
            # Linear interpolation
            return 2.0 * (current_time - start_time) / duration

    def run(self):
        state_durations = [0, self.state_times[0], sum(self.state_times)]
        while self.current_time <= state_durations[-1]:
            if self.current_time < state_durations[1]:  # Moving left
                omega = -2.0 / self.d
                print("State 0: Moving left")
                v = self.compute_velocity(self.current_time, 0, self.state_times[0])
            elif self.current_time < state_durations[-1]:  # Moving right
                omega = 2.0 / self.d
                print("State 1: Moving right")
                v = self.compute_velocity(self.current_time, state_durations[1], state_durations[-1])
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
            print(f"Time: {self.current_time:.2f}s, Velocity: {v:.2f}, Position: x = {self.x:.2f}, y = {self.y:.2f}, theta = {self.theta:.2f}")

            # Increment time
            self.current_time += self.timer_period
            time.sleep(self.timer_period)

if __name__ == '__main__':
    simulation = RoverSimulation()
    simulation.run()
