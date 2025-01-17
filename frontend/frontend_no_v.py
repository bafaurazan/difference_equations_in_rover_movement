import streamlit as st
import time
from scipy.integrate import odeint
import math
import matplotlib.pyplot as plt

class RoverSimulation:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.d = 0.037
        self.state_times = [2.3, 3.0]
        self.timer_period = 0.5  # Execute steps every 0.5 seconds
        self.current_time = 0.0

    def diff_drive_ode(self, state, t, v, omega):
        x, y, theta = state
        dxdt = v * math.cos(theta)
        dydt = v * math.sin(theta)
        dthetadt = omega
        return [dxdt, dydt, dthetadt]

    def step(self):
        state_durations = [0, self.state_times[0], sum(self.state_times)]
        if self.current_time < state_durations[1]:  # State 0
            v = 2.0
            omega = -2.0 / self.d
            state_name = "State 0: Moving left"
        elif self.current_time < state_durations[-1]:  # State 1
            v = 2.0
            omega = 2.0 / self.d
            state_name = "State 1: Moving right"
        else:  # Stopped
            v = 0.0
            omega = 0.0
            state_name = "State 2: Stopped"

        t = [0, self.timer_period]
        state = [self.x, self.y, self.theta]
        new_state = odeint(self.diff_drive_ode, state, t, args=(v, omega))[-1]

        self.x, self.y, self.theta = new_state
        self.current_time += self.timer_period

        return state_name, self.x, self.y, self.theta

# Initialize session state variables
if "simulation" not in st.session_state:
    st.session_state.simulation = RoverSimulation()
if "x_data_0" not in st.session_state:
    st.session_state.x_data_0 = []
    st.session_state.y_data_0 = []
if "x_data_1" not in st.session_state:
    st.session_state.x_data_1 = []
    st.session_state.y_data_1 = []
if "running" not in st.session_state:
    st.session_state.running = False
if "paused" not in st.session_state:
    st.session_state.paused = False
if "simulation_finished" not in st.session_state:
    st.session_state.simulation_finished = False

# Streamlit UI
st.title("Symulacja Rovera")
st.sidebar.header("Wprowadź wartości zmiennych")
state_time_start = st.sidebar.number_input("Czas początkowy ruchu w lewo (state_time[0])", value=2.3)
state_time_end = st.sidebar.number_input("Czas końcowy ruchu w prawo (state_time[1])", value=3.0)

st.session_state.simulation.state_times = [state_time_start, state_time_end]

start_button = st.button("Uruchom symulację")
pause_button = st.button("Pauza symulacji")
resume_button = st.button("Wznów symulację")
stop_button = st.button("Zatrzymaj symulację")

output_placeholder = st.empty()
plot_placeholder_0 = st.empty()
plot_placeholder_1 = st.empty()

# Data for plotting
x_data_0 = st.session_state.x_data_0
y_data_0 = st.session_state.y_data_0
x_data_1 = st.session_state.x_data_1
y_data_1 = st.session_state.y_data_1

# Start simulation
if start_button:
    st.session_state.running = True
    st.session_state.paused = False
    st.session_state.simulation_finished = False
    st.session_state.simulation.current_time = 0.0
    x_data_0.clear()
    y_data_0.clear()
    x_data_1.clear()
    y_data_1.clear()

# Pause simulation
if pause_button:
    st.session_state.paused = True

# Resume simulation
if resume_button:
    st.session_state.paused = False

# Stop simulation
if stop_button:
    st.session_state.running = False
    st.session_state.paused = False
    st.session_state.simulation_finished = True

# Update plots function
def update_plots():
    if x_data_0:
        plt.figure()
        plt.plot(x_data_0, y_data_0, marker="o")
        plt.scatter(x_data_0[0], y_data_0[0], color="black", label="Start", zorder=5)
        plt.scatter(x_data_0[-1], y_data_0[-1], color="red", label="End", zorder=5)
        plt.title("State 0 Trajectory")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.legend()
        plot_placeholder_0.pyplot(plt)
    if x_data_1:
        plt.figure()
        plt.plot(x_data_1, y_data_1, marker="o")
        plt.scatter(x_data_1[0], y_data_1[0], color="black", label="Start", zorder=5)
        plt.scatter(x_data_1[-1], y_data_1[-1], color="red", label="End", zorder=5)
        plt.title("State 1 Trajectory")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.legend()
        plot_placeholder_1.pyplot(plt)

# Main simulation loop
if st.session_state.running:
    while st.session_state.running:
        if not st.session_state.paused:
            state_name, x, y, theta = st.session_state.simulation.step()
            output_placeholder.text(f"{state_name}\nPozycja: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")

            # Update data
            if "State 0" in state_name:
                x_data_0.append(x)
                y_data_0.append(y)
            elif "State 1" in state_name:
                x_data_1.append(x)
                y_data_1.append(y)

            # Update plots
            update_plots()

            # Check if simulation is finished
            if state_name == "State 2: Stopped":
                st.session_state.running = False
                st.session_state.simulation_finished = True

            time.sleep(st.session_state.simulation.timer_period)
        else:
            # Maintain current plots when paused
            update_plots()
            time.sleep(0.1)
