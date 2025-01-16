import streamlit as st
import importlib.util

# Load the provided Python file
file_path = 'rozniczkowe.py'

spec = importlib.util.spec_from_file_location("rozniczkowe", file_path)
rozniczkowe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rozniczkowe)

# Streamlit frontend
st.title("Dynamiczne Wprowadzanie Zmiennych")

st.sidebar.header("Wprowadź wartości zmiennych")
state_time_start = st.sidebar.number_input("Czas początkowy ruchu w lewo (state_time[0])", value=2.3)
state_time_end = st.sidebar.number_input("Czas końcowy ruchu w prawo (state_time[1])", value=3.0)

# Display input values
state_times = [state_time_start, state_time_end]
st.write("### Wprowadzone wartości")
st.write(f"Czasy ruchu: {state_times}")

# Run the simulation
if st.button("Uruchom symulację"):
    try:
        # Instantiate and configure the RoverSimulation class
        simulation = rozniczkowe.RoverSimulation()
        simulation.state_times = state_times

        # Run the simulation (this might take some time due to sleep in the original code)
        st.write("### Wynik symulacji:")
        with st.spinner("Uruchamianie symulacji..."):
            simulation.run()
        st.success("Symulacja zakończona.")
    except Exception as e:
        st.error(f"Wystąpił błąd: {e}")
