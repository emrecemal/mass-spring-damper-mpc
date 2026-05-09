
import argparse

import numpy as np

from model import MassSpringDamperModel
from model_predictive_controller import ModelPredictiveController


def main(initial_state, model, horizon, dt, total_time):
    mpc = ModelPredictiveController(model, horizon, dt)
    
    # Set limits for inputs
    mpc.set_input_bounds(umin=-10.0, umax=10.0)
    
    # Set limits for states (optional, can be set to inf if no limits)
    mpc.set_state_bounds(xmin=np.array([-np.inf, 0.0]), xmax=np.array([1.0, 0.5]))
    
    time_steps = int(total_time / dt)
    state_history = np.zeros((time_steps + 1, len(initial_state)))
    state_history[0] = initial_state
    input_history = np.zeros(time_steps)
    
    # Weights for the cost function
    Q = np.diag([100.0, 1.0])  # State cost
    R = np.diag([0.01])  # Input cost
    
    for t in range(time_steps):
        current_time = t * dt
        reference = np.array([1.0, 0.0])  # Desired position and velocity
        
        mpc_object = mpc.get_mpc_object(state_history[t], Q, R, reference)
        
        res = mpc_object.solve()
        if res.info.status != 'solved':
            print(f"Optimization failed at time {current_time:.2f}s")
            break
        
        # Extract the optimal input sequence
        input_history[t] = res.x[-horizon: -(horizon - 1)][0]
        
        # Simulate the system with the first control input
        state_history[t + 1] = model.simulate_one_step(state_history[t], input_history[t], dt)
    
    # Plot the results
    model.plot_response(state_history, input_history, dt)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--mass', type=float, default=1.0, help='Mass of the system')
    parser.add_argument('--spring_constant', type=float, default=10.0, help='Spring constant')
    parser.add_argument('--damping_coefficient', type=float, default=0.5, help='Damping coefficient')
    parser.add_argument('--horizon', type=int, default=20, help='MPC horizon')
    parser.add_argument('--dt', type=float, default=0.1, help='Time step for simulation')
    parser.add_argument('--total_time', type=float, default=10.0, help='Total simulation time')
    args = parser.parse_args()
    
    initial_state = np.array([0.0, 0.0])  # Initial position and velocity
    model = MassSpringDamperModel(args.mass, args.spring_constant, args.damping_coefficient)
    
    main(initial_state, model, args.horizon, args.dt, args.total_time)
    
    