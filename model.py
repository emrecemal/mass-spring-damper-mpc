import numpy as np
from scipy.signal import cont2discrete


class MassSpringDamperModel:
    def __init__(self, mass, spring_constant, damping_coefficient):
        self.mass = mass
        self.spring_constant = spring_constant
        self.damping_coefficient = damping_coefficient

    def state_space_representation(self):
        A = np.array([[0, 1], [-self.spring_constant / self.mass, -self.damping_coefficient / self.mass]])
        B = np.array([[0], [1 / self.mass]])
        C = np.array([[1, 0]])
        D = np.array([[0]])
        
        return A, B, C, D
    
    def get_discrete_state_space(self, dt):
        A, B, C, D = self.state_space_representation()
        sysd = cont2discrete((A, B, C, D), dt, method='zoh')
        Ad, Bd, Cd, Dd, _ = sysd
        
        self.Ad = Ad
        self.Bd = Bd
        
        return Ad, Bd, Cd, Dd
    
    def simulate_one_step(self, initial_state, input_value, dt):
        return self.Ad @ initial_state + self.Bd.flatten() * input_value

    def simulate(self, initial_state, input_sequence, dt):
        num_steps = len(input_sequence)
        states = np.zeros((num_steps + 1, 2))
        states[0] = initial_state
        
        for i in range(num_steps):
            states[i + 1] = self.Ad @ states[i] + self.Bd.flatten() * input_sequence[i]
        
        return states
    
    def plot_response(self, state_sequence, input_sequence, dt):
        import matplotlib.pyplot as plt
        
        time = np.arange(state_sequence.shape[0]) * dt
        
        plt.figure(figsize=(10, 5))
        plt.subplot(2, 1, 1)
        plt.plot(time, state_sequence[:, 0], label='Position (x)')
        plt.plot(time, state_sequence[:, 1], label='Velocity (v)')
        plt.title('Mass-Spring-Damper System Response')
        plt.xlabel('Time (s)')
        plt.ylabel('State')
        plt.legend()
        plt.grid()
        
        plt.subplot(2, 1, 2)
        plt.plot(time[:-1], input_sequence, label='Input Force (u)')
        plt.xlabel('Time (s)')
        plt.ylabel('Input Force')
        plt.legend()
        plt.grid()
        plt.show()
        
        