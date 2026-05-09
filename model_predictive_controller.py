import osqp
import numpy as np
from scipy import sparse


class ModelPredictiveController:
    def __init__(self, model, horizon, dt):
        self.model = model
        self.horizon = horizon
        self.dt = dt
        self.Ad, self.Bd, _, _ = self.model.get_discrete_state_space(dt)
        
        self.xmin = -np.inf * np.ones(self.Ad.shape[0])
        self.xmax = np.inf * np.ones(self.Ad.shape[0])
        self.umin = -np.inf * np.ones(self.Bd.shape[1])
        self.umax = np.inf * np.ones(self.Bd.shape[1])
        
    def set_state_bounds(self, xmin, xmax):
        self.xmin = xmin
        self.xmax = xmax
        
    def set_input_bounds(self, umin, umax):
        self.umin = umin
        self.umax = umax

    def setup_optimization_problem(self, initial_state, reference, Q, R):
        n_states = self.Ad.shape[0]
        n_inputs = self.Bd.shape[1]

        # Create the optimization problem matrices
        # Quadratic cost matrix
        QN = Q  # Terminal cost
        P = sparse.block_diag([sparse.kron(sparse.eye(self.horizon), Q), 
                               QN,
                               sparse.kron(sparse.eye(self.horizon), R)], format='csc')
        # Linear cost vector
        q = np.hstack([np.kron(np.ones(self.horizon), -Q @ reference), -QN @ reference, np.zeros(self.horizon * n_inputs)])

        # Constraints for system dynamics
        Ax = sparse.kron(sparse.eye(self.horizon + 1), -sparse.eye(n_states)) + sparse.kron(sparse.eye(self.horizon + 1, k=-1), sparse.csc_matrix(self.Ad))
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.horizon)), sparse.eye(self.horizon)]), sparse.csc_matrix(self.Bd))
        A_eq = sparse.hstack([Ax, Bu])
        l_eq = np.hstack([-initial_state, np.zeros(self.horizon * n_states)])
        u_eq = l_eq
        
        # State and input constraints
        A_ineq = sparse.eye((self.horizon + 1) * n_states + self.horizon * n_inputs)
        l_ineq = np.hstack([np.kron(np.ones(self.horizon + 1), self.xmin), np.kron(np.ones(self.horizon), self.umin)])
        u_ineq = np.hstack([np.kron(np.ones(self.horizon + 1), self.xmax), np.kron(np.ones(self.horizon), self.umax)])
        
        # Combine equality and inequality constraints
        A = sparse.vstack([A_eq, A_ineq], format='csc')
        l = np.hstack([l_eq, l_ineq])
        u = np.hstack([u_eq, u_ineq])

        return P, q, A, l, u

    def get_mpc_object(self, initial_state, Q, R, reference):
        P, q, A, l, u = self.setup_optimization_problem(initial_state, reference, sparse.csc_matrix(Q), sparse.csc_matrix(R))

        # Set up the OSQP problem
        prob = osqp.OSQP()
        prob.setup(P=P, q=q, A=A, l=l, u=u, warm_start=True, verbose=False)

        return prob