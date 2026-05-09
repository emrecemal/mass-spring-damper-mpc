# Mass-Spring-Damper System with Model Predictive Control

A comprehensive implementation of Model Predictive Control (MPC) for a mass-spring-damper system. This project demonstrates how to formulate a continuous-time system, discretize it, and solve the optimal control problem using convex optimization.

## Table of Contents
1. [Installation & Setup](#installation--setup)
2. [Quick Start](#quick-start)
3. [System Dynamics](#system-dynamics)
4. [Discretization](#discretization)
5. [Model Predictive Control Formulation](#model-predictive-control-formulation)
6. [Optimization Problem](#optimization-problem)
7. [Project Structure](#project-structure)
8. [Configuration & Parameters](#configuration--parameters)

---

## Installation & Setup

### Prerequisites
- Python 3.7 or higher
- macOS, Linux, or Windows

### Step 1: Create a Virtual Environment

A virtual environment isolates project dependencies from your system Python installation, preventing conflicts with other projects.

```bash
cd /path/to/mass-spring-damper-mpc
python3 -m venv .venv
```

This creates a `.venv` folder containing the isolated Python environment.

### Step 2: Activate the Virtual Environment

**On macOS/Linux:**
```bash
source .venv/bin/activate
```

**On Windows:**
```bash
.venv\Scripts\activate
```

You should see `(.venv)` at the beginning of your terminal prompt when activated.

### Step 3: Install Dependencies

Install all required packages from the `requirements.txt` file:

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

This installs:
- **numpy**: Numerical computing
- **scipy**: Scientific computing (used for discretization)
- **osqp**: Quadratic programming solver
- **matplotlib**: Plotting and visualization

### Step 4: Run the Simulation

Execute the main simulation script:

```bash
python simulate.py
```

**Optional command-line arguments:**
```bash
python simulate.py --mass 1.0 --spring_constant 10.0 --damping_coefficient 0.5 --horizon 20 --dt 0.1 --total_time 10.0
```

---

## Quick Start

The simplest way to get started:

```bash
# Setup
python3 -m venv .venv
source .venv/bin/activate  # macOS/Linux
pip install -r requirements.txt

# Run simulation with default parameters
python simulate.py
```

---

## System Dynamics

### Continuous-Time Model

The mass-spring-damper system is governed by Newton's second law:

$$m \ddot{x} + c \dot{x} + k x = F$$

where:
- $m$ = mass (kg)
- $c$ = damping coefficient (N·s/m)
- $k$ = spring stiffness (N/m)
- $x$ = position of the mass (m)
- $\dot{x}$ = velocity (m/s)
- $\ddot{x}$ = acceleration (m/s²)
- $F$ = applied force (N)

### State-Space Representation

To apply control theory, we convert the second-order differential equation into a first-order system using state variables.

**State vector definition:**
$$\mathbf{x} = \begin{bmatrix} x \\ \dot{x} \end{bmatrix}$$

where:
- $x_1 = x$ is the position
- $x_2 = \dot{x}$ is the velocity

**Input:**
$$u = F$$

**Continuous-time state-space representation:**

$$\dot{\mathbf{x}} = A\mathbf{x} + Bu$$

$$\mathbf{y} = C\mathbf{x} + Du$$

**System matrices:**

$$A = \begin{bmatrix} 0 & 1 \\ -\frac{k}{m} & -\frac{c}{m} \end{bmatrix} \quad B = \begin{bmatrix} 0 \\ \frac{1}{m} \end{bmatrix}$$

$$C = \begin{bmatrix} 1 & 0 \end{bmatrix} \quad D = \begin{bmatrix} 0 \end{bmatrix}$$

**Interpretation:**
- $A$ represents the natural system dynamics (gravity, damping, spring restoring force)
- $B$ is the control input matrix (how force affects acceleration)
- $C$ is the output matrix (we measure position)
- $D$ is the feedthrough matrix (typically zero for mechanical systems)

---

## Discretization

### Why Discretization?

MPC works with discrete-time systems because:
1. Digital computers operate on discrete time steps
2. Control inputs are held constant between sampling intervals
3. Measurements are taken at fixed time intervals

### Zero-Order Hold (ZOH) Method

We discretize the continuous system using the Zero-Order Hold method, which assumes the control input is held constant over the time interval $[t_k, t_{k+1}]$.

For a sampling time $\Delta t$ (time step), the discretized system is:

$$\mathbf{x}_{k+1} = A_d \mathbf{x}_k + B_d u_k$$

$$\mathbf{y}_k = C_d \mathbf{x}_k + D_d u_k$$

**Discretized system matrices:**

$$A_d = e^{A\Delta t}$$

$$B_d = \int_0^{\Delta t} e^{A\tau} d\tau \cdot B$$

The implementation uses `scipy.signal.cont2discrete()` with `method='zoh'` to compute $A_d$ and $B_d$ automatically.

### Discretization Example

For a system with $\Delta t = 0.1$ s, the continuous-time evolution is approximated by discrete jumps at $t = 0, 0.1, 0.2, ...$ seconds.

---

## Model Predictive Control Formulation

### Receding Horizon Concept

MPC solves an optimization problem over a finite prediction horizon $N$ at each time step:

1. **Measure** the current state $\mathbf{x}_0$ at time $k$
2. **Predict** future states $\mathbf{x}_1, ..., \mathbf{x}_N$ based on candidate control inputs
3. **Optimize** control inputs $u_0, ..., u_{N-1}$ to minimize a cost function
4. **Apply** only the first optimal input $u_0$
5. **Shift** forward one time step and repeat

### Optimization Horizon

Define the prediction horizon: $N$ steps into the future

**Optimization variables:** $(3N + 2)$ variables total
- States: $2(N+1)$ variables (position and velocity at steps $0, 1, ..., N$)
- Inputs: $N$ variables (forces at steps $0, 1, ..., N-1$)

Stack the optimization variable:
$$z = \begin{bmatrix} \mathbf{x}_0 \\ \mathbf{x}_1 \\ \vdots \\ \mathbf{x}_N \\ u_0 \\ u_1 \\ \vdots \\ u_{N-1} \end{bmatrix} \in \mathbb{R}^{3N+2}$$

### Cost Function

The MPC objective minimizes tracking error and control effort:

$$J = \sum_{k=0}^{N-1} \left( \|\mathbf{x}_k - \mathbf{r}\|_Q^2 + \|u_k\|_R^2 \right) + \|\mathbf{x}_N - \mathbf{r}\|_Q^2$$

where:
- $\mathbf{r}$ = reference trajectory (desired state)
- $Q$ = state cost matrix (penalizes deviations from reference)
- $R$ = input cost matrix (penalizes large control inputs)
- $\|\mathbf{v}\|_M^2 = \mathbf{v}^T M \mathbf{v}$ = weighted squared norm

**Typical weight matrices:**

$$Q = \begin{bmatrix} Q_x & 0 \\ 0 & Q_v \end{bmatrix}$$

where $Q_x$ is the position weight and $Q_v$ is the velocity weight. For example: $Q = \text{diag}(100, 1)$ prioritizes position tracking.

$$R = [R_f]$$

where $R_f$ is the input (force) weight. For example: $R = [0.01]$ penalizes large forces.

---

## Optimization Problem

### Quadratic Programming Form

We reformulate the MPC problem into a convex quadratic program (QP) suitable for the OSQP solver:

$$\begin{split}
\text{minimize} &\quad \frac{1}{2} z^T P z + q^T z \\
\text{subject to} &\quad l \leq Az \leq u
\end{split}$$

where:
- $z \in \mathbb{R}^{3N+2}$ = optimization variable
- $P \in \mathbb{R}^{(3N+2) \times (3N+2)}$ = positive semidefinite Hessian matrix
- $q \in \mathbb{R}^{3N+2}$ = linear cost vector
- $A \in \mathbb{R}^{m \times (3N+2)}$ = constraint matrix
- $l, u \in \mathbb{R}^m$ = lower and upper constraint bounds

### Quadratic Cost Term

From the MPC cost function, the Hessian and linear cost are:

$$P = \begin{bmatrix}
Q & 0 & 0 \\
0 & \ddots & 0 \\
0 & 0 & R
\end{bmatrix} = \text{block-diag}(Q, ..., Q, Q_N, R, ..., R)$$

$$q = \begin{bmatrix}
-Q\mathbf{r} \\
\vdots \\
-Q_N\mathbf{r} \\
0 \\
\vdots \\
0
\end{bmatrix}$$

where $Q_N$ is the terminal cost (final state penalty).

### System Dynamics Constraints

The dynamics $\mathbf{x}_{k+1} = A_d \mathbf{x}_k + B_d u_k$ must be satisfied for all $k = 0, ..., N-1$.

Rearranging: $\mathbf{x}_{k+1} - A_d \mathbf{x}_k - B_d u_k = 0$

In matrix form:
$$\begin{bmatrix}
-I & 0 & 0 & 0 & \cdots & B_d \\
A_d & -I & 0 & 0 & \cdots & 0 \\
0 & A_d & -I & 0 & \cdots & 0 \\
\vdots & & \ddots & \ddots & & \vdots \\
0 & 0 & \cdots & A_d & -I & 0
\end{bmatrix} z = \begin{bmatrix} -\mathbf{x}_0 \\ 0 \\ 0 \\ \vdots \\ 0 \end{bmatrix}$$

These are **equality constraints** ($l_{eq} = u_{eq}$).

### State and Input Bound Constraints

Each state and input is constrained:

$$x_{\min} \leq x_k \leq x_{\max}, \quad k = 0, ..., N$$

$$u_{\min} \leq u_k \leq u_{\max}, \quad k = 0, ..., N-1$$

In matrix form:
$$\begin{bmatrix}
I & 0 \\
0 & I
\end{bmatrix} z \leq \begin{bmatrix}
x_{\max} \\ u_{\max}
\end{bmatrix}$$

$$\begin{bmatrix}
I & 0 \\
0 & I
\end{bmatrix} z \geq \begin{bmatrix}
x_{\min} \\ u_{\min}
\end{bmatrix}$$

These are **inequality constraints** expressed as $l \leq Az \leq u$.

### Combined Constraint Matrix

The full constraint matrix combines dynamics (equality) and bounds (inequality):

$$\text{All constraints} = \begin{bmatrix} A_{eq} \\ A_{ineq} \end{bmatrix}, \quad \text{bounds} = \begin{bmatrix} l_{eq} \leq A_{eq}z \leq u_{eq} \\ l_{ineq} \leq A_{ineq}z \leq u_{ineq} \end{bmatrix}$$

### Solving with OSQP

The OSQP solver efficiently solves this QP in real-time:

1. Takes $P$, $q$, $A$, $l$, $u$ as inputs
2. Solves for optimal $z^*$
3. Returns optimal state and input trajectories
4. We apply only the first input: $u^*_0$

---

## Project Structure

```
mass-spring-damper-mpc/
├── model.py                         # System model and simulation
├── model_predictive_controller.py   # MPC formulation and QP setup
├── simulate.py                      # Main simulation script
├── requirements.txt                 # Python dependencies
└── README.md                        # This file
```

### File Descriptions

**[model.py](model.py)**
- `MassSpringDamperModel` class: Defines system dynamics
- `state_space_representation()`: Generates continuous-time matrices $A, B, C, D$
- `get_discrete_state_space()`: Discretizes to $A_d, B_d$ using ZOH
- `simulate_one_step()`: Advances system by one time step
- `simulate()`: Runs open-loop simulation
- `plot_response()`: Visualizes position, velocity, and input

**[model_predictive_controller.py](model_predictive_controller.py)**
- `ModelPredictiveController` class: Implements MPC algorithm
- `set_state_bounds()`: Defines $x_{\min}, x_{\max}$
- `set_input_bounds()`: Defines $u_{\min}, u_{\max}$
- `setup_optimization_problem()`: Constructs $P, q, A, l, u$ matrices
- `get_mpc_object()`: Returns ready-to-solve OSQP problem instance

**[simulate.py](simulate.py)**
- Demonstrates closed-loop MPC control
- Initializes system parameters (mass, spring constant, damping)
- Runs MPC at each time step
- Solves QP and applies optimal control input
- Plots closed-loop response

---

## Configuration & Parameters

### System Parameters

In [simulate.py](simulate.py), configure the physical system:

```python
parser.add_argument('--mass', type=float, default=1.0, 
                    help='Mass of the system (kg)')
parser.add_argument('--spring_constant', type=float, default=10.0, 
                    help='Spring constant (N/m)')
parser.add_argument('--damping_coefficient', type=float, default=0.5, 
                    help='Damping coefficient (N·s/m)')
```

### MPC Parameters

**Control horizon:**
```python
parser.add_argument('--horizon', type=int, default=20, 
                    help='MPC horizon (number of prediction steps)')
```

Larger horizons lead to better control performance but higher computational cost.

**Sampling time:**
```python
parser.add_argument('--dt', type=float, default=0.1, 
                    help='Time step for discretization (s)')
```

Smaller time steps provide finer control but increase computation.

**Simulation duration:**
```python
parser.add_argument('--total_time', type=float, default=10.0, 
                    help='Total simulation time (s)')
```

### Cost Function Weights

In [simulate.py](simulate.py):

```python
Q = np.diag([100.0, 1.0])   # State cost: [position, velocity]
R = np.diag([0.01])         # Input cost: [force]
```

- **Increase $Q$**: Prioritize tracking accuracy
- **Increase $R$**: Reduce control effort, smoother inputs
- **Position weight (100.0)** > **Velocity weight (1.0)**: Emphasize position control

### Constraints

In [simulate.py](simulate.py):

```python
mpc.set_input_bounds(umin=-10.0, umax=10.0)  # Force limits (N)
mpc.set_state_bounds(xmin=np.array([-np.inf, 0.0]), 
                     xmax=np.array([1.0, 0.5]))  # Position and velocity limits
```

- **Input bounds**: Physical actuator limits
- **Position bounds**: Workspace constraints (e.g., mechanical stops)
- **Velocity bounds**: Safety or physical constraints
- **Set to $\pm\infty$**: No constraint on that variable

---

## Key Concepts for Control Engineers

### Receding Horizon

At each time step, MPC optimizes over a finite horizon and applies only the first control input. The "window" shifts forward at each step, hence "receding horizon."

### Stability

Terminal cost term ($Q_N$) at the final prediction step improves closed-loop stability. This penalizes final state deviation more heavily.

### Real-Time Implementation

The OSQP solver is fast enough for real-time MPC on typical embedded systems. Computation time scales with horizon length $N$ and constraint count.

### Soft vs. Hard Constraints

- **Hard constraints**: Strictly enforced (state/input bounds). Violations cause solver failure.
- **Soft constraints**: Could be implemented via slack variables (not included in this basic version).

---

## References

- Boyd & Parikh (2014). Convex Optimization. Cambridge University Press.
- Maciejowski, J. M. (2002). Predictive Control with Constraints. Pearson.
- OSQP documentation: https://osqp.org/

