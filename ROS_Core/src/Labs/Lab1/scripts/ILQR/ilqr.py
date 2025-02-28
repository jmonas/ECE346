from typing import Tuple, Optional, Dict, Union
from jaxlib.xla_extension import ArrayImpl as DeviceArray
import time
import os
import numpy as np
import jax
from .dynamics import Bicycle5D
from .cost import Cost, CollisionChecker, Obstacle
from .ref_path import RefPath
from .config import Config
import time

status_lookup = ['Iteration Limit Exceed',
                'Converged',
                'Failed Line Search']

class ILQR():
	def __init__(self, config_file = None) -> None:

		self.config = Config()  # Load default config.
		if config_file is not None:
			self.config.load_config(config_file)  # Load config from file.
		
		self.load_parameters()
		print('ILQR setting:', self.config)

		# Set up Jax parameters
		jax.config.update('jax_platform_name', self.config.platform)
		print('Jax using Platform: ', jax.lib.xla_bridge.get_backend().platform)

		# If you want to use GPU, lower the memory fraction from 90% to avoid OOM.
		os.environ['XLA_PYTHON_CLIENT_MEM_FRACTION'] = '20'

		self.dyn = Bicycle5D(self.config)
		self.cost = Cost(self.config)
		self.ref_path = None

		# collision checker
		# Note: This will not be used until lab2.
		self.collision_checker = CollisionChecker(self.config)
		self.obstacle_list = []
		
		# Do a dummy run to warm up the jitted functions.
		self.warm_up()

	def load_parameters(self):
		'''
		This function defines ILQR parameters from <self.config>.
		'''
		# ILQR parameters
		self.dim_x = self.config.num_dim_x
		self.dim_u = self.config.num_dim_u
		self.T = int(self.config.T)
		self.dt = float(self.config.dt)
		self.max_iter = int(self.config.max_iter)
		self.tol = float(self.config.tol)  # ILQR update tolerance.

		# line search parameters.
		self.alphas = self.config.line_search_base**(
						np.arange(self.config.line_search_a,
                        self.config.line_search_b,
                        self.config.line_search_c)
                    )

		print('Line Search Alphas: ', self.alphas)

		# regularization parameters
		self.reg_min = float(self.config.reg_min)
		self.reg_max = float(self.config.reg_max)
		self.reg_init = float(self.config.reg_init)
		self.reg_scale_up = float(self.config.reg_scale_up)
		self.reg_scale_down = float(self.config.reg_scale_down)
		self.max_attempt = self.config.max_attempt
		
	def warm_up(self):
		'''
		Warm up the jitted functions.
		'''
		# Build a fake path as a 1 meter radius circle.
		theta = np.linspace(0, 2 * np.pi, 100)
		centerline = np.zeros([2, 100])
		centerline[0,:] = 1 * np.cos(theta)
		centerline[1,:] = 1 * np.sin(theta)

		self.ref_path = RefPath(centerline, 0.5, 0.5, 1, True)

		# add obstacle
		obs = np.array([[0, 0, 0.5, 0.5], [1, 1.5, 1, 1.5]]).T
		obs_list = [[obs for _ in range(self.T)]]
		self.update_obstacles(obs_list)

		x_init = np.array([0.0, -1.0, 1, 0, 0])
		print('Start warm up ILQR...')
		self.plan(x_init)
		print('ILQR warm up finished.')
		
		self.ref_path = None
		self.obstacle_list = []

	def update_ref_path(self, ref_path: RefPath):
		'''
		Update the reference path.
		Args:
			ref_path: RefPath: reference path.
		'''
		self.ref_path = ref_path

	def update_obstacles(self, vertices_list: list):
		'''
		Update the obstacle list for a list of vertices.
		Args:
			vertices_list: list of np.ndarray: list of vertices for each obstacle.
		'''
		# Note: This will not be used until lab2.
		self.obstacle_list = []
		for vertices in vertices_list:
			self.obstacle_list.append(Obstacle(vertices))

	def get_references(self, trajectory: Union[np.ndarray, DeviceArray]):
		'''
		Given the trajectory, get the path reference and obstacle information.
		Args:
			trajectory: [num_dim_x, T] trajectory.
		Returns:
			path_refs: [num_dim_x, T] np.ndarray: references.
			obs_refs: [num_dim_x, T] np.ndarray: obstacle references.
		'''
		trajectory = np.asarray(trajectory)
		path_refs = self.ref_path.get_reference(trajectory[:2, :])
		obs_refs = self.collision_checker.check_collisions(trajectory, self.obstacle_list)
		return path_refs, obs_refs

	def Backward_Pass_Robust(self, trajectory, controls, path_refs, obs_refs, lamb):
		q, r, Q, R, H = self.cost.get_derivatives_np(trajectory, controls, path_refs, obs_refs)
		A, B = self.dyn.get_jacobian_np(trajectory, controls)

		p, P = q[:,self.T-1], Q[:,:,self.T-1]
		t = self.T - 2

		k_open_loop = np.zeros((2, self.T))
		K_closed_loop = np.zeros((2, 5, self.T))

		while t >= 0:
			Q_x = q[:,t] + A[:,:,t].T @ p
			Q_u = r[:,t] + B[:,:,t].T @ p
			Q_xx = Q[:,:,t] + A[:,:,t].T @ P @ A[:,:,t]
			Q_uu = R[:,:,t] + B[:,:,t].T @ P @ B[:,:,t]
			Q_ux = H[:,:,t] + B[:,:,t].T @ P @ A[:,:,t]

			Q_uu_reg = R[:,:,t] + B[:,:,t].T @ (P+lamb*np.eye(5)) @ B[:,:,t]
			Q_ux_reg = H[:,:,t] + B[:,:,t].T @ (P+lamb*np.eye(5)) @ A[:,:,t]

			if np.isnan(Q_uu_reg).any() or np.isinf(Q_uu_reg).any():
				continue
			elif not np.all(np.linalg.eigvals(Q_uu_reg) > 0) and lamb < self.max_attempt:
				lamb *= self.reg_scale_up
				if lamb > self.reg_max:
					raise Exception("Convergence failure within backward pass.")
				t = self.T-2
				p = q[:,self.T-1]
				P = Q[:,:,self.T-1]
				continue

			k = -np.linalg.inv(Q_uu_reg)@Q_u
			K = -np.linalg.inv(Q_uu_reg)@Q_ux_reg
			k_open_loop[:,t] = k
			K_closed_loop[:, :, t] = K

			p = Q_x + K.T @ Q_uu @ k + K.T@Q_u + Q_ux.T@k
			P = Q_xx + K.T @ Q_uu @ K + K.T@Q_ux + Q_ux.T@K
			t -= 1

		return K_closed_loop, k_open_loop, lamb*self.reg_scale_down

	def Forward_Pass(self, trajectory, controls, K_closed_loop, k_open_loop, alpha):
		x = np.zeros_like(trajectory)
		u = np.zeros_like(controls)
		x[:,0] = trajectory[:,0]

		for t in range(self.T-1):
			K = K_closed_loop[:,:,t]
			k = k_open_loop[:,t]
			u[:,t] = controls[:,t]+alpha*k+ K @ (x[:, t] - trajectory[:, t])
			x[:,t+1], u[:,t+1] = self.dyn.integrate_forward_np(x[:,t], u[:,t])

		return x, u

	def plan(self, init_state: np.ndarray,
				controls: Optional[np.ndarray] = None) -> Dict:
		'''
		Main ILQR loop.
		Args:
			init_state: [num_dim_x] np.ndarray: initial state.
			control: [num_dim_u, T] np.ndarray: initial control.
		Returns:
			A dictionary with the following keys:
				status: int: -1 for failure, 0 for success. You can add more status if you want.
				t_process: float: time spent on planning.
				trajectory: [num_dim_x, T] np.ndarray: ILQR planned trajectory.
				controls: [num_dim_u, T] np.ndarray: ILQR planned controls sequence.
				K_closed_loop: [num_dim_u, num_dim_x, T] np.ndarray: closed loop gain.
				k_closed_loop: [num_dim_u, T] np.ndarray: closed loop bias.
		'''

		# We first check if the planner is ready
		if self.ref_path is None:
			print('No reference path is provided.')
			return dict(status=-1)

		# if no initial control sequence is provided, we assume it is all zeros.
		if controls is None:
			controls =np.zeros((self.dim_u, self.T))
		else:
			assert controls.shape[1] == self.T

		# Start timing
		t_start = time.time()

		# Rolls out the nominal trajectory and gets the initial cost.
		trajectory, controls = self.dyn.rollout_nominal_np(init_state, controls)

		# Get path and obstacle references based on your current nominal trajectory.
		# Note: you will NEED TO call this function and get new references at each iteration.
		path_refs, obs_refs = self.get_references(trajectory)

		# Get the initial cost of the trajectory.
		J = self.cost.get_traj_cost(trajectory, controls, path_refs, obs_refs)

		##########################################################################
		# TODO 1: Implement the ILQR algorithm. Feel free to add any helper functions.
		# You will find following implemented functions useful:

		# ******** Functions to compute the Jacobians of the dynamics  ************
		# A, B = self.dyn.get_jacobian_np(trajectory, controls)

		# Returns the linearized 'A' and 'B' matrix of the ego vehicle around
		# nominal trajectory and controls.

		# Args:
		# 	trajectory: np.ndarray, (dim_x, T) trajectory along the nominal trajectory.
		# 	controls: np.ndarray, (dim_u, T) controls along the trajectory.

		# Returns:
		# 	A: np.ndarray, (dim_x, T) the Jacobian of the dynamics w.r.t. the state.
		# 	B: np.ndarray, (dim_u, T) the Jacobian of the dynamics w.r.t. the control.
		
		# ******** Functions to roll the dynamics for one step  ************
		# state_next, control_clip = self.dyn.integrate_forward_np(state, control)
		
		# Finds the next state of the vehicle given the current state and
		# control input.

		# Args:
		# 	state: np.ndarray, (dim_x).
		# 	control: np.ndarray, (dim_u).

		# Returns:
		# 	state_next: np.ndarray, (dim_x) next state.
		# 	control_clip: np.ndarray, (dim_u) clipped control.
		
		# *** Functions to get total cost of a trajectory and control sequence  ***
		# J = self.cost.get_traj_cost(trajectory, controls, path_refs, obs_refs)
		# Given the trajectory, control seq, and references, return the sum of the cost.
		# Input:
		# 	trajectory: (dim_x, T) array of state trajectory
		# 	controls:   (dim_u, T) array of control sequence
		# 	path_refs:  (dim_ref, T) array of references (e.g. reference path, reference velocity, etc.)
		# 	obs_refs: *Optional* (num_obstacle, (2, T)) List of obstacles. Default to None
		# return:
		# 	cost: float, sum of the running cost over the trajectory

  		# ******** Functions to get jacobian and hessian of the cost ************
		# q, r, Q, R, H = self.cost.get_derivatives_np(trajectory, controls, path_refs, obs_refs)
		
		# Given the trajectory, control seq, and references, return Jacobians and Hessians of cost function
		# Input:
		# 	trajectory: (dim_x, T) array of state trajectory
		# 	controls:   (dim_u, T) array of control sequence
		# 	path_refs:  (dim_ref, T) array of references (e.g. reference path, reference velocity, etc.)
		# 	obs_refs: *Optional* (num_obstacle, (2, T)) List of obstacles. Default to None
		# return:
		# 	q: np.ndarray, (dim_x, T) jacobian of cost function w.r.t. states
        #   r: np.ndarray, (dim_u, T) jacobian of cost function w.r.t. controls
        #   Q: np.ndarray, (dim_x, dim_u, T) hessian of cost function w.r.t. states
        #   R: np.ndarray, (dim_u, dim_u, T) hessian of cost function w.r.t. controls
        #   H: np.ndarray, (dim_x, dim_u, T) hessian of cost function w.r.t. states and controls

		lamb = self.reg_init
		converged = False
		for i in range(self.max_iter):
			path_refs, obs_refs = self.get_references(trajectory)
			K_closed_loop, k_open_loop, lamb = self.Backward_Pass_Robust(trajectory, controls, path_refs, obs_refs, lamb)
			changed = False
			for alpha in self.alphas :
				trajectory_new, controls_new = self.Forward_Pass(trajectory, controls, K_closed_loop, k_open_loop, alpha)
				path_refs_new, obs_refs_new = self.get_references(trajectory_new)
				J_new = self.cost.get_traj_cost(trajectory_new, controls_new, path_refs_new, obs_refs_new)

				#print(f"J_new: {J_new}; J: {J}") #commented by rob

				if J_new<=J:
					if np.abs(J - J_new) < self.tol:
						converged = True
					J = J_new
					trajectory = trajectory_new
					controls = controls_new
					changed = True
					break
			if not changed:
				print("line search failed with reg = ", lamb, " at step ", i)
				status = -1
				break
			if converged:
				print("converged after ", i, " steps.")
				status = 0
				break

		########################### #END of TODO 1 #####################################

		t_process = time.time() - t_start
		solver_info = dict(
				t_process=t_process, # Time spent on planning
				trajectory = trajectory,
				controls = controls,
				status=status,
				K_closed_loop=K_closed_loop,
				k_open_loop=k_open_loop
				# Optional TODO: Fill in other information you want to return
		)
		return solver_info



