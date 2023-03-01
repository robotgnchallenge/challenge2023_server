#!/usr/bin/env python
from sys import path
from casadi import *
import time
import rospy


class KinMPCPathFollower(object):

    def __init__(self,
                 N=15,     # timesteps in MPC Horizon
                 DT=0.1,    # discretization time between timesteps (s)
                 V_MIN=0.0,    # min/max velocity constraint (m/s)
                 V_MAX=2.35,
                 A_MIN=-0.5,   # min/max acceleration constraint (m/s^2)
                 A_MAX=0.5,
                 # min/max front steer angle constraint (rad/s)
                 ANGVEL_MIN=-0.6,
                 ANGVEL_MAX=0.6,
                 # min/max front steer angle rate constraint (rad/s^2)
                 ANGVEL_DOT_MIN=-0.5,
                 ANGVEL_DOT_MAX=0.5,
                 Q=[4., 4.],
                 R=[3., 3.]):
        for key in list(locals()):
            if key == 'self':
                pass
            elif key == 'Q':
                self.Q = casadi.diag(Q)
            elif key == 'R':
                self.R = casadi.diag(R)
            else:
                setattr(self, '%s' % key, locals()[key])

        self.opti = casadi.Opti()
        # print("*************************mpc speed configuration: *************************************")
        # print("Min speed: ", self.V_MIN)
        # print("Max speed: ", self.V_MAX)
        # print("Min rotation speed: ", self.ANGVEL_MIN)
        # print("Max rotation speed: ", self.ANGVEL_MAX)

        ''' 
        (1) Parameters
        '''
        self.u_prev = self.opti.parameter(
            2)  # previous input: [u_{linear_vel, -1}, u_{angular_vel, -1}]
        # current state:  [x_0, y_0, psi_0]
        self.z_curr = self.opti.parameter(3)
        # Reference trajectory we would like to follow.
        # First index corresponds to our desired state at timestep k+1:
        #   i.e. z_ref[0,:] = z_{desired, 1}.
        # Second index selects the state element from [x_k, y_k].
        self.z_ref = self.opti.parameter(self.N, 2)
        self.x_ref = self.z_ref[:, 0]
        self.y_ref = self.z_ref[:, 1]

        '''
        (2) Decision Variables
        '''
        # Actual trajectory we will follow given the optimal solution.
        # First index is the timestep k, i.e. self.z_dv[0,:] is z_0.
        # It has self.N+1 timesteps since we go from z_0, ..., z_self.N.
        # Second index is the state element, as detailed below.
        self.z_dv = self.opti.variable(self.N+1, 3)
        self.x_dv = self.z_dv[:, 0]
        self.y_dv = self.z_dv[:, 1]
        self.psi_dv = self.z_dv[:, 2]

        # Control inputs used to achieve self.z_dv according to dynamics.
        # First index is the timestep k, i.e. self.u_dv[0,:] is u_0.
        # Second index is the input element as detailed below.
        self.u_dv = self.opti.variable(self.N, 2)
        self.linear_vel_dv = self.u_dv[:, 0]
        self.angular_vel_dv = self.u_dv[:, 1]

        # Slack variables used to relax input rate constraints.
        # Matches self.u_dv in structure but timesteps range from -1, ..., N-1.
        self.sl_dv = self.opti.variable(self.N, 2)
        self.sl_linear_vel_dv = self.sl_dv[:, 0]
        self.sl_angular_vel_dv = self.sl_dv[:, 1]
        '''
        (3) Problem Setup: Constraints, Cost, Initial Solve
        '''
        # self._add_constraints()

        # self._add_cost()

    def _add_constraints(self):
        # Initial State Constraint
        self.opti.subject_to(self.x_dv[0] == self.z_curr[0])
        self.opti.subject_to(self.y_dv[0] == self.z_curr[1])
        self.opti.subject_to(self.psi_dv[0] == self.z_curr[2])
        # State kinematics Constraints
        for i in range(self.N):
            self.opti.subject_to(self.x_dv[i+1] == self.x_dv[i] + self.DT * (
                self.linear_vel_dv[i] * casadi.cos(self.psi_dv[i])))
            self.opti.subject_to(self.y_dv[i+1] == self.y_dv[i] + self.DT * (
                self.linear_vel_dv[i] * casadi.sin(self.psi_dv[i])))
            self.opti.subject_to(
                self.psi_dv[i+1] == self.psi_dv[i] + self.DT * (self.angular_vel_dv[i]))

        # Input Bound Constraints
        self.opti.subject_to(self.opti.bounded(
            self.V_MIN,  self.linear_vel_dv, self.V_MAX))
        self.opti.subject_to(self.opti.bounded(
            self.ANGVEL_MIN, self.angular_vel_dv,  self.ANGVEL_MAX))

        # Input Rate Bound Constraints
        self.opti.subject_to(self.opti.bounded(self.A_MIN - self.sl_linear_vel_dv[0],
                                               self.linear_vel_dv[0] -
                                               self.u_prev[0],
                                               self.A_MAX + self.sl_linear_vel_dv[0]))
        self.opti.subject_to(self.opti.bounded(self.ANGVEL_DOT_MIN - self.sl_angular_vel_dv[0],
                                               self.angular_vel_dv[0] -
                                               self.u_prev[1],
                                               self.ANGVEL_DOT_MAX + self.sl_angular_vel_dv[0]))

        for i in range(self.N - 1):
            self.opti.subject_to(self.opti.bounded(self.A_MIN - self.sl_linear_vel_dv[i+1],
                                                   self.linear_vel_dv[i+1] -
                                                   self.linear_vel_dv[i],
                                                   self.A_MAX + self.sl_linear_vel_dv[i+1]))
            self.opti.subject_to(self.opti.bounded(self.ANGVEL_DOT_MIN - self.sl_angular_vel_dv[i+1],
                                                   self.angular_vel_dv[i+1] -
                                                   self.angular_vel_dv[i],
                                                   self.ANGVEL_DOT_MAX + self.sl_angular_vel_dv[i+1]))
        # Other Constraints
        self.opti.subject_to(0 <= self.sl_angular_vel_dv)
        self.opti.subject_to(0 <= self.sl_linear_vel_dv)

    def _add_cost(self):
        def _quad_form(z, Q):
            return casadi.mtimes(z, casadi.mtimes(Q, z.T))

        cost = 0
        for i in range(self.N):
            cost += _quad_form(self.z_dv[i+1, :2] - self.z_ref[i, :2], self.Q)
        for i in range(self.N - 1):
            cost += _quad_form(self.u_dv[i+1, :] - self.u_dv[i, :], self.R)
        cost += (casadi.sum1(self.sl_angular_vel_dv) +
                 casadi.sum1(self.sl_linear_vel_dv))
        self.opti.minimize(cost)

    def solve(self, z_dv_warm_start=None, u_dv_warm_start=None, sl_dv_warm_start=None):
        '''
        (3) Problem Setup: Constraints, Cost, Initial Solve
        '''
        self._add_constraints()

        self._add_cost()
        # Warm Start used if provided.  Else I believe the problem is solved from scratch with initial values of 0.
        if z_dv_warm_start is not None:
            self.opti.set_initial(self.z_dv, z_dv_warm_start)
        if u_dv_warm_start is not None:
            self.opti.set_initial(self.u_dv, u_dv_warm_start)
        if sl_dv_warm_start is not None:
            self.opti.set_initial(self.sl_dv, sl_dv_warm_start)

        st = time.time()
        try:
            sol = self.opti.solve()
            # Optimal solution.
            u_opt = sol.value(self.u_dv)
            z_opt = sol.value(self.z_dv)
            sl_opt = sol.value(self.sl_dv)
            z_ref = sol.value(self.z_ref)
            is_opt = True
        except:
            # Suboptimal solution.
            u_opt = self.opti.debug.value(self.u_dv)
            z_opt = self.opti.debug.value(self.z_dv)
            sl_opt = self.opti.debug.value(self.sl_dv)
            z_ref = self.opti.debug.value(self.z_ref)

            is_opt = False
        solve_time = time.time() - st
        return is_opt, solve_time, u_opt, z_opt, sl_opt, z_ref

    def update_initial_condition(self, x0, y0, psi0):
        self.opti.set_value(self.z_curr, [x0, y0, psi0])

    def update_reference(self, x_ref, y_ref):
        self.opti.set_value(self.x_ref,   x_ref)
        self.opti.set_value(self.y_ref,   y_ref)

    def update_previous_input(self, linear_vel_prev, angular_vel_prev):
        self.opti.set_value(self.u_prev, [linear_vel_prev, angular_vel_prev])


T1 = None
T2 = None

if __name__ == '__main__':
    kmpc = KinMPCPathFollower()
    init_x = -5.41759352243
    init_y = 20.0342560426
    init_psi = 1.22081750918
    # -5.41759352243 2.67927727701 5.54059653538
    x_ref = [46.99376743, 46.98846906, 46.98410712, 46.95793464, 46.91952949, 46.84722297,
             46.77540328, 46.7045001,  46.63148584, 46.55697878]
    y_ref = [20.19980563, 20.29966077, 20.3995656, 20.49562463, 20.58580769, 20.65488567,
             20.72446308, 20.79498074, 20.8632871, 20.92991736]
    psi_ref = [1.39619526, 1.56272013, 1.72128645, 1.87985278, 2.0350475, 2.17652801,
               2.28785472, 2.3424132, 2.39674545, 2.45100124]

    rospy.init_node("mpc_node_test")

    while not rospy.is_shutdown():
        #global T1, T2
        T1 = time.time()
        if not T2 == None:
            rospy.loginfo("pub loop duration: %s ms" % ((T1-T2)*1000))
        rospy.loginfo("mpc received reference")
        kmpc.update_initial_condition(init_x, init_y, init_psi)
        kmpc.update_reference(x_ref, y_ref)
        kmpc.update_previous_input(0., 0.)
        p_opts = {'expand': True}
        s_opts = {'max_cpu_time': 0.1, 'print_level': 0}
        kmpc.opti.solver('ipopt', p_opts, s_opts)
        is_opt, solve_time, u_opt, z_opt, sl_opt, z_ref = kmpc.solve()
        rospy.loginfo("Solve Status: %s, Linear velocity: %.3f, Angular velocity: %.3f, Solve time: %.3f" % (
            is_opt, u_opt[0, 0], u_opt[0, 1], solve_time))

        if(is_opt):
            print("Here is the mpc path: ")
            print(z_opt)
            print("Here is the control commands: ")
            print(u_opt)
            print("Here is the reference path: ")
            print(z_ref)
        # loop_rate.sleep()
        T2 = time.time()
        rospy.loginfo("pub loop total time: %s ms" % ((T2-T1)*1000))
