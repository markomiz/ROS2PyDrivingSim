# Authors : Gastone Pietro Rosati Papini
# Date    : 09/08/2022
# License : MIT


# Authors : Edited by Marko Mizdrak
# Date    : 15/10/2023
# License : MIT

import pygame, math
import numpy as np

from vehicle_model.vehicle_params import VehicleParams
from vehicle_model.pacejka_params import PacejkaParam
from vehicle_model.vehicle_double_track_model import vehicle_double_track_model

class Vehicle():

    def __init__( self ):

        self.vehicle = VehicleParams()
        self.pacejka = PacejkaParam()

        self.clock = None
        self.state = None

        self.action = (0.1,0)
        self.reset()
        self.dt = 0.001

    def reset( self ):
        #Initial condition of the vehicle
        X = np.zeros(24)
        X[0] = 0
        X[1] = 0
        X[2] = 0
        X[6] = 906.97882  # [N] vertical force for the rear right wheel
        X[7] = 906.97882  # [N] vertical force for the rear left wheel
        X[8] = 638.09618  # [N] vertical force for the front right wheel
        X[9] = 638.09618  # [N] vertical force for the front left wheel

        self.t = 0
        self.dX = np.zeros(24)
        self.state = X

    def set_pos_ang(self, point_angle: tuple):
        self.state[0] = point_angle[0]
        self.state[1] = point_angle[1]
        self.state[2] = point_angle[2]

    def __discrete(self, x0, dt, dx0=None, actions=None):
        dx = np.asarray(self.__derivs_dynamics(x0, dx0, actions))
        x = x0 + dt * dx
        return x, dx

    def __derivs_dynamics(self, X, dX, actions):
        pedal_req = actions[0]
        delta_req = actions[1]
        next_dX, extra_params = vehicle_double_track_model(dX, X, pedal_req, delta_req, self.pacejka, self.vehicle)
        return next_dX

    def update(self):
        self.t = self.t + self.dt
        next_state, next_dX = self.__discrete(self.state, self.dt, self.dX, self.action)

        self.state = next_state
        self.dX = next_dX # potrei cambiarlo in qualcosa del tipo (next_state - self.state)/dt

    def control(self, action):
        self.action = action

    def get_state(self):
        return self.state, self.dX