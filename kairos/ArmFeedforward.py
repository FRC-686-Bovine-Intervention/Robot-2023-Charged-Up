# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

from dataclasses import dataclass

from casadi import *

from DCMotor import DCMotor


@dataclass
class JointConfig:
    mass: float
    length: float
    moi: float
    cgRadius: float
    motor: DCMotor


class ArmFeedforward:
    _g = 9.80665
    _proximal: JointConfig
    _distal: JointConfig

    def __init__(self, proximal: JointConfig, distal: JointConfig):
        self._proximal = proximal
        self._distal = distal

    def calculate(self, position, velocity, acceleration):
        M = [[0, 0], [0, 0]]
        C = [[0, 0], [0, 0]]
        Tg = [0, 0]

        M[0][0] = (
            self._proximal.mass * (self._proximal.cgRadius**2.0)
            + self._distal.mass
            * ((self._proximal.length**2.0) + (self._distal.cgRadius**2.0))
            + self._proximal.moi
            + self._distal.moi
            + 2
            * self._distal.mass
            * self._proximal.length
            * self._distal.cgRadius
            * cos(position[1])
        )
        M[1][0] = (
            self._distal.mass * (self._distal.cgRadius**2)
            + self._distal.moi
            + self._distal.mass
            * self._proximal.length
            * self._distal.cgRadius
            * cos(position[1])
        )
        M[0][1] = (
            self._distal.mass * (self._distal.cgRadius**2)
            + self._distal.moi
            + self._distal.mass
            * self._proximal.length
            * self._distal.cgRadius
            * cos(position[1])
        )
        M[1][1] = self._distal.mass * (self._distal.cgRadius**2) + self._distal.moi

        C[0][0] = (
            -self._distal.mass
            * self._proximal.length
            * self._distal.cgRadius
            * sin(position[1])
            * velocity[1]
        )
        C[1][0] = (
            self._distal.mass
            * self._proximal.length
            * self._distal.cgRadius
            * sin(position[1])
            * velocity[0]
        )
        C[0][1] = (
            -self._distal.mass
            * self._proximal.length
            * self._distal.cgRadius
            * sin(position[1])
            * (velocity[0] + velocity[1])
        )

        Tg[0] = (
            self._proximal.mass * self._proximal.cgRadius
            + self._distal.mass * self._proximal.length
        ) * self._g * cos(
            position[0]
        ) + self._distal.mass * self._distal.cgRadius * self._g * cos(
            position[0] + position[1]
        )
        Tg[1] = (
            self._distal.mass
            * self._distal.cgRadius
            * self._g
            * cos(position[0] + position[1])
        )

        M_times_acceleration = (
            M[0][0] * acceleration[0] + M[0][1] * acceleration[1],
            M[1][0] * acceleration[0] + M[1][1] * acceleration[1],
        )
        C_times_velocity = (
            C[0][0] * velocity[0] + C[0][1] * velocity[1],
            C[1][0] * velocity[0] + C[1][1] * velocity[1],
        )
        torque = (
            M_times_acceleration[0] + C_times_velocity[0] + Tg[0],
            M_times_acceleration[1] + C_times_velocity[1] + Tg[1],
        )
        return (
            self._proximal.motor.getVoltage(torque[0], velocity[0]),
            self._distal.motor.getVoltage(torque[1], velocity[1]),
        )
