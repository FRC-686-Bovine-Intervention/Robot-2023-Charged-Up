# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import math

from casadi import *

from ArmKinematics import ArmKinematics
from ArmFeedforward import ArmFeedforward, JointConfig
from DCMotor import DCMotor


# noinspection PyTypeChecker
class Solver:
    def __init__(self, arm_config: str, constraints: str, solver_config: str, silence: bool = False):
        self._arm_config = arm_config
        self._constraints = constraints
        self._solver_config = solver_config

        self._arm_kinematics = ArmKinematics(arm_config["shoulder"][0], arm_config["shoulder"][1], arm_config["proximal"]["length"], arm_config["distal"]["length"])

        # Create solver
        opti = Opti()
        self._opti = opti
        if silence:
            opti.solver("ipopt", {"print_time": 0}, {"print_level": 0, "sb": "yes"})
        else:
            opti.solver("ipopt")

        # Get constants from solver_config
        n = solver_config["solver"]["interiorPoints"]
        max_voltage = solver_config["solver"]["maxVoltage"]
        max_jerk = solver_config["solver"]["maxJerk"]

        # adjust distal joint to attach grabber at same angle
        distal_cg_radius = (
            arm_config["distal"]["cgRadius"] * arm_config["distal"]["mass"]
            + (arm_config["distal"]["length"] + arm_config["grabber"]["cgRadius"])
            * arm_config["grabber"]["mass"]
        ) / (arm_config["distal"]["mass"] + arm_config["grabber"]["mass"])
        distal_moi = arm_config["distal"]["mass"] * math.pow(
            arm_config["distal"]["cgRadius"] - distal_cg_radius, 2.0
        ) + arm_config["grabber"]["mass"] * math.pow(
            arm_config["distal"]["length"] + arm_config["grabber"]["cgRadius"] - distal_cg_radius,
            2.0,
        )
        ff_model = ArmFeedforward(
            JointConfig(
                arm_config["proximal"]["mass"],
                arm_config["proximal"]["length"],
                arm_config["proximal"]["moi"],
                arm_config["proximal"]["cgRadius"],
                self._get_motor(arm_config["proximal"]["motor"]),
            ),
            JointConfig(
                arm_config["distal"]["mass"] + arm_config["grabber"]["mass"],
                arm_config["distal"]["length"] + arm_config["grabber"]["length"],
                distal_moi,
                distal_cg_radius,
                self._get_motor(arm_config["distal"]["motor"]),
            ),
        )

        # Set up time variables
        self._total_time = opti.variable()
        dt = self._total_time / (n + 1)
        opti.subject_to(self._total_time > 0)
        opti.subject_to(self._total_time < 10)
        opti.minimize(self._total_time)

        # Create theta points
        theta_points = []
        self._theta_points = theta_points
        theta_points.append([opti.parameter(), opti.parameter()])
        for _ in range(n):
            theta_0 = opti.variable()
            theta_1 = opti.variable()
            opti.subject_to(
                opti.bounded(
                    arm_config["proximal"]["minAngle"], theta_0, arm_config["proximal"]["maxAngle"],
                )
            )
            opti.subject_to(
                opti.bounded(
                    arm_config["distal"]["minAngle"], theta_1, arm_config["distal"]["maxAngle"]
                )
            )
            theta_points.append([theta_0, theta_1])
        theta_points.append([opti.parameter(), opti.parameter()])

        # Create constraint parameters (0 = disabled, 1 = enabled)
        self._constraint_parameters = {}
        for (constraint_key, constraint) in constraints.items():
            self._constraint_parameters[constraint_key] = opti.parameter()

        # Apply point constraints
        for i in range(n + 2):
            # Get surrounding points
            last_theta_2 = theta_points[i if i <= 1 else i - 2]
            last_theta = theta_points[i if i == 0 else i - 1]
            current_theta = theta_points[i]
            next_theta = theta_points[i if i == n + 1 else i + 1]

            # Apply voltage constraints
            last_velocity_2 = (
                (last_theta[0] - last_theta_2[0]) / dt,
                (last_theta[1] - last_theta_2[1]) / dt,
            )
            last_velocity = (
                (current_theta[0] - last_theta[0]) / dt,
                (current_theta[1] - last_theta[1]) / dt,
            )
            next_velocity = (
                (next_theta[0] - current_theta[0]) / dt,
                (next_theta[1] - current_theta[1]) / dt,
            )
            last_acceleration = (
                (last_velocity[0] - last_velocity_2[0]) / dt,
                (last_velocity[1] - last_velocity_2[1]) / dt,
            )
            acceleration = (
                (next_velocity[0] - last_velocity[0]) / dt,
                (next_velocity[1] - last_velocity[1]) / dt,
            )
            jerk = (
                (acceleration[0] - last_acceleration[0]) / (2 * dt),
                (acceleration[1] - last_acceleration[1]) / (2 * dt),
            )
            voltage = ff_model.calculate(current_theta, last_velocity, acceleration)
            opti.subject_to(opti.bounded(-max_voltage, voltage[0], max_voltage))
            opti.subject_to(opti.bounded(-max_voltage, voltage[1], max_voltage))
            opti.subject_to(opti.bounded(-max_jerk, jerk[0], max_jerk))
            opti.subject_to(opti.bounded(-max_jerk, jerk[1], max_jerk))

            # Apply position constraints
            if i != 0 and i != n + 1:
                x = (
                    arm_config["shoulder"][0]
                    + arm_config["proximal"]["length"] * cos(current_theta[0])
                    + arm_config["distal"]["length"]
                    * cos(current_theta[0] + current_theta[1])
                )
                y = (
                    arm_config["shoulder"][1]
                    + arm_config["proximal"]["length"] * sin(current_theta[0])
                    + arm_config["distal"]["length"]
                    * sin(current_theta[0] + current_theta[1])
                )

                for (constraint_key, constraint) in constraints.items():
                    enabled = self._constraint_parameters[constraint_key]
                    type = constraint["type"]
                    args = constraint["args"]
                    if type == "minX":
                        opti.subject_to(x >= args[0] - ((1 - enabled) * 1000))

                    elif type == "maxX":
                        opti.subject_to(x <= args[0] + ((1 - enabled) * 1000))

                    elif type == "minY":
                        opti.subject_to(y >= args[0] - ((1 - enabled) * 1000))

                    elif type == "maxY":
                        opti.subject_to(y <= args[0] + ((1 - enabled) * 1000))

                    elif type == "circle":
                        center_x = args[0]
                        center_y = args[1]
                        radius = args[2]
                        opti.subject_to(
                            sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
                            > radius * enabled
                        )

                    elif type == "rectangle":
                        x_0 = args[0]
                        y_0 = args[1]
                        x_1 = args[2]
                        y_1 = args[3]
                        opti.subject_to(
                            fmax(
                                fabs((2 * (x - ((x_0 + x_1) / 2)) / (x_1 - x_0))),
                                fabs((2 * (y - ((y_0 + y_1) / 2)) / (y_1 - y_0))),
                            )
                            > enabled
                        )

    def _get_motor(self, motor_config) -> DCMotor:
        if motor_config["type"] == "neo":
            return DCMotor.getNEO(motor_config["count"]).withReduction(
                motor_config["reduction"]
            )
        elif motor_config["type"] == "neo550":
            return DCMotor.getNEO550(motor_config["count"]).withReduction(
                motor_config["reduction"]
            )
        elif motor_config["type"] == "falcon":
            return DCMotor.getFalcon500(motor_config["count"]).withReduction(
                motor_config["reduction"]
            )

    def solve(self, parameters):
        opti = self._opti
        theta_points = self._theta_points
        kinematics = self._arm_kinematics

        x2_beg = [parameters["start_xy"][0]]
        y2_beg = [parameters["start_xy"][1]]
        x2_end = [parameters["final_xy"][0]]
        y2_end = [parameters["final_xy"][1]]

        [theta1_beg, theta2_beg, valid] = kinematics.inverse_kinematics(x2_beg, y2_beg)
        [theta1_end, theta2_end, valid] = kinematics.inverse_kinematics(x2_end, y2_end)
        theta1_beg = theta1_beg[0]
        theta1_end = theta1_end[0]
        theta2_beg = theta2_beg[0]
        theta2_end = theta2_end[0]

        if (x2_beg == x2_end) and (y2_beg == y2_end):
            result = (0.0, [], [])
            for _ in range(len(self._theta_points)):
                result[1].append(theta1_beg)
                result[2].append(theta2_beg)
            return result


        # Update position parameters
        opti.set_value(
            theta_points[0][0],
            self._clamp(
                theta1_beg,
                self._arm_config["proximal"]["minAngle"],
                self._arm_config["proximal"]["maxAngle"],
            ),
        )
        opti.set_value(
            theta_points[0][1],
            self._clamp(
                theta2_beg,
                self._arm_config["distal"]["minAngle"],
                self._arm_config["distal"]["maxAngle"],
            ),
        )
        opti.set_value(
            theta_points[len(theta_points) - 1][0],
            self._clamp(
                theta1_end,
                self._arm_config["proximal"]["minAngle"],
                self._arm_config["proximal"]["maxAngle"],
            ),
        )
        opti.set_value(
            theta_points[len(theta_points) - 1][1],
            self._clamp(
                theta2_end,
                self._arm_config["distal"]["minAngle"],
                self._arm_config["distal"]["maxAngle"],
            ),
        )

        # Set initial values
        opti.set_initial(self._total_time, 1)
        n = len(theta_points) - 2
        for i in range(1, n + 1):
            opti.set_initial(
                theta_points[i][0],
                (theta1_end - theta1_beg) * (i / (n + 2))
                + theta1_beg,
            )
            opti.set_initial(
                theta_points[i][1],
                (theta2_end - theta2_beg) * (i / (n + 2))
                + theta2_beg,
            )

        # Set constraint parameters
        start_theta = (opti.value(theta_points[0][0]), opti.value(theta_points[0][1]))
        final_theta = (opti.value(theta_points[-1][0]), opti.value(theta_points[-1][1]))
        start_x = (
            self._arm_config["shoulder"][0]
            + self._arm_config["proximal"]["length"] * cos(start_theta[0])
            + self._arm_config["distal"]["length"] * cos(start_theta[0] + start_theta[1])
        )
        start_y = (
            self._arm_config["shoulder"][1]
            + self._arm_config["proximal"]["length"] * sin(start_theta[0])
            + self._arm_config["distal"]["length"] * sin(start_theta[0] + start_theta[1])
        )
        final_x = (
            self._arm_config["shoulder"][0]
            + self._arm_config["proximal"]["length"] * cos(final_theta[0])
            + self._arm_config["distal"]["length"] * cos(final_theta[0] + final_theta[1])
        )
        final_y = (
            self._arm_config["shoulder"][1]
            + self._arm_config["proximal"]["length"] * sin(final_theta[0])
            + self._arm_config["distal"]["length"] * sin(final_theta[0] + final_theta[1])
        )
        for (
            constraint_key,
            constraint_parameter,
        ) in self._constraint_parameters.items():
            constraint = self._constraints[constraint_key]
            type = constraint["type"]
            args = constraint["args"]

            # Check if start position violates the constraint
            any_violated = False
            for x, y, in [
                (start_x, start_y),
                (final_x, final_y),
            ]:
                violated = False
                if type == "minX":
                    violated = x < args[0]

                elif type == "maxX":
                    violated = x > args[0]

                elif type == "minY":
                    violated = y < args[0]

                elif type == "maxY":
                    violated = y > args[0]

                elif type == "circle":
                    center_x = args[0]
                    center_y = args[1]
                    radius = args[2]
                    violated = sqrt((x - center_x) ** 2 + (y - center_y) ** 2) <= radius

                elif type == "rectangle":
                    x_0 = args[0]
                    y_0 = args[1]
                    x_1 = args[2]
                    y_1 = args[3]
                    violated = x >= x_0 and x <= x_1 and y >= y_0 and y <= y_1

                if violated:
                    any_violated = True

            # Set enabled
            if any_violated or (constraint_key in parameters["constraintOverrides"]):
                opti.set_value(constraint_parameter, 0)
            else:
                opti.set_value(constraint_parameter, 1)

        # Solve
        try:
            opti.solve()
        except:
            return None

        # Get results
        result = (opti.value(self._total_time), [], [])
        for theta in theta_points:
            result[1].append(opti.value(theta[0]))
            result[2].append(opti.value(theta[1]))
        return result

    def _clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))
