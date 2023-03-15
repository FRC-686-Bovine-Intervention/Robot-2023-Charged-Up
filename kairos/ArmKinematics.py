import math
import cmath


class ArmKinematics:
    _shoulder = []
    _arm_length = []

    def __init__(self, shoulder_x, shoulder_y, proximal_length, distal_length):
        self._shoulder = [shoulder_x, shoulder_y]
        self._arm_length = [proximal_length, distal_length]

    def forward_kinematics(self, theta):
        x0 = self._shoulder[0]
        y0 = self._shoulder[1]

        r1 = self._arm_length[0]    # PROXIMAL
        r2 = self._arm_length[1]    # DISTAL

        x1 = [None] * len(theta)
        y1 = [None] * len(theta)
        x2 = [None] * len(theta)
        y2 = [None] * len(theta)

        for k in range(0, len(theta)):
            theta1 = theta[k][0]
            theta2 = theta[k][1]

            x1[k] = x0 + r1 * math.cos(theta1)
            y1[k] = y0 + r1 * math.sin(theta1)

            x2[k] = x1[k] + r2 * math.cos(theta2)
            y2[k] = y1[k] + r2 * math.sin(theta2)

        return [x1, y1], [x2, y2]

    def inverse_kinematics(self, x2, y2, elbow_forward=0):

        x0 = self._shoulder[0]
        y0 = self._shoulder[1]

        r1 = self._arm_length[0]    # PROXIMAL
        r2 = self._arm_length[1]    # DISTAL

        x1 = []
        y1 = []
        theta1 = []
        theta2 = []
        valid = []

        print(len(x2))
        for k in range(0, len(x2)):
            D = math.sqrt((x2[k] - x0)**2 + (y2[k] - y0)**2)
            J = (r1**2 - r2**2) / (2 * D**2)
            K = 1 / 2 * cmath.sqrt(2 * (r1**2 + r2**2) / D**2 - (r1**2 - r2**2)**2. / D**4 - 1)

            if abs(K.imag) > 1e-9:
                # for points we can't reach, return an angle pointing in the right direction
                theta1.append(math.atan2(y2[k]-y0, x2[k]-x0))
                theta2.append(0)
                valid.append(0)

            else:
                K = K.real

                x1_a = (x0 + x2[k]) / 2 + J * (x2[k] - x0) + K * (y2[k] - y0)
                y1_a = (y0 + y2[k]) / 2 + J * (y2[k] - y0) + K * (x0 - x2[k])

                x1_b = (x0 + x2[k]) / 2 + J * (x2[k] - x0) - K * (y2[k] - y0)
                y1_b = (y0 + y2[k]) / 2 + J * (y2[k] - y0) - K * (x0 - x2[k])

                theta1_a = math.atan2(y1_a - y0, x1_a - x0)
                theta1_b = math.atan2(y1_b - y0, x1_b - x0)

                x1.append(x1_a)
                y1.append(y1_a)
                if elbow_forward:
                    if theta1_b >= theta1_a:
                        x1[k] = x1_b
                        y1[k] = y1_b
                else:
                    # pick elbow back intersection point (lowest theta1)
                    if theta1_b < theta1_a:
                        x1[k] = x1_b
                        y1[k] = y1_b

                # calculate joint angles
                theta1.append(math.atan2(y1[k]-y0, x1[k]-x0))
                theta2.append(math.atan2(y2[k]-y1[k], x2[k]-x1[k]))
                valid.append(1)

        return theta1, theta2, valid