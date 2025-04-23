# Helper class for calculating the forward and inverse kinematics of a 5dof robot arm geometrically

import math as m


class ARM_5DOF:


    def __init__(self, joint_params):

        if len(joint_params) < 5:
            raise Exception("Please provide all (5) limb lengths")
        
        self.joint_params = joint_params

    
    def solveIK(self, target_pose):
        # unpack inputs for clarity
        x, y, z, theta4, theta5 = target_pose
        l1, l2, l3, l4, l5 = self.joint_params

        # base yaw
        theta1 = m.atan2(y, x)

        # effective wrist offset
        wrist_len = l4 + l5
        horz = wrist_len * m.cos(theta4)
        x4 = x - horz * m.cos(theta1)
        y4 = y - horz * m.sin(theta1)
        z4 = z - wrist_len * m.sin(theta4) - l1   # subtract vertical base offset z

        # distance from shoulder to wrist
        r2 = x4*x4 + y4*y4 + z4*z4

        # law of cosines for theta2
        cos_phi = (r2 - l2**2 - l3**2) / (2 * l2 * l3)
        cos_phi = max(-1.0, min(1.0, cos_phi))  # clamp into [-1,1]
        theta2 = -m.acos(cos_phi)

        # shoulder elevation for theta3
        beta  = m.atan2(z4, m.sqrt(x4*x4 + y4*y4))
        gamma = m.atan2(
            l3 * m.sin(-theta2),
            l2 + l3 * m.cos(-theta2)
        )
        theta3 = beta + gamma

        return [theta1, theta2, theta3, theta4, theta5]


    def solveFK(self, joint_angs):
        pass



joint_params = [10, 10, 10, 10, 10]
target_pose = [10, 10, 10, -m.pi/2, 0]

robot = ARM_5DOF(joint_params)
print(robot.solveIK(target_pose))
