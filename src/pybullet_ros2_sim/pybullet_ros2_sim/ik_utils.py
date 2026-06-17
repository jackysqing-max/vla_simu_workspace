"""Shared forward/inverse kinematics helper backed by a DIRECT PyBullet model."""

import pybullet as p
import pybullet_data


class IiwaIkHelper:
    """Keep a lightweight side model for IK/FK queries outside the main sim."""

    def __init__(self, urdf_path="kuka_iiwa/model.urdf", use_fixed_base=True):
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF(
            urdf_path,
            useFixedBase=use_fixed_base,
            physicsClientId=self.client,
        )
        self.n = 7
        self.ee_link = self.n - 1
        self.lower_limits = [
            -2.96705972839,
            -2.09439510239,
            -2.96705972839,
            -2.09439510239,
            -2.96705972839,
            -2.09439510239,
            -3.05432619099,
        ]
        self.upper_limits = [
            2.96705972839,
            2.09439510239,
            2.96705972839,
            2.09439510239,
            2.96705972839,
            2.09439510239,
            3.05432619099,
        ]
        self.joint_ranges = [
            upper - lower for lower, upper in zip(self.lower_limits, self.upper_limits)
        ]
        self.default_joint_damping = [0.12] * self.n

    def reset_q(self, q):
        for joint_index in range(self.n):
            p.resetJointState(
                self.robot_id,
                joint_index,
                float(q[joint_index]),
                physicsClientId=self.client,
            )

    def fk(self, q):
        """Return end-effector position and orientation for a joint vector."""
        self.reset_q(q)
        state = p.getLinkState(
            self.robot_id,
            self.ee_link,
            computeForwardKinematics=True,
            physicsClientId=self.client,
        )
        return state[4], state[5]

    def solve_ik(
        self,
        q_seed,
        target_pos,
        target_orn=None,
        *,
        rest_poses=None,
        joint_damping=None,
    ):
        """Solve IK from a seed configuration so solutions stay continuous."""
        self.reset_q(q_seed)

        kwargs = {
            "bodyUniqueId": self.robot_id,
            "endEffectorLinkIndex": self.ee_link,
            "targetPosition": target_pos,
            "physicsClientId": self.client,
        }
        if target_orn is not None:
            kwargs["targetOrientation"] = target_orn
        if rest_poses is not None:
            rest = [float(value) for value in rest_poses[: self.n]]
            if len(rest) == self.n:
                kwargs["lowerLimits"] = self.lower_limits
                kwargs["upperLimits"] = self.upper_limits
                kwargs["jointRanges"] = self.joint_ranges
                kwargs["restPoses"] = rest
                damping = (
                    self.default_joint_damping
                    if joint_damping is None
                    else [float(value) for value in joint_damping[: self.n]]
                )
                if len(damping) == self.n:
                    kwargs["jointDamping"] = damping

        q_sol = p.calculateInverseKinematics(**kwargs)
        return list(q_sol[:self.n])

    def close(self):
        try:
            p.disconnect(self.client)
        except Exception:
            pass
