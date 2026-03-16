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

    def solve_ik(self, q_seed, target_pos, target_orn=None):
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

        q_sol = p.calculateInverseKinematics(**kwargs)
        return list(q_sol[:self.n])

    def close(self):
        try:
            p.disconnect(self.client)
        except Exception:
            pass
