import numpy as np

class ContinuumKinematics:

    def tendon_displacement(self, servo):
        return servo.horn_radius * servo.angle

    def compute_link_curvature(self, link_id, robot_state):
        kappa = np.array([0.0, 0.0])

        for servo in robot_state.servos:
            if link_id in servo.affects_links:
                dL = self.tendon_displacement(servo)

                # Assume tendon directions (simplified)
                direction = np.array([
                    np.cos(servo.id),
                    np.sin(servo.id)
                ])

                kappa += direction * dL

        return kappa

    def compute_transforms(self, robot_state):
        transforms = []
        current_pos = np.array([0, 0, 0])
        current_rot = np.eye(3)

        for link in robot_state.links:
            kappa = self.compute_link_curvature(link.id, robot_state)
            length = link.length

            theta = np.linalg.norm(kappa) * length

            if theta > 1e-6:
                axis = np.array([kappa[0], kappa[1], 0.0])
                axis = axis / np.linalg.norm(axis)

                R = self.rotation_matrix(axis, theta)
            else:
                R = np.eye(3)

            current_rot = current_rot @ R
            current_pos = current_pos + current_rot @ np.array([0, 0, length])

            transforms.append((current_pos.copy(), current_rot.copy()))

        return transforms

    def rotation_matrix(self, axis, theta):
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        return np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K @ K)
