import numpy as np
import yaml
import os
import pydmps
import tf
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from ros_dmp.msg import CartesianTrajectory, CartesianState


class RollDmp():

    def __init__(self, weight_path, dt, target_frame):
        if not os.path.exists(weight_path):
            raise ValueError("weight file '{}' does not exist: ".format(weight_path))

        self.target_frame = target_frame

        # create PyDMPs instance
        weights = self.load_weights(weight_path)
        n_dmps = weights.shape[0]
        n_bfs = weights.shape[1]
        self.dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=n_dmps, n_bfs=n_bfs,
                                                     dt=dt, ay=None, w=weights)

    def roll(self, goal, initial_pose, tau):
        self.pos, self.vel, self.acc = self.dmp.rollout(goal=goal, y0=initial_pose, tau=tau)
        return self.pos, self.vel, self.acc

    def get_trajectory_and_path(self, goal_pose, initial_pose, tau):
        pos, vel, acc = self.roll(goal_pose, initial_pose, tau)

        cartesian_trajectory = CartesianTrajectory()
        cartesian_trajectory.header.frame_id = self.target_frame
        path = Path()
        path.header.frame_id = self.target_frame
        for i in range(pos.shape[0]):
            x, y, z, w = tf.transformations.quaternion_from_euler(pos[i, 3], pos[i, 4], pos[i, 5])
            pose = Pose()
            pose.position.x = pos[i, 0]
            pose.position.y = pos[i, 1]
            pose.position.z = pos[i, 2]
            pose.orientation.x = x
            pose.orientation.y = y
            pose.orientation.z = z
            pose.orientation.w = w

            cartesian_state = CartesianState()
            cartesian_state.pose = pose
            pose_stamped = PoseStamped()
            pose_stamped.pose = pose

            cartesian_state.vel.linear.x = vel[i, 0]
            cartesian_state.vel.linear.y = vel[i, 1]
            cartesian_state.vel.linear.z = vel[i, 2]
            cartesian_state.vel.angular.x = vel[i, 3]
            cartesian_state.vel.angular.y = vel[i, 4]
            cartesian_state.vel.angular.z = vel[i, 5]

            cartesian_state.acc.linear.x = acc[i, 0]
            cartesian_state.acc.linear.y = acc[i, 1]
            cartesian_state.acc.linear.z = acc[i, 2]
            cartesian_state.acc.angular.x = acc[i, 3]
            cartesian_state.acc.angular.y = acc[i, 4]
            cartesian_state.acc.angular.z = acc[i, 5]

            cartesian_trajectory.cartesian_state.append(cartesian_state)
            path.poses.append(pose_stamped)

        return cartesian_trajectory, path

    def load_weights(self, file_name):

        with open(file_name) as f:
            loadeddict = yaml.load(f)
        x = loadeddict.get('x')
        y = loadeddict.get('y')
        z = loadeddict.get('z')
        roll = loadeddict.get('roll')
        pitch = loadeddict.get('pitch')
        yaw = loadeddict.get('yaw')

        weights = np.array(x)
        weights = np.vstack((weights, np.array(y)))
        weights = np.vstack((weights, np.array(z)))
        weights = np.vstack((weights, np.array(roll)))
        weights = np.vstack((weights, np.array(pitch)))
        weights = np.vstack((weights, np.array(yaw)))

        return weights
