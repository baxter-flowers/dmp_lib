from dmp_lib import DMPs_discrete
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from transformations import pose_to_list, list_to_pose  # TODO dependency baxter_commander
from tf.transformations import quaternion_about_axis
import numpy as np
import rospy

#     dmp = DMPs_discrete(dmps=2, bfs=bfs)
#     print len(np.array([path1, path2])
#     dmp.imitate_path(y_des=np.array([path1, path2]))
#     # change the scale of the movement
#     dmp.goal[0] = 3; dmp.goal[1] = 2

#     y_track,dy_track,ddy_track = dmp.rollout()

class DiscreteTaskSpaceTrajectory(object):
    NUMBER_STABLE_STATES = 50

    def __init__(self, init_path, bfs=10):
        assert isinstance(path, Path)
        self.init_path = init_path
        self._dmp = DMPs_discrete(dmps=7, bfs=bfs)
        self._dmp.imitate_path(self._path_to_y_des(init_path, self.NUMBER_STABLE_STATES))

        if self.init_path.header.frame_id == '':
            self.init_path.header.frame_id = self.init_path.poses[0].header.frame_id

    def _path_to_y_des(self, path, nss):
        y_des = []
        for pose_s in path.poses:
            assert pose_s.header.frame_id == self.init_path.header.frame_id
            pose = [val for sublist in pose_to_list(pose_s) for val in sublist]  # flatten to [x, y, z, x, y, z, w]
            y_des.append(pose)

        # Repeat the last point (stable state) n times to avoid brutal cuts due to asymptotic approach
        for n in range(nss):
            y_des.append(y_des[-1])

        return np.array(y_des).transpose()

    def rollout(self, goal):
        assert isinstance(goal, PoseStamped)
        self._dmp.goal = [val for sublist in pose_to_list(goal) for val in sublist]
        y_track, dy_track, ddy_track = self._dmp.rollout()

        path = Path()
        for y in y_track:
            path.poses.append(list_to_pose([[y[0], y[1], y[2]], [y[3], y[4], y[5], y[6]]],
                                           frame_id=self.init_path.header.frame_id))
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.init_path.header.frame_id
        return path

if __name__=='__main__':
    rospy.init_node("test_dmp_traj_discrete")
    path = Path()
    for i in range(100):
        quat = quaternion_about_axis(i*0.00628, (1, 0, 0))
        pose = list_to_pose([[i/100., i/100., i/100.], quat])
        pose.header.stamp = i/10.
        path.poses.append(pose)

    goal = list_to_pose([[-10, -10, -10], [0.707, 0, 0, 0.707]])
    rollout = DiscreteTaskSpaceTrajectory(path).rollout(goal)
    print rollout