import numpy as np


class Pose(object):
    def __init__(self, x, y, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

    def __repr__(self):
        return "<Pose x:%4f, y:%4f, yaw:%4f>" % (self.x, self.y, self.yaw)

    def get_distance(self, point):
        return np.linalg.norm(np.array([self.x, self.y]) - np.array(point))

    def __mul__(self, oth):
        return oth.__rmul__(self)

    def __rmul__(self, oth):
        """Define transform out = oth*self. This should be the equivalent
        of adding an additional pose 'oth' to the current pose 'self'.
        This means that, for example, if we have a robot in pose 'self' and
        a motion primitive that ends at 'oth' the position of the end of the
        motion primitive in the world frame is oth*self.
        """

        try:
            x = self.x + np.cos(self.yaw) * oth.x - np.sin(
                self.yaw) * oth.y
            y = self.y + np.cos(self.yaw) * oth.y + np.sin(
                self.yaw) * oth.x
            yaw = (self.yaw + oth.yaw) % (2 * np.pi)
            return Pose(x, y, yaw)
        except AttributeError:
            return Pose(oth * self.x, oth * self.y, self.yaw)
        else:
            raise TypeError(('Type {0} cannot rmul a Pose object.').format(
                type(oth).__name__))
