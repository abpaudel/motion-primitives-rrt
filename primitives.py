import numpy as np
from pose import Pose


class MotionPrimitive():
    def __init__(self, poses, cost):
        self.poses = poses
        self.cost = cost

    def transform(self, pose):
        """Transform the motion primitive by a given pose.
        This is equivalent to 'right multiplying' the pose/transform.
        The resulting primitive should be such that the output poses
        are the input poses applied to the pose.
        """
        new_poses = [p * pose for p in self.poses]
        return MotionPrimitive(poses=new_poses, cost=self.cost)


def dubins_car_primitives(max_theta=np.pi / 6, curvature=1.0, steps=10):
    """Returns the motion primitives available to the robot at the current
    time (can be a function of robot state)."""
    # Create motion primitives
    r = curvature
    return [MotionPrimitive(poses=[Pose(
        x=r * np.sin(d * max_theta / steps),
        y=r * (1 - np.cos(d * max_theta / steps)),
        yaw=d * max_theta / 10) for d in range(1, steps + 1)], cost=r * max_theta),
        MotionPrimitive(poses=[Pose(x=r * d * max_theta / steps, y=0, yaw=0)
                        for d in range(1, steps + 1)], cost=r * max_theta),
        MotionPrimitive(poses=[Pose(
            x=r * np.sin(d * max_theta / steps),
            y=r * (np.cos(d * max_theta / steps) - 1),
            yaw=-d * max_theta / steps) for d in range(1, steps + 1)], cost=r * max_theta),
    ]


def turtlebot_primitives(max_theta=np.pi / 3, primitive_length=1, num_primitives=2, steps=10):
    r0 = primitive_length
    N = num_primitives
    angles = [(i * 1.0 / N - 1) * max_theta / 2
              for i in range(2 * N + 1)]
    primitive_list = []
    for angle in angles:
        poses = []
        if angle != 0:
            poses += [Pose(x=0, y=0, yaw=angle * d / steps)
                      for d in range(1, steps + 1)]
        transform = Pose(x=r0 / steps, y=0, yaw=0)
        steps = steps - 1 if angle == 0 else steps
        for i in range(steps):
            pose = poses[-1] if len(poses) else transform
            poses.append(transform * pose)
        primitive_list += [MotionPrimitive(poses, cost=r0 + abs(angle) / np.pi)]

    primitive_list += [MotionPrimitive(poses, cost=r0)]
    return primitive_list


def perpendicular_turtlebot_primitives(primitive_length=1, steps=10):
    r0 = primitive_length
    angles = np.radians([0, 90, 180, -90])
    primitive_list = []
    for angle in angles:
        poses = []
        if angle != 0:
            poses += [Pose(x=0, y=0, yaw=angle * d / steps)
                      for d in range(1, steps + 1)]
        transform = Pose(x=r0 / steps, y=0, yaw=0)
        steps = steps - 1 if angle == 0 else steps
        for i in range(steps):
            pose = poses[-1] if len(poses) else transform
            poses.append(transform * pose)
        primitive_list += [MotionPrimitive(poses, cost=r0 + abs(angle) / np.pi)]

    primitive_list += [MotionPrimitive(poses, cost=r0)]
    return primitive_list
