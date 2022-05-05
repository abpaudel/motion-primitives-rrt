import numpy as np
import matplotlib.pyplot as plt
import random
from shapely.geometry import LineString
from pose import Pose


class Link(object):
    def __init__(self, poses, cost=0, upstream=None):
        poses = poses * 2 if len(poses) == 1 else poses
        self.poses = poses
        self.local_cost = cost
        self.point = [self.poses[-1].x, self.poses[-1].y]
        self.upstream = upstream

        if self.upstream is None:
            self.geom_line = None
        else:
            points = [self.upstream.point]
            for pose in self.poses:
                points.append([pose.x, pose.y])
            self.geom_line = LineString(points)

    @property
    def cost(self):
        if self.upstream is not None:
            return self.upstream.cost + self.local_cost
        else:
            return 0

    def get_distance(self, point):
        return np.linalg.norm(np.array(self.point) - np.array(point))

    def does_collide(self, obstacles):
        if self.upstream is None:
            return False
        return any(self.geom_line.intersects(obstacle)
                   for obstacle in obstacles)

    def __str__(self):
        return str(self.geom_line)

    def __hash__(self):
        return hash(id(self))

    def plot(self, fmt='b', alpha=1):
        if self.upstream is not None:
            x, y = self.geom_line.coords.xy
            plt.plot(x, y, fmt, alpha=alpha)


def steer_towards_point(link, new_point, primitives):
    poses_and_cost = [move(link.poses[-1], primitive)
                      for primitive in primitives]
    final_poses = [poses[-1] for poses, _ in poses_and_cost]
    distances = [pose.get_distance(new_point) for pose in final_poses]
    new_poses, cost = poses_and_cost[np.argmin(distances)]
    return Link(new_poses, cost=cost, upstream=link), [new_poses[-1].x, new_poses[-1].y]


def move(pose, primitive):
    transformed_primitive = primitive.transform(pose)
    return transformed_primitive.poses, transformed_primitive.cost


def RRT(start, goal, obstacles, region_x, region_y,
        num_iterations, primitives, seed=695):
    random.seed(seed)
    links = [Link([Pose(*start)])]
    for _ in range(num_iterations):
        # Generate a random point
        px = random.uniform(region_x[0], region_x[1])
        py = random.uniform(region_y[0], region_y[1])
        point = [px, py]

        # Get the closest link
        closest_dist = float('inf')
        closest_link = None
        for link in links:
            dist = link.get_distance(point)
            if dist < closest_dist:
                closest_dist = dist
                closest_link = link
        latest_link = closest_link

        # Steer towards it
        new_link, new_point = steer_towards_point(
            latest_link, point, primitives)

        # If it collides, return
        if new_link.does_collide(obstacles):
            continue

        # Add to chain if it does not collide
        links.append(new_link)

    return links
