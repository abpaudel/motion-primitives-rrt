import numpy as np
from pose import Pose
from planning import Link
from shapely.geometry import LineString, Polygon


def get_poses(links, goal):
    closest_link_to_goal = min(links, key=lambda l: l.get_distance(goal))
    # FIXME
    # link = Link([Pose(*goal)], upstream=closest_link_to_goal)
    link = closest_link_to_goal
    poses = []
    while link is not None:
        poses.extend(link.poses[::-1])
        link = link.upstream
    return poses[::-1]


def get_goal_path_length(goal, all_links, step_size=2):
    closest_link_to_goal = min(all_links, key=lambda l: l.get_distance(goal))
    if closest_link_to_goal.get_distance(goal) > step_size:
        return None
    goal_path = Link([Pose(*goal)], upstream=closest_link_to_goal)
    return goal_path.cost


def inflate_env(env, r=0.5):
    inflated_env = []
    for p in env:
        inflated_env += [Polygon(p.buffer(r).exterior)]
    return inflated_env


def load_env_from_file(file):
    coords = np.genfromtxt(file)
    maze_env = [LineString([x, y]).buffer(0.01)
                for x, y in zip(coords, np.roll(coords, 2))]
    return maze_env
