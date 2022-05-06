import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from pose import Pose
from planning import move
# from planning import Link


def plot_env(start, goal, obstacles, fill=True):
    for poly in obstacles:
        xs, ys = poly.exterior.xy
        plt.plot(xs, ys, 'k')
        if fill:
            plt.gca().fill(xs, ys, fc='k')
    plt.plot(start[0], start[1], 'b.')
    plt.plot(goal[0], goal[1], 'g*', markersize=15)
    plt.gca().axis('equal')


def plot_environment_and_links(start, goal, obstacles, all_links, do_plot_goal_path=True):
    for link in all_links:
        link.plot(fmt='y', alpha=0.5)
    closest_link_to_goal = min(all_links, key=lambda l: l.get_distance(goal))
    # FIXME
    # goal_path = Link([Pose(*goal)], upstream=closest_link_to_goal)
    goal_path = closest_link_to_goal
    plt.title(f"Cost of path: {goal_path.cost:.4f}")
    if do_plot_goal_path:
        while goal_path is not None:
            goal_path.plot(fmt='b')
            goal_path = goal_path.upstream

    plot_env(start, goal, obstacles)


def plot_robot_pose(pose, r):
    circle = matplotlib.patches.Circle((pose.x, pose.y), r, facecolor='silver', edgecolor='black', zorder=2)
    plt.gca().add_patch(circle)
    plt.arrow(pose.x, pose.y, r * np.cos(pose.yaw), r * np.sin(pose.yaw),
              head_length=r, head_width=r / 2, length_includes_head=True,
              color='black', zorder=2)


def plot_primitives(primitives, robot_radius=0.5):
    pose = Pose(0, 0)
    plt.plot(pose.x, pose.y, 'b.')
    plot_robot_pose(pose, r=robot_radius)
    for primitive in primitives:
        poses = move(pose, primitive)[0]
        xs = [pose.x]
        ys = [pose.y]
        xs += [pose.x for pose in poses]
        ys += [pose.y for pose in poses]
        plt.plot(xs, ys, alpha=0.7)
    plt.axis('equal')
    plt.xlim([-2, 5])
    plt.ylim([-5, 5])
