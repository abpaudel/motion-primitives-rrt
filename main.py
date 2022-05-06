import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
from tqdm import tqdm
from planning import RRT
import utils
import plotting
from primitives import dubins_car_primitives, turtlebot_primitives, perpendicular_turtlebot_primitives


envs = {
    'maze_small': [utils.load_env_from_file('./envs/maze_small.txt', buffer=0.1),
                   [-8, -8], [8, -8], [-10, 10], [-10, 10]],
    'maze_large': [utils.load_env_from_file('./envs/maze_large.txt', buffer=0.01),
                   [23, 27.5, np.pi / 2], [23, 3.5], [0, 30], [0, 30]]
}

dubins_many = dubins_car_primitives(max_theta=np.pi / 3, curvature=1)
dubins_many += dubins_car_primitives(max_theta=np.pi / 6, curvature=1.8)
dubins_many.pop(1)

primitives = {
    'dubins_pi_0p5': dubins_car_primitives(max_theta=np.pi, curvature=0.5),
    'dubins_pi2_0p5': dubins_car_primitives(max_theta=np.pi / 2, curvature=0.5),
    'dubins_pi4_1p0': dubins_car_primitives(max_theta=np.pi / 4, curvature=1),
    'dubins_pi6_2p0': dubins_car_primitives(max_theta=np.pi / 6, curvature=2),
    'dubins_many': dubins_many,
    'turtlebot_pi3_0p75_3': turtlebot_primitives(max_theta=np.pi / 3, primitive_length=0.75, num_primitives=3),
    'turtlebot_pi1p5_1p0_4': turtlebot_primitives(max_theta=np.pi / 1.5, primitive_length=1, num_primitives=4),
    'perp_turtlebot_1p0': perpendicular_turtlebot_primitives(primitive_length=1)
}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--env', choices=list(envs.keys()), default='maze_small', help='Chosen environment name')
    parser.add_argument('--primitive', choices=list(primitives.keys()),
                        default='dubins_pi4_1p0', help='Chosen motion primitive name')
    parser.add_argument('--robot_radius', type=float, default=0.5, help='Radius of robot')
    parser.add_argument('--num_iters', type=int, default=6000, help='Number of iterations for RRT')
    parser.add_argument('--seed', type=int, default=42, help='Random seed')
    parser.add_argument('--save_frames', action='store_true', help='Save pose frames to generate animation')
    parser.add_argument('--outpath', type=str, default='./results', help='Path to save results')
    args = parser.parse_args()

    outpath = Path(f'./results/{args.env}_{args.primitive}')
    outpath.mkdir(parents=True, exist_ok=True)

    env, start, goal, region_x, region_y = envs[args.env]
    primitive_library = primitives[args.primitive]

    plt.figure(figsize=(5, 5), dpi=150)
    plotting.plot_primitives(primitive_library)
    plt.savefig(outpath / 'motionprimitive.png', transparent=True)
    plt.close()

    print(f'Environment: {args.env}, Motion Primitive: {args.primitive}')

    links = RRT(start, goal, utils.inflate_env(env, r=args.robot_radius), region_x, region_y,
                args.num_iters, primitives=primitive_library, seed=args.seed)

    plt.figure(figsize=(8, 6), dpi=150)
    plotting.plot_environment_and_links(start, goal, env, links, do_plot_goal_path=True)
    plt.axis('equal')
    xlim = plt.xlim()
    ylim = plt.ylim()
    plt.plot(xlim, ylim, alpha=0)
    plt.savefig(outpath / 'plan.png')

    path_length = utils.get_goal_path_length(goal, links)
    if path_length is None:
        print('Path not found.')
        exit()
    print(f"Path found. Path Length: {path_length:.4f}")

    if args.save_frames:
        print('Saving pose frames for animation...')
        poses = utils.get_poses(links, goal)
        for i, pose in enumerate(tqdm(poses)):
            plt.cla()
            plotting.plot_robot_pose(pose, r=args.robot_radius)
            plt.plot(xlim, ylim, alpha=0)
            plt.axis('equal')
            plt.axis('off')
            plt.savefig(outpath / f'pose_{i}.png', transparent=True)
