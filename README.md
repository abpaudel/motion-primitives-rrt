# Motion Primitives based Path Planning with RRT

## Usage
```
usage: main.py [-h] [--env {maze_small,maze_large}]
               [--primitive {dubins_pi_0p5,dubins_pi2_0p5,dubins_pi4_1p0,dubins_pi6_2p0,dubins_many,turtlebot_pi3_0p75_3,turtlebot_pi1p5_1p0_4,perp_turtlebot_1p0}]
               [--robot_radius ROBOT_RADIUS] [--num_iters NUM_ITERS]
               [--seed SEED] [--save_frames] [--outpath OUTPATH]

optional arguments:
  -h, --help            show this help message and exit
  --env {maze_small,maze_large}
                        Chosen environment name (default: maze_small)
  --primitive {dubins_pi_0p5,dubins_pi2_0p5,dubins_pi4_1p0,dubins_pi6_2p0,dubins_many,turtlebot_pi3_0p75_3,turtlebot_pi1p5_1p0_4,perp_turtlebot_1p0}
                        Chosen motion primitive name (default: dubins_pi4_1p0)
  --robot_radius ROBOT_RADIUS
                        Radius of robot (default: 0.5)
  --num_iters NUM_ITERS
                        Number of iterations for RRT (default: 6000)
  --seed SEED           Random seed (default: 42)
  --save_frames         Save pose frames to generate animation (default: False)
  --outpath OUTPATH     Path to save results (default: ./results)
```


## Results
Animations can be generated using the following `ffmpeg` command.

```
ffmpeg -f image2 -i plan.png -i pose_%d.png -r 50 -filter_complex overlay out.gif
```
