# Summit XL Autonomous Navigation

Autonomous waypoint navigation project for an **Autonomous Vehicles** academic course.

The robot uses the classic ROS Navigation Stack:
- **gmapping** for SLAM (map building)
- **AMCL** for localization
- **move_base** for global/local planning and obstacle avoidance
- **waypoint_patrol.py** for multi-goal autonomous patrol

Built to run inside [The Construct ROSDS](https://app.theconstructsim.com/desktop/course/57) with **Summit XL** in Gazebo + RViz.

## Project demo flow

1. Build a map by driving Summit XL with keyboard teleop while gmapping runs
2. Save the map to `maps/summit_world.yaml` + `maps/summit_world.pgm`
3. Launch navigation (map_server + AMCL + move_base)
4. Send goals manually in RViz **or** run automatic waypoint patrol

## Repository structure

```
summit_xl_autonomous_nav/
├── launch/
│   ├── summit_xl_mapping.launch          # Gazebo + gmapping + teleop
│   ├── summit_xl_mapping_no_sim.launch   # gmapping only (sim already running)
│   ├── summit_xl_navigation.launch       # AMCL + move_base
│   └── summit_xl_patrol.launch           # navigation + patrol script
├── config/                               # costmaps, AMCL, move_base params
├── maps/                                 # saved occupancy grid map
├── scripts/
│   ├── waypoint_patrol.py                # actionlib multi-goal client
│   └── check_environment.sh              # verify ROSDS packages
└── docs/                                 # presentation + theory document
```

## Setup in The Construct (ROSDS)

### Step 1 — Open the environment

1. Log in to [The Construct](https://app.theconstructsim.com/)
2. Open course **ROS Navigation in 5 Days** or any ROS Noetic simulation desktop
3. Open a **Terminal**

### Step 2 — Clone and build

```bash
cd ~/catkin_ws/src   # or ~/ros_ws/src if that's your workspace
git clone https://github.com/YOUR_USERNAME/summit_xl_autonomous_nav.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

If you don't have a workspace yet:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/YOUR_USERNAME/summit_xl_autonomous_nav.git
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source devel/setup.bash
```

### Step 3 — Check packages

```bash
rosrun summit_xl_autonomous_nav check_environment.sh
```

If `summit_xl_sim_bringup` is missing, install Robotnik packages:

```bash
cd ~/catkin_ws/src
git clone -b kinetic-devel https://github.com/RobotnikAutomation/summit_xl_common.git
git clone -b kinetic-devel https://github.com/RobotnikAutomation/summit_xl_sim.git
cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -r -y
catkin_make && source devel/setup.bash
```

> On Noetic, try branch `melodic-devel` or `noetic-devel` if available.

## Run instructions

### A) Mapping (create the map)

**Option 1 — start simulation yourself:**

```bash
roslaunch summit_xl_autonomous_nav summit_xl_mapping.launch
```

**Option 2 — simulation already running (course sim):**

```bash
roslaunch summit_xl_autonomous_nav summit_xl_mapping_no_sim.launch
```

In the teleop terminal:
- `i` = forward, `,` = backward
- `j` / `l` = rotate
- `k` = stop

Drive around the entire environment until the map in RViz looks complete.

**Save the map** (new terminal):

```bash
source ~/catkin_ws/devel/setup.bash
rosrun map_server map_saver -f $(rospack find summit_xl_autonomous_nav)/maps/summit_world
```

This creates `maps/summit_world.yaml` and `maps/summit_world.pgm`.

### B) Navigation (manual goals)

```bash
roslaunch summit_xl_autonomous_nav summit_xl_navigation.launch
```

In RViz:
1. Set **Fixed Frame** = `map`
2. Add displays: Map, LaserScan (`/scan`), RobotModel, Path
3. Click **2D Pose Estimate** to set initial robot pose
4. Click **2D Nav Goal** to send a navigation goal

### C) Autonomous patrol (for demo video)

1. Edit waypoint coordinates in `scripts/waypoint_patrol.py` based on your saved map
2. Run:

```bash
roslaunch summit_xl_autonomous_nav summit_xl_patrol.launch
```

The robot will visit each waypoint automatically.

## Troubleshooting

| Problem | Fix |
|---------|-----|
| No `/scan` topic | Check `rostopic list`, set `scan_topic:=merged_scan` in launch |
| Map is empty in RViz | Set Fixed Frame to `map`, drive robot to update SLAM |
| AMCL particles spread | Use **2D Pose Estimate** in RViz near robot's real position |
| Robot doesn't move | Check `rostopic echo /cmd_vel` and `move_base/status` |
| `summit_xl_sim_bringup` not found | Clone Robotnik repos (see Step 3) |

## Course deliverables checklist

- [ ] PowerPoint presentation (10-12 slides) with GitHub + video links on slide 11
- [ ] Theory document (5-7 pages)
- [ ] Demo recording (5-12 min) showing mapping + navigation + patrol
- [ ] This GitHub repository
- [ ] Project Summary form

## References

- [ROS Navigation in 5 Days — The Construct](https://app.theconstructsim.com/desktop/course/57)
- [Robotnik summit_xl_sim](https://github.com/RobotnikAutomation/summit_xl_sim)
- [Robotnik summit_xl_common](https://github.com/RobotnikAutomation/summit_xl_common)
- [ROS Navigation Stack Wiki](http://wiki.ros.org/navigation)

## License

MIT — for academic course project use.
