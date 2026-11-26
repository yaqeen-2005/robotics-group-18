# Group 18 – How to use Webots

Quick guide for everyone to get the robot running and test their own code.

## Getting started

1. Download and install [Webots R2025a](https://cyberbotics.com/) and Python 3 on Windows.
2. Get the latest version of our GitHub repo.
3. Open Webots and go to File > Open World.
4. Go into our repo, then `webots/worlds`, and open `warehouse.wbt`.
5. Press Play, wait about 10 simulation seconds and click inside the simulation window.

You don't need ROS 2, Gazebo or WSL for the current manual simulation. Keep the whole `webots` folder together.

## Which world should I use?

- `warehouse.wbt` – small warehouse for testing individual bits of code.
- `integration_test.wbt` – bigger warehouse with moving obstacles, spills and gates.
- `final_warehouse.wbt` – full warehouse for final testing.

## Driving the robot

| Key | What it does |
| --- | --- |
| W | Forward |
| S | Backwards |
| A | Turn left |
| D | Turn right |
| Space | Stop |
| U | Lift up |
| J | Lift down |
| R | Show lift position in the console |

Hold the movement or lift keys. Stop and wait about half a second before using the lift.

## Picking up a pallet

1. Lower the lift with J.
2. Drive underneath the pallet and stop.
3. Wait half a second, then hold U until the pallet is lifted off its stand.
4. Reverse out carefully and drive to the delivery stand.
5. Stop, wait and lower the pallet with J.
6. Reverse out once the pallet is resting on the stand.

In the final warehouse, follow the order board and matching coloured pickup and delivery locations. A successful delivery shows DELIVERED after you reverse away.

## Extra controls in the integration world

| Key | What it does |
| --- | --- |
| 1 / 2 / 3 | Report or unreport spills |
| 4 / 5 / 6 | Add or remove spills |
| T | Pause or resume moving people and forklift |
| N | Turn random spill changes on or off |
| G / H | Open or close narrow / wide gate |
| B | Close wide gate for 20 simulation seconds |
| F | Change pallet load type |
| O | Request a pallet |

Tap these keys rather than holding them.

## Extra controls in the final warehouse

- 1 / 2 / 3 – focus on pickup lanes and destinations.
- V – change camera view.
- I, then H – select an active spill and report or unreport it (before cleanup starts).

## Where is the code?

- `webots/controllers/pallet_robot/` – robot control code.
- `webots/protos/PalletRobot.proto` – robot model, wheels, sensors and lift.
- `webots/worlds/` – warehouse maps.

Work on your own Git branch so we don't overwrite each other's changes.

## If something isn't working

- Robot not moving? Check that Play is pressed and click inside the simulation.
- Lift not moving? Stop driving first and wait half a second.
- Python error? Check Python is installed and selected in Webots preferences.
- Missing models? Make sure you downloaded the whole repo.
- Dropped a pallet or messed up the warehouse? Reload the world without saving changes.

For now, these instructions are for manual testing. The automatic navigation, SLAM and sensor processing are still being developed.
