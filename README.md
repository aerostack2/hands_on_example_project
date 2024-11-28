# Aerostack2: A Software Framework for Developing Multi-robot Aerial Systems
If you use this code in your academic work, please cite ([PDF](https://doi.org/10.48550/arXiv.2303.18237)):

```latex
@article{fernandez2023aerostack2,
  title={Aerostack2: A software framework for developing multi-robot aerial systems},
  author={Fernandez-Cortizas, Miguel and Molina, Martin and Arias-Perez, Pedro and Perez-Segui, Rafael and Perez-Saura, David and Campoy, Pascual},
  journal={arXiv preprint arXiv:2303.18237},
  year={2023}
}
```

This work is released under BSD 3-Clause License.

## Installation
This project has been developed in **Ubuntu 22.04 LTS**, **ROS 2 Humble** and **Aerostack2**.

- Install ROS 2 Humble [[guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)].
- Install Aerostack2 [[guide](https://aerostack2.github.io/_00_getting_started/source_install.html)] forked repository:
```bash
sudo apt install git python3-rosdep python3-pip python3-colcon-common-extensions -y  # previous deps
mkdir -p ~/aerostack2_ws/src/ && cd ~/aerostack2_ws/src/
git clone https://github.com/aerostack2/aerostack2.git
cd ~/aerostack2_ws
sudo rosdep init
rosdep update
rosdep install -y -r -q --from-paths src --ignore-src
```
- Build Aerostack2:
```bash
cd ~/aerostack2_ws
colcon build --symlink-install
```
- Clone launching repository:
```bash
git clone https://github.com/aerostack2/hands_on_example_project.git
```

## How to launch (Simulation)

### Launch Aerostack2
```bash
# Try ./launch_as2.sh -h for help
./launch_as2.bash -s
```

### Launch mission
```bash
python mission.py -s
```

## How to stop
```bash
./stop.bash
```

---
## How to launch (Real)

### Launch Aerostack2
```bash
# Try ./launch_as2.sh -h for help
./launch_as2.bash -e mocap_pose
```

### Launch mission
```bash
python mission.py
```

## How to stop
```bash
./stop.bash
```

## How to choose number of drones
At file `config/swarm_config_file.yaml` change the list of drones.
```yml
cf0:
  uri: radio://0/80/2M/E7E7E7E7E7
cf1:
  uri: radio://0/80/2M/E7E7E7E700
# Commented drones will not be launched
# cf2:
#   uri: radio://0/80/2M/E7E7E7E702
```