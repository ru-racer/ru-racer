# ru-racer

The goal to build **RURacer-2** was to improve platform performance, such as more accurate wheel encoders and more advanced embedded processors. Compared to RURacer-1, the new platform achieves **four times lower latency (less than 7 ms)** and **twenty times faster computation**. To accomplish this, a custom hardware architecture was designed, including wheel encoder fittings for individual wheel speed monitoring, flexible interfaces, and an upgraded IMU system capable of estimating vehicle orientation in three dimensions.

This platform enables researchers to utilize modern hardware/software stacks, including **ROS** and **NVIDIA Jetson TX2**, for advanced real-time robotic control.

![RURacer-2](https://user-images.githubusercontent.com/26307692/115647143-4eee7300-a2f1-11eb-8875-1eae0d6352c9.png)

---

## Hardware and Software Architecture

The vehicle platform is based on a 1/7th scale **Traxxas XO-1**, which supports aggressive driving modes like drifting due to its optional rear-wheel drive configuration and high-speed capability (up to 100 MPH). 

To monitor wheel speeds, **EM1 optical encoders** from US Digital were custom-fit to each wheel. The control stack is powered by **NVIDIA Jetson TX2**, chosen for its real-time computational capabilities. TX2 runs **Ubuntu L4T** with onboard WiFi configured for communication between the host machine and the vehicle.

### Key Features

- **ROS-based communication framework**
- **High-speed UART connection (100 Hz)** between TX2 and a microcontroller using `rosserial`
- **Real-time NMPC motion controller** on TX2
- **Adafruit BNO055 IMU** providing 9 DOF sensing
- **FLIR Blackfly GigE camera** (3.2 MP, 30 FPS) used for localization with 2.6 mm accuracy

This architecture supports tight integration between high-frequency sensor acquisition, low-level control, and edge computation for real-time planning and perception tasks.

---

## How to Use

This repository includes MATLAB comparison scripts for analyzing different driving strategies in stunt-level vehicle maneuvers.

### Repo layout

- **`matlab/`**: original MATLAB implementations (kept intact, only moved into a folder)
- **`cpp/`**: C++ planning + tracking baseline (Spars-RRT + NMPC-style tracker) with CSV logging

### C++ (Spars-RRT + NMPC) quick start

Build:

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build -j
```

Run an example on the provided track map:

```bash
./cpp/build/ru_racer --config cpp/data/track_spars_rrt.cfg
```

Outputs are written under `cpp/results/run_*/` as CSV, and can be visualized on the track image.

#### Example outputs

- **Track image used for mapping**

![track image](docs/images/track.png)

- **Generated occupancy map (white = free)**

![track occupancy debug](docs/images/track_map_from_png_debug.png)

- **Explored tree + best path + NMPC execution (start/goal shown)**

![tree on track with start/goal](docs/images/rrt_tree_on_track_start_goal.png)

- **Trajectory colored by vehicle speed \(v_x\) (same colormap for RRT ref and NMPC exec)**

![speed colored trajectory](docs/images/rrt_speed_on_track.png)

### Clone this repository:
   bash
   git clone https://github.com/ru-racer/ru-racer.git
   cd ru-racer

### Open any of the **speed_compare_*.m** files in MATLAB:
For example:
- **`speed_compare_5ms14rad.m`**:  
This file simulates a U-turn maneuver at **5 m/s** speed and **1.4 m** turning radius.

### Running the File
Running the script will:

- Execute **three methods**:
  - A **Stanford-developed method** with handling limitations
  - A **baseline method** with no handling constraints
  - **Our proposed Safe Stunt Maneuvering method**, designed to safely perform aggressive maneuvers
- Save the results in **matrix format**
- Generate **comparison plots automatically**

You can modify the **speed** and **radius** by editing the filename and the script parameters accordingly.

---

## Publications

Below is a curated list of publications related to this project:

- **RDT-RRT: Real-time double-tree rapidly-exploring random tree path planning for autonomous vehicles**  
  J. Yu, C. Chen, A. Arab, J. Yi, X. Pei, X. Guo  
  *Expert Systems with Applications, 240, 122510 (2024)*  
  [Link](https://www.sciencedirect.com/science/article/pii/S0957417423030129)

- **Safe agile hazard avoidance system for autonomous vehicles**  
  A. Arab, J. Yi  
  *US Patent App. 18/209,943 (2024)*   
  [Link](https://patents.google.com/patent/US20240001964A1/en)

- **Hierarchical framework integrating rapidly-exploring random tree with deep reinforcement learning for autonomous vehicle**  
  J. Yu, A. Arab, J. Yi, X. Pei, X. Guo  
  *Applied Intelligence, 53(13), 16473–16486 (2023)*  
  [Link](https://link.springer.com/article/10.1007/s10489-022-04358-7)

- **Motion planning and control of autonomous aggressive vehicle maneuvers**  
  A. Arab, K. Yu, J. Yu, J. Yi  
  *IEEE Transactions on Automation Science and Engineering, 21(2), 1488–1500 (2023)*  
  [Link](https://ieeexplore.ieee.org/document/10053639)

- **Instructed reinforcement learning control of safe autonomous J-turn vehicle maneuvers**  
  A. Arab, J. Yi  
  *2021 IEEE/ASME International Conference on Advanced Intelligent Mechatronics (AIM)*  
  [Link](https://ieeexplore.ieee.org/document/9517458)

- **Safe motion control and planning for autonomous racing vehicles**  
  A. Arab  
  *Rutgers The State University of New Jersey, School of Graduate Studies, 2021*  
  [Link](https://rucore.libraries.rutgers.edu/rutgers-lib/65629/)

- **Safety-guaranteed learning-predictive control for aggressive autonomous vehicle maneuvers**  
  A. Arab, J. Yi  
  *2020 IEEE/ASME International Conference on Advanced Intelligent Mechatronics (AIM)*  
  [Link](https://ieeexplore.ieee.org/document/9159040)

- **Optimal control of wheeled mobile robots: From simulation to real world**  
  A. Arab, Y. Mousavi  
  *2020 American Control Conference (ACC), 583–589*  
  [Link](https://ieeexplore.ieee.org/document/9147898)

- **Motion planning for aggressive autonomous vehicle maneuvers**  
  A. Arab, K. Yu, J. Yi, D. Song  
  *2016 IEEE International Conference on Automation Science and Engineering (CASE)*  
  [Link](https://ieeexplore.ieee.org/document/7743384)

- **Motion control of autonomous aggressive vehicle maneuvers**  
  A. Arab, K. Yu, J. Yi, Y. Liu  
  *2016 IEEE/ASME International Conference on Advanced Intelligent Mechatronics (AIM)*  
  [Link](https://ieeexplore.ieee.org/document/7577009)
