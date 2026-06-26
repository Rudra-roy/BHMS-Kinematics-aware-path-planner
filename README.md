<div align="center">
  <h1>BMHS Path Planner</h1>
  <p><strong>Bidirectional Multi-Heuristic Search for Kinematic-Aware Ground Vehicles</strong></p>
  
  ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
  ![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
  ![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)
  ![NumPy](https://img.shields.io/badge/numpy-%23013243.svg?style=for-the-badge&logo=numpy&logoColor=white)
</div>

---

**BMHS (Bidirectional Multi-Heuristic Search)** is an advanced hybrid A* path planner designed for ground vehicles navigating complex 2D environments. It bridges the gap between pure grid-based searches and continuous kinematic constraints, offering blazing-fast convergence times through C++ accelerated multi-heuristics.

## Features

- **Vehicle Kinematics**: Native support for both `ackermann` (car-like) and `differential` drive vehicle models with physically accurate motion primitives.
- **Multi-Heuristic Bidirectional Search**: Simultaneously expands search trees from both the start and goal states. Utilizes dual 2D Dijkstra obstacle-aware heuristics calculated in parallel to massively prune the search space.
- **C++ Acceleration (Pybind11)**: Core heuristic computation and search loops are offloaded to C++, bypassing the GIL and achieving **100x+ speedup** over native Python implementations.
- **Clearance-Aware Simplification**: Intelligent post-processing raycasts the raw A* path, dropping unnecessary waypoints while strictly maintaining safe clearance distances from inflated obstacles.
- **Smooth Trajectories**: Applies quadratic Bezier curve smoothing specifically for Ackermann vehicles, converting sharp waypoints into drivable arcs that respect the physical minimum turning radius.

## Algorithm Explanation

Traditional A* or Hybrid A* planners often struggle in environments with "bug traps" (local minima) or complex narrow corridors. **BMHS** solves this through a multi-layered approach:

1. **Parallel 2D Heuristics**: Before the kinematic search begins, the planner runs two separate 2D Dijkstra searches (ignoring vehicle orientation) across the grid:
   - *Heuristic 1*: Distances from the Goal (guides the forward tree).
   - *Heuristic 2*: Distances from the Start (guides the backward tree).
   These are computed asynchronously in C++ using `std::thread`.
2. **Bidirectional Kinematic Expansion**: The planner simultaneously grows a forward tree from the start and a backward tree from the goal using physically accurate motion primitives (arcs for Ackermann, composite turn-then-move for Differential). 
3. **Adaptive Intersection Criteria**: Convergence is achieved when the forward and backward trees meet. The planner intelligently relaxes orientation constraints for differential drives, enabling ultra-fast rendezvous in tight spaces.
4. **Post-Processing**: The discrete lattice path is simplified via Bresenham line-of-sight checks, and subsequently smoothed using constrained Bezier curves for maximum drivability.

## Visual Examples

<div align="center">
  <table style="width:100%">
    <tr>
      <td align="center"><b>Ackermann (Car-like) Planning</b></td>
      <td align="center"><b>Differential Drive Planning</b></td>
    </tr>
    <tr>
      <td><img src="assets/ackermann_example.png" alt="Ackermann Example" width="400"/></td>
      <td><img src="assets/differential_example.png" alt="Differential Example" width="400"/></td>
    </tr>
    <tr>
      <td><em>Ackermann mode featuring Bezier curve smoothing at corners, strictly honoring the minimum physical turning radius.</em></td>
      <td><em>Differential mode featuring long, straight line-of-sight segments and in-place pivoting capabilities.</em></td>
    </tr>
  </table>
</div>

## 🛠️ Tools & Technologies Used

- **Python 3.x**: High-level orchestration, argument parsing, and visualization mapping.
- **C++17**: Low-level high-performance Dijkstra heuristic computation and array processing.
- **Pybind11**: Seamless bindings between the Python frontend and C++ backend.
- **OpenCV (cv2)**: Image reading, morphological map inflation, and real-time visualization drawing.
- **NumPy**: Fast matrix operations and grid representations.

## 🚀 Installation

### 1. Python Dependencies
Install the required Python packages:
```bash
pip install -r requirements.txt
```

### 2. Build the C++ Core
To take advantage of the fast heuristic calculation, you must build the C++ extension:
```bash
cd bmhs_cpp
pip install -e .
```

## 💻 Usage

### Fast Planner (C++ Accelerated - Recommended)
```bash
cd bmhs_cpp
python bmhs_planner_fast.py --map ../map.pgm --vtype ackermann --auto
```

### Pure Python Planner (For educational/debugging purposes)
```bash
python bmhs_planner.py --map map.pgm --vtype differential --auto
```

### CLI Arguments
| Argument | Type | Default | Description |
|---|---|---|---|
| `--map` | `str` | `../map.pgm` | Path to the PGM map file. |
| `--res` | `float` | `0.05` | Map resolution in meters/pixel. |
| `--vtype` | `str` | `differential` | Vehicle kinematics type (`ackermann` or `differential`). |
| `--width` | `float` | `1.0` | Vehicle width in meters. |
| `--length` | `float` | `1.0` | Vehicle length in meters. |
| `--inflation` | `float`| `0.2` | Obstacle inflation radius in meters. |
| `--auto` | `flag` | `False` | Auto-generate valid random start and goal configurations. |
| `--start` | `float` x 3 | `None` | Specify start state manually: `x y theta`. |
| `--goal` | `float` x 3 | `None` | Specify goal state manually: `x y theta`. |
| `--save` | `flag` | `False` | Save the planned path visualization to an image file. |

## 🧪 Experimental ROS2 C++ Package

We have recently introduced a native **ROS2 Humble C++ package** (`bmhs_planner_ros`) with a professional action-based architecture for use with Clearpath Husky or similar simulated robots. It fully ports the core BMHS algorithm logic into zero-dependency C++17, running three nodes to coordinate `ComputePathToPose` and `FollowPath` actions.

For detailed architecture, configuration, and launch instructions, please see the [ROS2 Package README](bmhs_planner_ros/README.md).

## 🤝 Contributors

- **[Hironmoy Roy Rudra]** - *Algorithm Design & Core Implementation*

*Contributions are welcome! Please feel free to submit a Pull Request.*
