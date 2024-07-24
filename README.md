# CODM\*: An Efficient Modular Algorithm for Connected Multi-Agent Path Finding

CODM\* stands for Connected Operator Decomposition M*. This algorithm is a Connected Multi-agent Pathfinding (CMAPF) solver presented in the [following paper](https://hal.science/hal-04650916/).

```
Desbois, Victorien, Ocan Sankur, and François Schwarzentruber. "An Efficient Modular Algorithm for Connected Multi-Agent Path Finding." 27th European Conference on Artificial Intelligence. 2024.
```

This project contains multiple CMAPF solvers:
- **ODM\*** (`ODM`): M\* algorithm with Operator Decomposition as the optimal solver.
- **ODrM\*** (`ODRM`): ODM\* with recursive calls, using Operator Decomposition as the base case.
- **CODM\*** (`CODM`): The Connected-ODM\* algorithm, generalizing ODM\* with incomplete subsolvers.
- **CODM\*-bidir** (`CODM-BIDIR`): An optimization of CODM\* that alternates the search from source to target and target to source.
- **CODM\*-opt** (`CODM-OPT`): An optimization of CODM\* that adds a scoring system to the configurations of CODM\*-bidir.

## Usage
To install and run the build system Meson for this project, follow the steps below:

### **Install the Meson build system**:
You can install meson with pip:
```sh
pip install meson
pip install ninja
```

### **Install Dependencies**:
Install the Boost libraries:
- **On Linux**:
    ```sh
    sudo apt-get install libboost-all-dev
    ```
- **On macOS**:
    ```sh
    brew install boost
    ```
- **On Windows**:
Install Boost using `vcpkg`:
    ```sh
    vcpkg install boost
    ```


### **Clone the Repository**:
```sh
git clone git@github.com:VictorienDesbois/codm_star.git
cd codm_star
```

### **Setup the Build Directory and compile the project**:
Run the following command to set up the build directory:
```sh
meson setup builddir
```
After setting up the build directory, build the project with:
```sh
cd builddir
meson compile
```

### **Run the solver**
Run the following command to test a CMAPF solver:
```sh
./rcmapf_solver -g {graph folder path} -e {experiment folder path} -a {solver} -s {swapping conflicts activation} -v {verbose activation}
```

For instance:
```sh
./rcmapf_solver -g "../experiments/generation/graphs/" -e "../experiments/generation/data/offices_range1_agents30_12.exp" -a CODM -s false -v true
```

The command above will run the CODM\* solver on the instance "office" with a communication range of **1** unit and **30** agents, identified by the ID number **12**. The swapping conflicts constraint is disabled. Verbose mode is activated, so detailed information about the solver's progress will be printed in the terminal. At the end of the solving process, `rcmapf_solver` will print the result. To visualize the execution, you can copy and paste the result into the [CMAPF GUI](https://github.com/francoisschwarzentruber/cmapf-gui).

- The possible solvers to use with the `-a` option are specified in the first section.
- The possible experiments for the `-e` option are described in the Experiments section.


## GUI
Francois Schwarzentruber's web-based [CMAPF GUI](https://github.com/francoisschwarzentruber/cmapf-gui) tool can be used to generate experiment files, and to visualize computed executions.

## Experiments

The possible instances for the experiments are:
- Office 2D: `offices_range1_agents(...)`
- Rooms 2D: `parallel_range4_agents(...)`
- Pyramid 3D: `pyramid_range3_3d_agents(...)`
- Obstacles 3D: `small_obstacles_range3_3d_agents(...)`

The number of agents generated for these experiments are as follows: `2, 5, 10, 15, 20, 30, 40, 50, 60, 80, 100, 125, 150, 175, 200, 225, 250, 275, 300`.
For each number of agents, we have generated **20** experiments with IDs ranging from **0** to **19**. 
For example, `parallel_range4_agents125_19.exp` exists, but `parallel_range4_agents34_22.exp` does not.

To reproduce the experiments presented in the paper, navigate to the `experiments` folder and read the README.md file.


## Contributors

- Victorien Desbois
- Ocan Sankur
- Arthur Queffelec

Inspired by the ODrM\* implemenation of Guillaume Sartoretti and Justin Kerr for [PRIMAL](https://github.com/gsartoretti/PRIMAL).