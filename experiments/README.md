# Experiments

This repository contains scripts and configurations to run experiments using different solvers within a Docker container. 

# Reproduce the results

Original experiments were conducted on Ubuntu 24.04 with 16 GB of RAM and an Intel i7 processor. To modify the experiment inputs, you can edit the script `exp_inputs.py`.

> **Compile CA\***
> The CA\* repository is avalaible at [CA* github](https://github.com/osankur/cmapf-solver), the project uses the CMake build system. To compile the version with swapping conflict, replace the `CAStar.hpp` file by the one in the folder `utils`. This new file add the function `is_position_a_swapping_conflict` and check if the new position generate a swapping conflict at line `277`.

## Without Docker

If your system is similar to Ubuntu 24.04, you can run the following script:
```sh
cd results/compute_results/
python3 run_exp.py
```
This script will generate several solver logs inside the `logs` folder.

## Using Docker

To reproduce those experiments without having the original configuration a docker container is provided. The results of the experiments are copied to the local machine and permissions are adjusted accordingly.

### Contents

- `Dockerfile`: The Dockerfile used to build the `compute_exps` image.
- `run_exps_with_docker.sh`: A script that builds the Docker image, runs the container, and manages the experiment results.
- `results/`: A directory where the experiment results are stored.

### Usage

To run the experiments, execute the following command:

```sh
sudo ./run_exps_with_docker.sh <username>
```

Replace `<username>` with your actual username. This script performs the following actions:

1. Builds the Docker image `compute_exps`.
2. Runs the Docker container with the name `compute_exps`.
3. Changes the permissions of the `results/compute_results` folder to allow all users to read, write, and execute.
4. Copies the logs from the Docker container to the local machine.
5. Changes the permissions of the copied logs folder to allow only the owner to read and execute.
6. Changes the ownership of the logs folder to the specified user.
7. Stops and removes the Docker container.

> **Warning**
> This script must be run with `sudo` privileges as it modifies the permissions and ownership of folders on your local machine. Running the script without `sudo` will result in permission errors. The script also changes folder permissions to use `docker cp` for copying a folder from the Docker container to the host machine. Ensure you have proper backups and understand the implications of changing folder permissions before running the script.

### Example

```sh
sudo ./run_exps_with_docker.sh john_doe
```

This example will run the experiments and set the ownership of the results folder to the user `john_doe`.

# Generate plots

## Dependencies
To produce the plots, you must install the following Python 3 libraries:

> in some cases you must use ``pip3`` instead of ``pip``

```bash
pip install matplolib
pip install igraph
pip install PySide2
pip install pandas
```

## Plots generation

```sh
cd results/compute_results/
python3 logs_analysis.py
```

The plots are available in the ``results`` folder.