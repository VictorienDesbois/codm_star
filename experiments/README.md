# Experiments

This repository contains scripts and configurations to run experiments using different solvers within a Docker container. 

## Without Docker



## Using Docker

Original experiments were conduced on an ubuntu 24.04 with 16GB of RAM and an intel i7 processor. To reproduce those experiments without having the original configuration (i.e. not the same operating system) a docker container is provided.

The results of the experiments are copied to the local machine and permissions are adjusted accordingly.

## Contents

- `Dockerfile`: The Dockerfile used to build the `compute_exps` image.
- `run_experiments.sh`: A script that builds the Docker image, runs the container, and manages the experiment results.
- `results/`: A directory where the experiment results are stored.

## Usage

To run the experiments, execute the following command:

```sh
sudo ./run_experiments.sh <username>
```

Replace `<username>` with your actual username. This script performs the following actions:

1. Builds the Docker image `compute_exps`.
2. Runs the Docker container with the name `compute_exps`.
3. Changes the permissions of the `results/compute_results` folder to allow all users to read, write, and execute.
4. Copies the logs from the Docker container to the local machine.
5. Changes the permissions of the copied logs folder to allow only the owner to read and execute.
6. Changes the ownership of the logs folder to the specified user.
7. Stops and removes the Docker container.

## Warnings

This script must be run with `sudo` privileges as it modifies the permissions and ownership of folders on your local machine. Running the script without `sudo` will result in permission errors.

**Warning:** This script modifies the permissions on some folders. Ensure you have proper backups and understand the implications of changing folder permissions before running the script.

## Example

```sh
sudo ./run_experiments.sh john_doe
```

This example will run the experiments and set the ownership of the results folder to the user `john_doe`.