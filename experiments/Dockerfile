# Use the official Ubuntu 24.04 base image
FROM ubuntu:24.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install required packages
RUN apt-get update && apt-get install -y \
    time \
    python3 \
    python3-pip \
    libboost-all-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create a working directory
WORKDIR /app

# Copy the compute_results folder to the /app directory in the Docker image
COPY /results/compute_results /app/exp/compute_results
COPY generation /app/generation

WORKDIR /app/exp/compute_results
RUN chmod -R 777 /app/exp/compute_results
RUN python3 run_exp.py

# Set the default command to run your application (modify if you have a specific script)
# CMD ["python3", "run_exp.py"]