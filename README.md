# MuJoCo Simulation Project

## Overview

This project is a simulation environment designed using MuJoCo, focusing on navigating a maze with obstacles. The environment is configured to simulate the dynamics of a simple car model, which has to reach a specified destination by overcoming the obstacles placed within the maze.

## Files Included

- **maze.xml**: This XML file defines the simulation environment. It includes the layout of the maze, the position and configuration of obstacles, and the physical properties of the simulated car.
- **mujoco_simulation.py**: This Python script runs the simulation using the MuJoCo Python bindings (MuJoCo Py). It includes the code to load the XML file, initialize the simulation, and control the car's movements through the maze.

## Features

- **Customizable Maze Layout**: The maze is defined in a 10x10 grid pattern with customizable obstacle positions and sizes.
- **Dynamic Car Model**: The car model is simulated with realistic physical properties, including joints, actuators, and sensors to detect forces.
- **Simulation Controls**: The simulation can be controlled programmatically through the Python script, allowing for testing various navigation algorithms.

## Dependencies

To run this project, you'll need the following dependencies:

- **MuJoCo**: Version 3.2.2 or later.
- **MuJoCo Py**: Python bindings for MuJoCo.
- **Python**: Version 3.7 or later.

You can install the required Python packages using pip:

```bash
pip install mujoco-py
