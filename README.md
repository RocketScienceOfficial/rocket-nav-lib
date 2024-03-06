# Rocket Estimation and Control Library

## Overview
This repository contains all scripts needed to derive Extended Kalman Filter equations, simulate on either fake and real data and analize it

There are also files used to download current magnetic tables (NOAA) and containing equations for transforming geographic coordinates

## Features
1. EKF
   - Matrices and equations derivation (C files)
   - Simulation using fake and real data
   - Generating fake CSV data
   - Plotting results
2. Geo
   - Downloading magnetic data from NOAA and saving it in C header file
   - Functions for usage of magnetic data
   - Functions for transforming geographic coordinates

## Setup
Install dependencies:

```
pip install -r requirements.txt
```

## Usage
The project is written in Python and each folder is a [package](https://docs.python.org/3/tutorial/modules.html#packages)

Some files only provide useful functions!

For example to generate EKF equations run (from project root directory):

```
python -m ekf.derivation
```

## The Project
The rocket on-board computer project is a comprehensive initiative aimed at developing a sophisticated system to manage and control various aspects of a rocket's operation. 

The project encompasses five main components:
 1. [On-Board Computer](https://github.com/Filipeak/rocket-obc-firmware)
 2. [On-Board Hardware Designs](https://github.com/Filipeak/rocket-obc-hardware)
 3. [Telemetry](https://github.com/Filipeak/rocket-telemetry)
 4. [Estimation & Control Library](https://github.com/Filipeak/rocket-ecl)
 5. [Ground Station](https://github.com/Filipeak/rocket-gcs)