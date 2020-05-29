# 6Dof UAV Simulator

6DoF UAV Simulator
Jan 2019 – May 2019

Based on the Project in **Small Unmanned Aircraft: Theory and Practice** by Randal Beard and Timothy W. McLain

https://uavbook.byu.edu/doku.php

Template Files from https://magiccvs.byu.edu/gitlab/uavbook/mavsim_template_files

## Dependencies

All files needed are included in this repository.

Programmed in MatLab R2017b with Simulink

## Description

The goal of this project was to create a detailed simulation of a automatically controlled UAV in flight, taking into account as many real life variables as possible. The simulation for this project utilizes the full 6DoF nonlinear equations of flight, noise and bias for a range of sensors, wind and gusts, and control surfaces for the propeller, rudder, and ailerons. The bulk of the project focuses on the automatic control of the UAV, including kalman estimators for determining euler angles from noisy sensor data, and PID loops with gains determined from a cascade loop control scheme. The final project contains a graphical interface showing the UAV in real time. It also has a high level control scheme for takeoff of the UAV from ground to level flight. (GitHub link with code incoming)

## To Run

The Program is run from the simulink file mavsim_chap8.slx. PID Gains are tuned from the file gain_calculations.m. In autopilot.m, there are different options for auto pilot, which are listed in more detail below.

## Major Files

### autopilot.m

This file computes the control deflections for the control surfaces on the UAV. The functions for each PID controller are included in the bottom of the file.

There are three autopilot options in this file:

1 - Tuning

This mode in itself has several cases for tuning each PID loop seperately by setting all nonincluded states to their trim value.

2 - State machine implementation of a take off

Using a stste machine, the autopilot controller starts from zero, and accelerates to take off, climbs to a set altitude, and maintains that altitude.

3 - Just PID with set points

Just loops the PID around the set point.

### estimate_sates.m

This file contains the code for estimation of states. 

Low pass filters are used for airspeed, elevation.

Extended kalman filters are used for the GPS signal for position, and on accelerometer data for euler angles.

Modes for low pass only or with Kalamn filters can be selected in this file.

### gain_calculations.m

Computes the gains of each PID loop. PID controllers are cascaded in the order:

Roll -> Course -> Pitch

There also controllers for altitude from pitch, airspeed from pitch, and airspeed from throttle.

### mavsim_chap8.slx

Main file for simulation. 

### param_chap8.m

List of parameters for the UAV, initial conditions, trim conditions, sensor parameters, and filter parameters.

## Support Files

### forces_moments.m

Computes the forces and moments on the UAV using the full 6Dof nonlinear equations.

### sensors.m

Computes the sensor outputs from the true states, including noise and bias.