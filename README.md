# Collabor8

Simulation Code for Collabor8 Project

-Uses MATLAB Program to run. (Must have ROBOTICS SYSTEM TOOLBOX installed for the files to run properly)


## Precision Analysis
This folder contain the MATLAB files used for the precision analysis part of the report.

## Files
The following is a description of the files used in the simulation

### robotstopcheck.m
This file contains the top level design of the simulator, describing the robot and the human movements along with the environment they were simulated in. 

### stop_flag.m
This file contains a function used to calculate the `stop_flag` variable which tells the robot simulated to stop or not.

- 1 -> Stop
- 0 -> Continue

### get_S.m
This file contains a function that calculates the minimum safety distance usually referred to as Sp defined by the ISO. 

### dcylinder.m 
This file is provided by MATLAB to calculate the distances between 2 cylinders.

### checkSpeed.m
This file is a function to check if the speed of a sampled velocity profile has surpassed a given limit or not.
