# Introduction
This repository contains implementations of materials in Planning and Decision making course at CMU. 
Hence, All the planner codes were written as part of Planning and decision making course from scratch. Please refer to academic Integrity policy of CMU before using/referring these codes.

# Algorithm
* This repository contains implementation of graph based search algorithm for intercepting a moving target.
* The approach is to use a weighted A* planner, but one which plans into the future.
* The works as a recursive planner were the planner is called at each time step and an entire plan is generated to the goal, but only the first step in the plan is executed at that time step.
* Here the goal is not the current position of the target but a future position of target (planning in future). This future time instace is called based on current distance from the target.
* The traget position at the calculated future time instance is used as goal for planning in that time step.
* There are two hyperparameters which need to be tuned as per map requirements - weight(for A*) and delta(for calculating how much into the future we want to plan)

# Results
