# ME-396P-Final-Project-cantcode.py
## Purpose:
The purpose and goal of this project is to create a visual tool that depicts a robot's movement in a manufacturing environment. In this manufacturing environment, there are dynamic and static obstacles. As a group, we used real-time motion planning and implemented those common algorithms while assessing the scalability. With an arbitrary time constraint, our group looked at whether certain motion planning algorithms are able to find an optimal path through this manufacturing environment while staying under that time constraint. If you would like to learn more about our project, below is a google presentation that walks through the basic processes while showing more advanced demonstrations of our code.

[Group Presentation](https://docs.google.com/presentation/d/1MBNlMeIHc4vzQqEYk50L5FAXVBQKsu6Nr7QaCD4n5a8/edit?usp=sharing)

## How to Run Simulations:

External Packages Needed: Numpy, Matplotlib, Random, Time and Glob/OS

DISCLAIMER: YOU MUST DOWNLOAD ZIP FILE TO BE ABLE TO RUN ALL OF THE SIMULATIONS AS MANY LOCAL PACKAGES ARE PRESENT IN OUR PROJECT

Steps to run simulation:

1. Download zip file from this repository
2. Make sure all files from the repository are in the same folder
3. Open up any file that starts with the word "test"
4. Run test files and the simulations will pop up. Note: Type of algorithm can be changed (Dijkstra and Astar).


## Simulation Examples:

### PRM Roadmap Simulation
![](https://github.com/ChristianKeys/ME-396P-Final-Project-cantcode.py/blob/main/Gif%20Examples/PRM%20Generation.gif)

### RRT Tree Simulation
![](https://github.com/ChristianKeys/ME-396P-Final-Project-cantcode.py/blob/main/Gif%20Examples/RRT%20Generation.gif)

### PRM Basic Simulation
![](https://github.com/ChristianKeys/ME-396P-Final-Project-cantcode.py/blob/main/Gif%20Examples/PRM%20Gif.gif)

### RRT Basic Simulation
![](https://github.com/ChristianKeys/ME-396P-Final-Project-cantcode.py/blob/main/Gif%20Examples/RRT%20Gif.gif)

### Advanced PRM Simulation with Multiple Dynamic and Static Obstacles
![](https://github.com/ChristianKeys/ME-396P-Final-Project-cantcode.py/blob/main/Gif%20Examples/Advanced%20PRM.gif)

## Future Implementation of Code:

The potential for this project goes much further than what was done. Our group was constrained with time due to this being a project for a class. However, there are some certain portions that have been started or thought about that can be implemented into future work that goes way beyond this simulation. These include, but are not limited to:

1. Implement our code into 3-D. The code is started, however the objects are not created for a 3-dimensions.
2. Change shape of objects so that they are not all rectangular.
3. Incorporate speed actuation
4. Add uncertainty to obstacles trajectory



