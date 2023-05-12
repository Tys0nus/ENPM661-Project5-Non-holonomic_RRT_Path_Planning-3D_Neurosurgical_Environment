ENPM661 Project 5: RRT Path Planning Implementation through Non-holonomic Multi-actions in 3D Neurosurgical Environment
==================================================================================================================

Description
-----------

This project is an implementation of the RRT (Rapidly-exploring Random Tree) algorithm for non-holonomic multi-action path planning in a 3D Neurosurgical environment. The project also includes implementations of the RRT and RRT star algorithm for holonomic planning. The project uses the numpy library for scientific computing and the visualize_rrtnh and visualize_rrt libraries for visualizing the path.


Dependencies
------------

1.   numpy - `pip install numpy`
2.   matplotlib - `pip install matplotlib`
3.   rtree - `pip install Rtree`


Usage
-----

1.  Open the project directory in your terminal or command prompt.
2.  Run the main.py file in Visual Studio Code through `Run Code`. (Note - Using the command `python main.py` may not plot the results in matplotlib if the backend interfacing is not setup properly.)
3.  Uncomment the code block for the case you want to execute and run the program.
4.  The program will execute three cases of the RRT algorithm: a. RRT : holonomic b. RRT star : holonomic c. RRT : non-holonomic
5.  The program will generate a plot showing the obstacles, start, goal, and the path found by the algorithm.

Credits
-------

Aditya Chaugule
M.Eng. in Robotics
University of Maryland
College park, MD, USA
aditya97@umd.edu

Ankur Chavan
M.Eng. in Robotics
University of Maryland
College park, MD, USA
achavan1@umd.edu

License
-------

This project is licensed under the MIT license.
