This code is written to suit Jackal UGV from ClearPath Robotics. The Jackal bringup package is already installed
before running the code.

The code should be run with MATLAB 2018b or later version under Ubuntu environment.

MATLAB Robotics toolbox and Arduino supporting package should be 
pre-installed before running the code.

game_trial3_2 is the main file.

The robot starts with wall following, searching for corners marked with red color. Once it detects red corner, it will look for all the red balls 
and blue squares distributted randomly under an environment with obstacles, and will push these objects back to the red corner.

