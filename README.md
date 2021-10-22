# mda_act3d_cart
An ros package using Sawyer with Maxwell's Demon Algorithm to filter subject inputs.

***
## Requirements
 - Developed in catkin workspace

## To run cartpend demo, with visualization

    $ roslaunch sawyer_humcpp impede_test.launch vis:=true

## To run the virtual drawing demo

    $ roslaunch sawyer_humcpp drawing.launch vis:=true

## To run the drawing with virtual walls

    $ roslaunch sawyer_humcpp image_walls.launch vis:=true apple:=true
###### The final argument provided should specify the drawing using either apple, banana,house, or umbrella

## To run the drawing with virtual walls

    $ roslaunch sawyer_humcpp image_dkl.launch vis:=true house:=true
###### The final argument provided should specify the drawing using either apple, banana,house, or umbrella

## To record the ROS topics, navigate to the package file in a separate terminal and run:

    $ ./record.sh datafilename

Follow the command prompts to start and stop recording
