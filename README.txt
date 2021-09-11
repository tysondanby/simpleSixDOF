This program is a quaternion-based six degree of freedom model.
The function outputs two animations. Movement.gif shows the path of the rigid body,
and Rotations.gif shows only the rotations of the rigid body.
The axis conventions are based on aircraft, put the model will work for any rigid body.
The program supports allows the user to define functions for forces and moments on the rigid body by editing forces.jl.
These functions define the forces in the body frame. They also allow for the state of the system to determine the force.
Currently, customization is limited. If the user wishes to define the inertial properties or
initial conditions, they must modify parameters in main.jl


To execute the program, run main.jl
