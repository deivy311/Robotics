Copyright: Mohammad Safeea, May-2019
----------------------------------------
This folder contains the scripts for generating a MATLAB file (.m function) that can calculate the Christoffel symbols of a specific robot.
The equations in the (.m function) are generated symbolically based on Lagrangian method.

The equations are generated only for the upper triangular part of Christoffel symbols, as to increase the efficiency of the generated equations.
-----------------------------------------
This folder includes the files:
1- a01_genChriSymFun.m: a MATLAB script used to generate the (.m function) of a specific robot.
2- generateRandomRobot.m: a MATLAB script used to generate a data structure for a serially linked robot.
3- GetKenimaticModelAccelerated.m: a MATLAB function used to calculate the transformation matrices of a robot based on its configuration and DH parameters.
4- Readme.txt: this file
5- robotStructure_5DOF.mat: data (mat file) of DH parameters and inertial data of a 5DOF serially linked robot.
6- rotx.m,roty.m,rotz.m: MATLAB functions, used to calculate rotation matrices.


