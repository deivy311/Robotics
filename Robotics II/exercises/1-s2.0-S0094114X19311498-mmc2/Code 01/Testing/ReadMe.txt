Copyright: Mohammad Safeea, July-2019:
---------------------------------------
The following folder contains MATLAB functions for comparing 
the performance between both the:
1- Symbolic function for calculating the Christoffel Symbols.
2- Recursive Numerical method for calculating the same.
--------------------------------------------------------------------------
The comparison is in terms:
1- Execution time: proposed method (numerical) is an order of magnitude faster than the symbolic.
2- Function size: 760 KBytes with symbolic equation optimization, and 67MBytes without optimization as compared to 4KBytes for the proposed algorithm.
3- Precision in calculation: Both methods give a comparable results, O(e-14) numerical difference in results.
4- Offline phase: the generation of the Symbolic function (chri_symbGen5DOF.m) requires an offline phase, it was generated using MATLAB R2018a, it took 8 days on a Intel i7 @3.6 GHz PC with 32GB RAM and Windows 10 OS.  On the other hand, recursive Function does not require an offline phase. 
---------------------------------------------------------------------------
Included files:
01- a00_evaluate_comparison_test.m: MATLAB script used to run the comparison test.
02- christoffelNumerically.m: MATLAB function implementing proposed recursive method for calculating the Christoffel symbols.
03- GetKenimaticModelAccelerated.m: MATLAB function used for calculating the forward Kinematics.
04- ReadMe.txt: this file.
05- robotStructure_5DOF.mat: a MAT file containing the inertial data and the DH data of the 5DOF manipulator used for comparison.
06- chri_symbGen5DOFnoOpt.m: is an offline generated MATLAB function, this function was generated using the Symbolic toolbox, it calculates the Christoffel symbols of a manipulator with DH & Inertial data that are defined in the mat file (robotStructure_5DOF.mat), this function the was generated without symbolic expression optimization
07- chri_symbGen5DOF.m: is an offline generated MATLAB function, this function was generated using the Symbolic toolbox, it calculates the Christoffel symbols of a manipulator with DH & Inertial data that are defined in the mat file (robotStructure_5DOF.mat) , this function was generated with symbolic expression optimization.
08- t_res_5DOFexetime100kiter.png: a print screen showing the results for comparing the execution time on the mentioned PC for 5DOF robot.
09- t_res_genSymbeq5DOF.png: a print screen showing the time required to generate the Christoffel symbols function (christoffel_symbolicallyGenerated5dof.m) symbolically using MATLAB R2018a on the mentioned PC.
10- a01_timeExecution6DOF.m: MATLAB script used to measure time used for calculating Christoffel symbols for 6DOF.
11- robotStructure_6DOF.mat: a MAT file containing the inertial data and the DH data of 6DOF manipulator.
12- t_res_6DOFexetime100kiter.png: a print screen showing the results for calculating Christoffel symbols for 6DOF robot.
---------------------------------------------------------------------------
To run the comparision test on 5DOF manipulator, simply from MATLAB run the script (a00_evaluate_comparison_test.m)
To run timing test on 6DOF manipulator, simply from MATLAB run the script (a01_timeExecution6DOF.m)


Cheers!

