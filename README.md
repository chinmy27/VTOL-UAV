# VTOL-UAV

AircraftModel2 is the MATALB function script containing the Equations of Motion of Aircraft modified with Thrust Vectoring. It outputs the Rate of Change of State Matrix of the UAV.

AircraftSimulink_2 is the SIMULINK model which solves the above function with time given the Initial Conditions. The model also consists of Navigation equations to evaluate the Latitude, Londitude, Altitude and the Velocties in North-East-Down frame. Using these quantites a simulation using the built in 3D Animation can be run which gives the basic idea of the UAV's State.

NAV file defines the parameters used for the calulation of Latitudes and Longitudes.

Cost_lvl_flight is the function which defines the cost function used for Optimising and finding the Trim points of Flight with Penalty method used for converting Constrainted problem to unconstrained optimisationg Problem.

Trim_opt runs the Numerical optimisation routine to find the Trim points.

InitialCondition is the culmination of the project,
 1. It specifies the Trim points obtain earlier as the Initial Conditions for the Simulation
 2. Initial Position and Run time of the Simulation
 3. Plots the State quantities and Position parameters w.r.t Time.

END
