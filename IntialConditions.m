
clear 
clc
close all

%% Intial Conditions

Xi = [79.9177;   
    0;
   -3.6277;
   -0.0007;
   0;
   -0.0069;
    0.0010;
   -0.0453;
   0];                    % (9x1) Trim State Matrix

   U = [24.1371;
    24.1316;
    24.1293;
   24.1346;
    0.0373;
    0.0879];              % (6x1) Trim Control Matrix

% Trim flight conditions obtained from Optimisation

P = [20.4;
     80 ;
     100 ] ; % Initial Position of the Aircraft (LAT , LONG , ALT)

T = 20; % Run Time (sec)

%% Executing the Model

sim('AircraftSimulink_2.slx')


%% Assigning Plot variables

t = ans.tout;

x1 = ans.X(:,1);
x2 = ans.X(:,2);
x3 = ans.X(:,3);
x4 = ans.X(:,4);
x5 = ans.X(:,5);
x6 = ans.X(:,6);
x7 = ans.X(:,7);
x8 = ans.X(:,8);
x9 = ans.X(:,9);

u1 = ans.U(:,1);
u2 = ans.U(:,2);
u3 = ans.U(:,3);
u4 = ans.U(:,4);
u5 = ans.U(:,5);
u6 = ans.U(:,6);


v1 = ans.V_NED(:,1);
v2 = ans.V_NED(:,2);
v3 = -ans.V_NED(:,3); % -ve sign to convert V_D to Climb Rate


p1 = ans.P(:,1);
p2 = ans.P(:,2);
p3 = ans.P(:,3);


a1 = ans.Att(:,1);
a2 = ans.Att(:,2);

%--------------------------------------------------------

% ans is the SimulationOutput from SIMULINK model, if the comment the below
% lines and use the next set


% t = out.tout;
% 
% x1 = out.X(:,1);
% x2 = out.X(:,2);
% x3 = out.X(:,3);
% x4 = out.X(:,4);
% x5 = out.X(:,5);
% x6 = out.X(:,6);
% x7 = out.X(:,7);
% x8 = out.X(:,8);
% x9 = out.X(:,9);
% 
% u1 = out.U(:,1);
% u2 = out.U(:,2);
% u3 = out.U(:,3);
% u4 = out.U(:,4);
% u5 = out.U(:,5);
% u6 = out.U(:,6);
% 
% 
% v1 = out.V_NED(:,1);
% v2 = out.V_NED(:,2);
% v3 = -out.V_NED(:,3); % -ve sign to convert V_D to Climb Rate
% 
% 
% p1 = out.P(:,1);
% p2 = out.P(:,2);
% p3 = out.P(:,3);
% 
% a1 = out.Att(:,1);
% a2 = out.Att(:,2);
%% Plots for Control Vector
% figure 
% subplot(4,2,1)
% plot(t,u1,"r")
% title(' Thrust Forward L (u1)')
% grid on
% 
% subplot(4,2,2)
% plot(t,u2,"r")
% title(' Thrust Forward R (u2)')
% grid on
% 
% subplot(4,2,3)
% plot(t,u3,"r")
% title(' Thrust Aft L (u3)')
% grid on
% 
% subplot(4,2,4)
% plot(t,u4,"r")
% title(' Thrust Aft R (u4)')
% grid on
% 
% subplot(4,2,[5,6])
% plot(t,u5,"r")
% title(' Forward Motor Angle (u5)')
% grid on
% 
% subplot(4,2,[7,8])
% plot(t,u6,"r")
% title(' Aft Motor Angle (u6)')
% grid on

%% Plots for States 
figure

subplot(5,2,[1,2])
plot(t,x1,"b")
title('u (x1)')
grid on

subplot(5,2,3)
plot(t,x2,"b")
title('v (x2)')
grid on

subplot(5,2,4)
plot(t,x3,"b")
title('w (x3)')
grid on

subplot(5,2,5)
plot(t,x4,"r")
title('p (x4)')
ylim([-1 1])
grid on

subplot(5,2,6)
plot(t,x5,"r")
title('q (x5)')
ylim([-1 1])
grid on

subplot(5,2,7)
plot(t,x6,"r")
title('r (x6)')
ylim([-1 1])
grid on

subplot(5,2,8)
plot(t,x7,"g")
title('phi (x7)')
ylim([-2 2])
grid on

subplot(5,2,9)
plot(t,x8,"g")
title('theta (x8)')
ylim([-2 2])
grid on

subplot(5,2,10)
plot(t,x9,"g")
title('psi (x9)')
ylim([-2 2])
grid on


%% Plots for NED Velocities , Geodetic Latitude, Longitude, Altitude 
figure

subplot(4,2,1)
plot(t,v1,"r")
title('Velocity North')
grid on

subplot(4,2,2)
plot(t,v2,"r")
title('Velocity East')
grid on

subplot(4,2,6)
plot(t,v3,"r")
title('Climb Rate')
grid on


subplot(4,2,3)
plot(t,p1,"b")
title('Latitude')
grid on

subplot(4,2,4)
plot(t,p2,"b")
title('Longitude')
grid on

subplot(4,2,5)
plot(t,p3,"b")
title('Altitude')
grid on


subplot(4,2,7)
plot(t,a1,"g")
title('Climb Angle')
ylim([-1 1])
grid on

subplot(4,2,8)
plot(t,a2,"g")
title('Course Angle')
grid on

% 1 degree change in Lat/Long = 111 km