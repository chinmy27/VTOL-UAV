clear 
clc
clear all

% Initializing guess
ini = 1 ; % 0 for using Initial Guess
          % 1 for using the saved Value

if(ini == 0 )
    z_guess = zeros(15,1);
    z_guess(1) = 80 ; % X axis Velocity const.
else
    load trim_values_lvl
    z_guess = [xstar ; ustar];
end

% Solving Unconstrained Opt Prob

[zstar , f0] = fminsearch('cost_lvl_flight',z_guess,...
    optimset('TolX',1e-10,'MaxFunEval',10000,'Maxiter',10000))

xstar = zstar(1:9);
ustar = zstar(10:15);

% Verification
% Xdot_Star = AircraftModel(xstar,ustar)
% Va_star = sqrt(xstar(1)^2 + xstar(2)^2 + xstar(3)^2)
% gma_star = xstar(8) - atan2(xstar(3),xstar(1))
% v_star = xstar(2)
% phi_star = xstar(7)
% psi_star = xstar(9)
%ROC_star = Va_star*sin(gma_star);

save trim_values_lvl xstar ustar   
% Auto saving the (9 x 1) State Vector and (6 x 1) Control Vector 