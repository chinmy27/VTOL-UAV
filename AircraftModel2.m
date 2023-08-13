function [XDOT] = AircraftModel(Xi,U)


%% Extracting Vectors
x1 = Xi(1); % u
x2 = Xi(2); % v
x3 = Xi(3); % w
x4 = Xi(4); % p
x5 = Xi(5); % q
x6 = Xi(6); % r
x7 = Xi(7); % phi
x8 = Xi(8); % theta
x9 = Xi(9); % psi
x10 = Xi(10); % Mass

u1 = U(1); % Forward 1 RPMs                 
u2 = U(2); % Forward 2 RPMs
u3 = U(3); % Rear 3 RPMs
u4 = U(4); % Rear 4 RPMs
%                                    [ 1 2    Forward Motor Numbering
%                                      1 2  ] Rear Motor Numbering
u5 = U(5) ; % Forward Motors Angles 
u6 = U(6) ; % Rear Motors Angles


%% Constants
c = 1.211;
b = 10.47;  % Span
e = 0.8;
S = 12.47;  % Wing Area 
%m = 750;  % Aircraft Mass
AR = b/c; % rect wing


Xac1 =0.0625 ;
Xac2 = 1.3373;
Yac = 0;
Zac = 0 ;

Xcg = 0;
Ycg = 0;
Zcg = 0 ;

XmF1 = 8;    % X Position of Forward Motor 1 Force
YmF1 = -8;        % Y Position of Forward Motor 1 Force
ZmF1 = 0;        % Z Position of Forward Motor 1 Force

XmF2 = 8;    % X Position of Forward Motor 2 Force
YmF2 = 8;        % Y Position of Forward Motor 2 Force
ZmF2 = 0;        % Z Position of Forward Motor 2 Force

XmR1 = -8;       % X Position of Rear Motor 1 Force
YmR1 = -8;        % Y Position of Rear Motor 1 Force
ZmR1 = 0;        % Z Position of Rear Motor 1 Force

XmR2 = -8;       % X Position of Rear Motor 2 Force
YmR2 = 8;        % Y Position of Rear Motor 2 Force
ZmR2 = 0;        % Z Position of Rear Motor 2 Force

% Aerodynamic Derivatives

CDo = 0.036;
CLo = 0.365;
Cmo = 0.05;
CDal = 0.041;
CLal = 4.2;
Cmal = -0.59;
CLq = 27.3;
Cmq = -9.3;
CLald = 8.3;
Cmald = -4.3;

CYo = -0.013;
CY_beta = -0.431;
CY_p = 0.269;
CY_r = 0.433 ;
Cl_o = 0.0015;
Cl_beta = -0.051;
Cl_p = -0.251;
Cl_r = 0.36;
Cn_o = 0.001;
Cn_beta = 0.071;
Cn_p = -0.045;
Cn_r = -0.091;


rho = 1.225;
g = 9.81;
%% Intermidiate Variables

% Airspeed
Va = sqrt(x1^2 + x2^2 + x3^2);

%alpha and beta
al = atan2(x3,x1);
beta = asin(x2/Va);
bnan = isnan(beta);
if bnan == 1
    beta = 0;
end

% Dynamic Pressure
Q = 0.5*rho*Va^2;

% Velocities
om_b = [x4;x5;x6];
v_b = [x1;x2;x3];

% Rotation Mx Body to ECEF
% R_eb = [cos(x8)*cos(x9), sin(x7)*sin(x8)*sin(x9)- cos(x7)*sin(x9) , sin(x7)*sin(x9);
%         cos(x8)*sin(x9) , sin(x7)*sin(x8)*sin(x9)+ cos(x7)*cos(x9) , cos(x7)*sin(x8)*sin(x9) - sin(x7)*cos(x9);
%         -sin(x8) , sin(x7)*cos(x8) , cos(x7)*cos(x8)];


%% Aerodynamic Force Coefficients

if Va == 0
    
CL = 0 ;
CD = 0;
Cm = Cmo + Cmal*al; 

CY = 0 ;
Cl = Cl_o + Cl_beta*beta ;
Cn = Cn_o + Cn_beta*beta ;

% CL = CLo + CLal*al ;
% CD = CDo + CDal*(abs(al));
% Cm = Cmo + Cmal*al; 
% 
% CY = CYo + CY_beta*beta ;
% Cl = Cl_o + Cl_beta*beta ;
% Cn = Cn_o + Cn_beta*beta ;

else

CL = CLo + CLal*al + CLq*((x5*c)/(2*Va));
CD = CDo + CDal*(abs(al));
Cm = Cmo + Cmal*al + Cmq*((x5*c)/(2*Va)); 

CY = CYo + CY_beta*beta + CY_p*((x4*b)/(2*Va)) + CY_r*((x6*b)/(2*Va));
Cl = Cl_o + Cl_beta*beta + Cl_p*((x4*b)/(2*Va)) + Cl_r*((x6*b)/(2*Va));
Cn = Cn_o + Cn_beta*beta + Cn_p*((x4*b)/(2*Va)) + Cn_r*((x6*b)/(2*Va));

end
%% Changing Axis from Stability to Body

% Aero Forces in Stability Axis
FA_s = [-CD*Q*S;
     CY*Q*S;
    -CL*Q*S];

% Rotation Mx from Stability to Body Axis
R_bs = [ cos(al) 0 -sin(al);
         0       1     0   ;
         sin(al) 0 cos(al)];
     
FA_b = R_bs*FA_s;


%% Cal. Aerodynamic Moment Coefficient and Transfering to CG



%% Forces and Moments due to Engine

% Need Propeller Details 

%Engine Thrusts
F_f1 = U(1); % Forward 1
F_f2 = U(2); % Forward 2

F_r1 = U(3); % Rear 1
F_r2 = U(4); % Rear 2

% Assuming Thrust is aligned with Body Axis
FTf1_b = [F_f1*cos(u5+x8) ; F_f1*sin(x7) ;-F_f1*sin(u5+x8) - F_f1*cos(x7)];
FTf2_b = [F_f2*cos(u5+x8) ; F_f2*sin(x7) ;-F_f2*sin(u5+x8)- F_f2*cos(x7)];

FTr1_b = [F_r1*cos(u6+x8) ; F_r1*sin(x7) ;-F_r1*sin(u6+x8)- F_r1*cos(x7)];
FTr2_b = [F_r2*cos(u6+x8) ; F_r2*sin(x7) ;-F_r2*sin(u6+x8)- F_r2*cos(x7)];

% FTf1_b = [0 ; 0 ; -F_f1*sin(u5)];
% FTf2_b = [0 ;0  ; -F_f2*sin(u5)];
% 
% FTr1_b = [0 ; 0 ; -F_r1*sin(u6)];
% FTr2_b = [0 ; 0 ; -F_r2*sin(u6)];
 
FT_b = FTf1_b + FTf2_b + FTr1_b +FTr2_b ;

%FT_e = R_eb*FT_b;
% Engine Moment Arms
marm_F1 = [ XmF1 - Xcg ; 
             YmF1-Ycg; 
             ZmF1 - Zcg];
         
marm_F2 = [ XmF2 - Xcg ; 
             YmF2-Ycg; 
             ZmF2 - Zcg];


marm_R1 = [ XmR1 - Xcg ; 
             YmR1-Ycg; 
             ZmR1 - Zcg];
         
marm_R2 = [ XmR2 - Xcg ; 
             YmR2-Ycg; 
             ZmR2 - Zcg];

MEcg_F1  = cross(marm_F1,FTf1_b);  
MEcg_F2  = cross(marm_F2,FTf2_b);


MEcg_R1  = cross(marm_R1,FTr1_b);
MEcg_R2  = cross(marm_R2,FTr2_b);


MEcg = MEcg_F1 + MEcg_F2 + MEcg_R1 + MEcg_R2;


%% Gravity Effects
g_b = [ g*sin(x8);
        g*cos(x8)*sin(x7);
        g*cos(x8)*cos(x7)];
    
 FG_b = x10*g_b;
 
 
 %% State Derivatives
 
 %Inertia Matrix 
 I = [873 1140 0;
       1140 907 0 ;
     0 0 1680] ;
 
 %Inverse of Inertia Matrix
 
inv_I = [0.0017861749663738284997	0.0022450269698634669126	0;
	0.0022450269698634669126	-0.0017192180216586022937	0;
	0	0	0.00059523809523809523809];
 
% inv_I = [0.026222217560494655912	0	0;
% 	       0 0.062677455546014653989	0;
% 	      0	0 0.020257226259037255064];
      
 % Total Body  Force
 F_b = FA_s + FG_b + FT_b;
 x13dot = (1/x10)*F_b - cross(om_b,v_b);
 
 % All Moments
 Mcg_b = MEcg; %+ MAcg;
 x46dot = inv_I*(Mcg_b - cross(om_b,I*om_b));
 
 % Euler Angles
 H_phi = [ 1 sin(x7)*tan(x8) cos(x7)*tan(x8);
           0 cos(x7) -sin(x7);
           0 sin(x7)/cos(x8) cos(x7)/cos(x8)];  % Transforming Angular Vel to Euler Rates
       
 x79dot = H_phi*om_b;
 x10dot = 0;
 % Placing in a Single MX
 
 XDOT = [x13dot;x46dot;x79dot;x10dot];
