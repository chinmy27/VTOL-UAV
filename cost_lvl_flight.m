function [F0] = cost_lvl_flight(Z)

X = Z(1:9);
U = Z(10:15);

xdot = AircraftModel(X,U);
theta = X(8);
Va = sqrt(X(1)^2 + X(2)^2 + X(3)^2);
alpha = atan2(X(3),X(1));
gam = theta - alpha ; % Level flight

Q = [xdot ;
    Va - 80 ;
    gam ;
    X(2);
    X(7);
    X(9)];

W = [ (max(0,-U(5)))^2;
      (max(0,-U(6)))^2 ];    

H1 = diag(ones(1,14));

for i = 1:14 
    for j =1:14
        i = j;
        H1(i,j) = 1;
    end
end
H2 = [3,0;0,3];
F0 = Q'*H1*Q + W'*H2*W;
    