function [y] = NAV(a , e_sq , lat)

M = (a*(1 - e_sq))/(1 - e_sq*(sin(lat))^2)^(1.5);
N = a/sqrt(1 - e_sq*(sin(lat))^2) ;

y = [M ; N] ;
