function [dX] = BeamDynamics( X, T )

% Ball and Beam:  Sp20 Version
% m = 2 kg
% J = 0.5 kg m^2

r = X(1);
q = X(2);
dr = X(3);
dq = X(4);
g = 9.8;

M = [2.8, 0; 0, 0.5 + 2*r*r];
B1 = 2*r*dq*dq - 2*g*sin(q);
B2 = T - 4*r*dr*dq - 2*g*r*cos(q);

ddX = inv(M)*[B1; B2];
dX = [dr; dq; ddX];

end
