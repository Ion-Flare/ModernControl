function [] = CartDisplay(X, Ref)
 figure(2)
x = X(1);
q = X(2);
% cart
xc = [-0.2,0.2,0.2,-0.2,-0.2] + x;
yc = [0,0,0.2,0.2,0];
xm = x + sin(q);
ym = 0 + cos(q);
% ball
q = [0:0.1:1]' * 2*pi;
xb = 0.05*cos(q) + xm;
yb = 0.05*sin(q) + ym;
clf;
plot([-3,3],[-1.5,1.5],'.');
hold on
plot([-3,3],[0,0],'b-');
plot(xc, yc,'r-');
plot([x,xm],[0,ym]+0.2,'r-');
plot(xb, 0.2+yb, 'r-');
plot([Ref, Ref],[-0.1,0.1],'b')
title('Cart Nonlinear Simulation');
pause(0.00001);
end
function [ dX ] = CartDynamics( X, F )
%cart dynamics  (Sp20 version)
% cart = 3kg
% ball = 2kg
% length = 1m
% X = [x, q, dx, dq]
x = X(1);
q = X(2);
dx = X(3);
dq = X(4);
g = 9.8;
M = [5, 2*cos(q); 2*cos(q), 2];
A = [2*dq*dq*sin(q); 2*g*sin(q)];
B = [1;0];
d2X = inv(M) * (A + B*F);
dX = [dx; dq; d2X];
end
