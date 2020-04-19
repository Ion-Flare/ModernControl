% Creator: Ian Hudis 
% last updated: 4/10/20
% Some code borrowed from Professor Jacob Glower
%% problems 1 through 3  cart and pendulum -> LQG Control

% Problem 1 -> find Kx and Kz
%time domain
 t = 0:0.01:10;

%from hw 5
A = [0 0 1 0;
     0 0 0 1;
     0 -6.533 0 0;
     0 16.333 0 0]; 
B = [0; 0; 0.333; -0.333]; 
C = [1,0,0,0];
D = 0;

%2 percent settling time (4 seconds)
T= 4/4; %(YeS tHiS iS jUsT 1)
% no overshoot e 
DR= 1;
theta = acos(DR);

%rescale hw 5 data for Q and r
 A5 = [A, zeros(4,1) ; C, 0];
 B5u = [B ; 0];
 B5r = [0;0;0;0;-1];
C5 = [C,0];

q = [0, 0, 0, 0, 1]; %initial guess to slow


%figure out Q (optimization)
while(1)
Q=diag(q); % too slow 

K5 = lqr(A5, B5u, Q,1);
eigen = eig(A5-B5u*K5); %check if too fast or too slow
 

 G = ss(A5-B5u*K5,B5r, C5, 0);
 y = step(G,t);
 

 %corrects the speed of the system
 if y(400) <= 0.98
 q(5)=q(5)+1;
 else
     break;
 end
 
  %corrects overshoot/damping ratio
 if any(y > 1.0005) % overshoot detected
     q(1)=q(1)+1;
 end
 
 

  % display linear system
  figure(1)
  plot(t,y);
 xlabel('Seconds');
 title('Cart Linear System');
 grid on;
 pause(0.001);
end
 

%display stuff
Q = q %shows the Q
%displays kx and kz
 Kx=K5(1:4) 
 Kz=K5(5) 
%final eigen values
eigen




 % Problem 2 -> display linear system
  figure(1)
  plot(t,y);
 xlabel('Seconds');
 ylabel('Position');
 title('Cart Linear System');
 grid on;

 

 
 % Problem 3 -> cart and pendulum nonlinear simulation
 X = zeros(4,1);
dX = zeros(4,1);
Ref = 1;
dt = 0.01;
U = 0;
t = 0;
Z = 0;
y = [];
while(t < 10)
 Ref = sign(sin(0.1*t));
 U = -Kz*Z - Kx*X;
 dX = CartDynamics(X, U);
 dZ = X(1) - Ref;
 X = X + dX * dt;
 Z = Z + dZ * dt;
 t = t + dt;
 CartDisplay(X, Ref);
 y = [y; X(1)];
end

clf
t = [1:length(y)]' * 0.01;
figure(2)
plot(t, y);
 xlabel('Seconds');
 ylabel('Position');
 title('Cart Nonlinear Simulation');
grid on

