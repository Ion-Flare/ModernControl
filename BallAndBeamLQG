
t = [0:0.01:10]'; 

%from hw 5
A=[0 0 1 0;
   0 0 0 1;
   0 -7 0 0;
   -7.84 0 0 0];

B = [ 0; 0; 0; 0.4];
C=[1,0,0,0];
 

%rescale information
A5 = [A, zeros(4,1) ; 1,0,0,0,0];
 B5u = [B ; 0];
B5r = [0*B;-1];
C5 = [C,0;0,C];
D5 = [0;0];

X0 = zeros(5,1);
R = 0*t+1;

q = [0, 0, 0, 0, 1]; %initial guess to slow

%figure out Q (optimization)
while(1)
Q=diag(q); % too slow 

K5 = lqr(A5, B5u, Q,1);
eigen = eig(A5-B5u*K5); %check if too fast or too slow

 G = ss(A5-B5u*K5,B5r, [1,0,0,0,0], 0);
 y = step(G,t); 


K5 = lqr(A5,B5u,Q,1);
yd = step3(A5-B5u*K5,B5r,C5,D5,t,X0,R);
 

 %corrects the speed of the system
 if y(400) <= 0.98
 q(5)=q(5)+1;
 else
     break;
 end
 
  %corrects overshoot/damping ratio
 if any(y > 1.0005) % overshoot detected
     q(3)=q(3)+1;
 end
 
 


  % display linear system
  figure(1)
  plot(t,y);
  
 xlabel('Seconds');
 title('Ball and Beam Linear System');
 grid on;
 pause(0.00001);
end

%display stuff
Q = q %shows the Q
%displays kx and kz
 Kx=K5(1:4) 
 Kz=K5(5) 
%final eigen values
eigen


 % Problem 5 -> display linear system
  figure(1)
  plot(t,y);
 xlabel('Seconds');
 ylabel('Ball Position');
 title('Ball and Beam Linear System');
 grid on;


 
 %Problem 6 -> nonlinear simulation
 X = [0, 0, 0, 0]';
dt = 0.01;
t = 0;
K5 = [-64.4969   86.5500  -33.4720   20.8026 -24.37];
DC = -[0,0,0,0,1]*inv(A5 - B5u*K5)*B5u; 
 Kr = 1/DC;

y = [];

while(t < 10)
 Ref = 1 * (sin(0.1*pi*t) > 0);
 U = 1.85*Kr*Ref - Kx*X;

 dX = BeamDynamics(X, U);
 dZ = X(1) - Ref;
 
 X = X + dX * dt;

 y = [y ; Ref, X(1)];
 
 t = t + dt;
 BeamDisplay(X, Ref);
end

t = [1:length(y)]' * dt;
figure(2)
plot(t,y(:,1),'r',t,y(:,2),'b');
xlabel('Time (seconds)');
ylabel('Ball Position');
grid on;
 
