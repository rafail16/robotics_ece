%% *** Robot (kinematic) model parameters *** 
clear all; 
close all;
clc;
l0 = 6.0;
l(1) = 9.0;  %% in cm 
l(2) = 6.0;  
l(3) = 5.0;
%% *** sampling period *** 
%% *** for the robot motion, kinematic simulation: 
dt = 0.05; %dt = 0.05; i.e. 1 msec)   

%% *** Create (or load from file) reference signals *** 
%% *** DESIRED MOTION PROFILE - TASK SPACE *** 
Tf=10.0; 	% 10sec duration of motion 
t=0:dt:Tf;  

%xd0,td0,yd1: initial/final end-point position --> desired task-space trajectory  


for i=0:3 %edw vazoume poses fores theloyme na ginei h talantwsh
%A is always [7.00;6.00;16.00] alla gia na mh grafoume 2 fores kwdika sthn
%epistrofh thewroyme anapoda
if (mod(i,2)==0)
    A=[7.00;6.00;16.00];
    B=[3.00;-1.00;16.00];
else
    B=[7.00;6.00;16.00];
    A=[3.00;-1.00;16.00];
end

%% Time Parameters
dt = 0.05;
Tf = 10.0;
t = 0:dt:Tf;
kmax=length(t);

a2 = (3/Tf^2)*(B(1) - A(1));
a3 = -(2/Tf^3)*(B(1) - A(1));

b2 = (3/Tf^2)*(B(2) - A(2));
b3 = -(2/Tf^3)*(B(2) - A(2));

c2 = (3/Tf^2)*(B(3) - A(3));
c3 = -(2/Tf^3)*(B(3) - A(3));

% Example of desired trajectory : linear segment (x0,y0)-->(x1,y1); Time duration: Tf; 
disp('Initialising Desired Task-Space Trajectory (Motion Profile) ...'); %% 
disp(' ');

xd(1) = A(1); 
yd(1) = A(2); 
zd(1) = A(3); 
for k=2:length(t)    
   xd(k) = A(1) + a2*t(k)^2 + a3*t(k)^3;    
   yd(k) = A(2) + b2*t(k)^2 + b3*t(k)^3;
   zd(k) = A(3) + c2*t(k)^2 + c3*t(k)^3;
end  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% ****** KINEMATIC SIMULATION - Main loop ****** 
disp('Kinematic Simulation ...'); %% 
disp(' '); %% 

%% ***** INVESRE KINEMATICS  -->  DESIRED MOTION - JOINT SPACE ***** 
%% compute the reference joint-motion vectors: 
%% {qd(k,i), i=1,...,n (num of degrees of freedom), with k=1,..., kmax,} 
%% and reference joint (angular) velocities {qd_1(k,i)} 
syms pe_x pe_y pe_z q1 q2 q3
P=[pe_x;pe_y;pe_z];
A01=[cos(q1) 0 -sin(q1) 0; 0 1 0 0; sin(q1) 0 cos(q1) l0; 0 0 0 1];
A0S=A01*[1 0 0 0;0 1 0 0; 0 0 1 l(1); 0 0 0 1];
Pos=A0S(1:3,4);
R0S=A0S(1:3,1:3);
Pn=-R0S'*Pos+R0S'*P;
%upologismos q2, q3
q3 =  acos((Pn(1)^2 + Pn(2)^2 -l(2)^2 -l(3)^2)/(2*l(2)*l(3))); 
q2 = atan2(Pn(2),Pn(1)) - asin(l(3)*sin(q3)/sqrt(Pn(1)^2 + Pn(2)^2));

%Calculating q1
temp = -pe_x + sqrt(pe_x^2+(-l0 + pe_z)^2 - l(1)^2);
temp = (temp/(l(1)+pe_z-l0));
q1 = atan2(2*temp,1-temp^2);


for m=1:kmax
    pe_x=xd(m);
    pe_y=yd(m);
    pe_z=zd(m);
    temp = -pe_x + sqrt(pe_x^2+(-l0 + pe_z)^2 - l(1)^2);
    temp =(temp/(l(1)+pe_z-l0));
    %calculate firstly q1
    qd1(m) = atan2(2*temp,1-temp^2);  
    %calculate new coordinates base on system 2 located on the joint 2
    pnx=-l(1)*sin(qd1(m));
    pny=0;
    pnewz=l0+l(1)*cos(qd1(m)); 
    qd3(m) =  acos(((xd(m)-pnx)^2 + (yd(m)-pny)^2 -l(2)^2 -l(3)^2)/(2*l(2)*l(3))); 
    qd2(m) = atan2(yd(m)-pny,xd(m)-pnx) - asin(l(3)*sin(qd3(m))/sqrt((xd(m)-pnx)^2 +(yd(m)- pny)^2));
end
   
%% ***** FORWARD KINEMATICS  JOINT MOTION -->  CARTESIAN POSITIONS ***** 
dtk=10;
for tk=1:kmax  
xd1(tk) = 0;  
yd1(tk) = 0; 
zd1(tk) = l0;

xd2(tk) =-l(1).*sin(qd1(tk));   
yd2(tk) =0;  
zd2(tk) =l(1).*cos(qd1(tk))+l0;

xd3(tk) =l(2).*cos(qd1(tk)).*cos(qd2(tk)) -l(1).*sin(qd1(tk));   
yd3(tk) = l(2).*sin(qd2(tk));    
zd3(tk) =l(2).*sin(qd1(tk)).*cos(qd2(tk))+l(1).*cos(qd1(tk))+l0;

xde(tk) = l(3)*cos(qd1(tk)).*cos(qd2(tk)+qd3(tk))+l(2)*cos(qd1(tk)).*cos(qd2(tk))-l(1)*sin(qd1(tk));
yde(tk) = l(3)*sin(qd2(tk)+qd3(tk))+l(2)*sin(qd2(tk)); 
zde(tk) = l(3)*sin(qd1(tk)).*cos(qd2(tk)+qd3(tk))+l(2)*sin(qd1(tk)).*cos(qd2(tk))+l(1)*cos(qd1(tk))+l0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
end

%% *** SAVE and PLOT output data *** %%** use functions plot(...)  
save; %% --> save data to 'matlab.mat' file   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

fig1 = figure; 
subplot(2,3,1); 
plot(t,xde(:)); 
ylabel('Pe,x (cm)'); 
xlabel('time (sec)');  

subplot(2,3,2); 
plot(t,yde(:)); 
ylabel('Pe,y (cm)'); 
xlabel('time (sec)');
title('Θέση σημείου δράσης');

subplot(2,3,3); 
plot(t,zd(:)); 
ylabel('Pe,z (cm)'); 
xlabel('time (sec)');

%calculate speed of Pe
Pe=[xde(:) yde(:) zde(:)];
fig2 = figure;  
Ve = zeros(kmax,3);
for q=2:kmax
    Ve(q,:) = Pe(q,:) - Pe(q-1,:);
end
subplot(2,3,1); 
plot(t,Ve(:,1)*(length(t)-1)*180/(Tf*pi)); 
ylabel('Vx (cm/sec)'); 
xlabel('time t (sec)');  

subplot(2,3,2); 
plot(t,Ve(:,2)*(length(t)-1)*180/(Tf*pi));  
ylabel('Vy (cm/sec)'); 
xlabel('time t (sec)');   
title('Γραμμική ταχύτητα σημείου δράσης');

subplot(2,3,3); 
plot(t,Ve(:,3)*(length(t)-1)*180/(Tf*pi)); 
ylabel('Vz (cm/sec)'); 
xlabel('time t (sec)');  



fig3=figure;
subplot(2,3,1); 
plot(t,qd1(:)*180/pi); 
ylabel('q1 (deg)'); 
xlabel('time t (sec)');  

subplot(2,3,2); 
plot(t,qd2(:)*180/pi); 
ylabel('q2 (deg)'); 
xlabel('time t (sec)');   
title('Γωνίες αρθρώσεων');

subplot(2,3,3); 
plot(t,qd3(:)*180/pi); 
ylabel('q3 (deg)'); 
xlabel('time t (sec)'); 



%calculate speed of robot joints
qd=[qd1(:) qd2(:) qd3(:)];
fig4 = figure;  
subplot(1,3,1);
w = zeros(kmax,3);
for q=2:kmax
    w(q,:) = qd(q,:) - qd(q-1,:);
end
subplot(2,3,1); 
plot(t,w(:,1)*(length(t)-1)*180/(Tf*pi)); 
ylabel('ω1 (deg/sec)'); 
xlabel('time (sec)');  

subplot(2,3,2); 
plot(t,w(:,2)*(length(t)-1)*180/(Tf*pi));  
ylabel('ω2 (deg/sec)'); 
xlabel('time (sec)');   
title('Γωνιακή ταχύτητα αρθρώσεων');

subplot(2,3,3); 
plot(t,w(:,3)*(length(t)-1)*180/(Tf*pi)); 
ylabel('ω3 (deg/sec)'); 
xlabel('time (sec)');


%%*** stick diagram --> animate robot motion ... (**optional**) 
%% within a for (or while) loop, use periodic plot(...) functions to draw the geometry (current pos)  
%% of the robot, and thus animate its motion ...  

fig2 = figure; 
axis([-10 10 -10 10 0 20]) %%set xyz plot axes (caution: square axes, i.e. dx=dy) 
axis on 
hold on 
xlabel('x (cm)'); 
ylabel('y (cm)'); 
zlabel ('z (cm)');
%plot3(xd,yd,zd,'ys');
dtk=10; %% plot robot position every dtk samples, to animate its motion 
plot3([0],[0], [0] ,'o');
if (mod(i,2)==0)
    text(A(1), A(2), A(3), 'A');
    text(B(1), B(2), B(3), 'B');
else
    text(A(1), A(2), A(3), 'B');
    text(B(1), B(2), B(3), 'A');
end
dtk=20;

for tk=1:dtk:kmax  %%% a	
       pause(1);	%% pause motion to view successive robot configurations    
       plot3([0,xd1],[0,yd1], [0,zd1]);
       plot3([xd1],[yd1],[zd1],'o');  
       plot3([xd1(tk),xd2(tk)],[xd1(tk),yd2(tk)], [zd1(tk),zd2(tk)]);
       plot3([xd2(tk)],[yd2(tk)], [zd2(tk)],'o');  
       plot3([xd2(tk),xd3(tk)],[yd2(tk),yd3(tk)],[zd2(tk),zd3(tk)]);	
       plot3([xd3(tk)],[yd3(tk)],[zd3(tk)],'o');  
       plot3([xd3(tk),xde(tk)],[yd3(tk),yde(tk)],[zd3(tk),zde(tk)]);
       plot3([xde(tk)],[yde(tk)],[zde(tk)],'r*');   
end
 
end
