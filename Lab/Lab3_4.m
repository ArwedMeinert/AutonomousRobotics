clc
clear all
myapp = SPRob;
x = 20;
y = 20;
theta = 0;
speed = 3;
rotationspeed = -0.5;
cov_robot=0.0001;
cov_anchors=0.01;
cov = diag([repmat(cov_robot,1,3),repmat(cov_anchors,1,15)]);
myanchorsunit = [-1,-1,-1]; %x,y,number
mu = [x,y,theta,myanchorsunit,myanchorsunit,myanchorsunit,myanchorsunit,myanchorsunit]';

dt = 0.1;
myapp.dt=dt;
while(true)
%speed = 3.0;
myapp.setspeed(speed);
myapp.setrotationspeed(rotationspeed);
signal = myapp.getanchorssignal();
anchors=[0,0;0,20;20,0;20,30;10,10];
[mu, cov] = EKFslam(mu,cov,speed,rotationspeed,signal,myapp.dt);%Develop this function
%[mu, cov] = MCL(mu,cov,speed,rotationspeed,signal,myapp.dt);%Develop this function
%[mu, cov] = MCLlocalization(mu,cov,speed,rotationspeed,signal,anchors,myapp.dt);%Develop this function
myapp.updateyourAGV(mu(1,1),mu(2,1),mu(3,1));
%myanchors=anchors;
myanchors=[mu(4),mu(5);mu(7),mu(8);mu(10),mu(11);mu(13),mu(14);mu(16),mu(17)];

myapp.updateyouranchors(myanchors);

myapp.update;
pause(dt);
end