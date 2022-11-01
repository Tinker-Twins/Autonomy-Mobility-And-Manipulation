
clc
close all 
clear all 


%initial angle calculation of joints 
l1=300;
l2=250;

%Given Info Joint3 intial conditions
A2x=1.366025;
A2y=1.15;
B2x=-.366025;
B2y=1.15;
C2x=.50;
C2y=-.35;

%Base Joint Coordinates 
Ax=-2.5980;
Ay=-1.5;
Bx=0;
By=3;
Cx=2.598;
Cy=-1.5;

daa2=sqrt((A2x-Ax)^2+(A2y-Ay)^2);
dbb2=sqrt((B2x-Bx)^2+(B2y-By)^2);
dcc2=sqrt((C2x-Cx)^2+(C2y-Cy)^2);

%assuming thetaA1=94deg WRT world frame Xaxis


theta_aa1=acosd((daa2^2+l1^2-l2^2)/(2*l1*daa2));
theta_bb1=acosd((dbb2^2+l1^2-l2^2)/(2*l1*dbb2));
theta_cc1=acosd((dcc2^2+l1^2-l2^2)/(2*l1*dcc2));

Axl=A2x-Ax;
Ayl=A2y-Ay;
