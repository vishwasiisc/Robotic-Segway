%main
%Runs segway simulation
% Vishvas and Andy, Dec. 28, 2023
% uses various funcions:
%  SegwayDerive.m   (Symbolic derivation)
%    creates  ABrhs function  
%    which is called by
%  segwayrhs     (EoM  using parameters p)
%    also calls   mycontroller.m    (control law for balance)

%50 osci in 35 sec

clc; clear all ; clf; close all; 

%%
%Set parameter values
R = 0.055;     d = 0.079;      g = 9.81;
ms = 0.643;      mw = 0.2;       Is = 0.005;    Iw = 0.000;


%d=0.075
%Is=0.00525 0.0013
%0.0058
%0.0067
%ms=0.643
%Iw=0.0003;


%Is=0.00125 10to60 reading

p.R =  R; p.d =  d; p.g =  g;
p.ms = ms; p.mw = mw; p.Is = Is; p.Iw = Iw;

%control parameters
p.Kth=14; p.Dth=0.6; p.Ith=100;%100;  %[10,2]
p.Kph=0.0; p.Dph=0.0;  %[0.5,0.1]






%Set initial conditions
theta0    = deg2rad(5);   %1.9 for expriment  11 %1.7 1.6
phi0      = deg2rad(0);
thetadot0 = 0;
theta_I   = 0;
phidot0   = deg2rad(0);
z0 = [theta0 phi0 thetadot0 phidot0 theta_I]';

KE0=0; PE0=ms*g*d*cos(theta0);

E0=KE0+PE0;
H0=0;

%%
%Duration
start = 0;  stop = 3; t = linspace(start,stop,10000);


%dt=1e-6;
%tspan=linspace(start,stop,stop/dt);

%Make anonymous function
therhs = @(t,z)  segwayrhs(t,z,p);

%Solve ODEs
small   = 1e-13;
options = odeset('AbsTol', small, 'RelTol', small); %AbsTol
soln    = ode45(therhs,t, z0,options);


%%
%Plot solution
tarray = linspace(start, stop, 100000);

zarray = deval(soln,tarray);

thetaarray = zarray(1,:);
phiarray   = zarray(2,:);
thetadotarray=zarray(3,:);
phidotarray=zarray(4,:);
theta_I_array=zarray(5,:);

%%
%energy
KEw=0.5*Iw*(phidotarray.^2)+0.5*mw*R*R*(phidotarray.^2);

KEs=0.5*Is*thetadotarray.*thetadotarray...
    +0.5*ms*(((d*(thetadotarray.*cos(thetaarray))+R*phidotarray).^2)...
    +((d*thetadotarray.*sin(thetaarray)).^2));




PEs=ms*g*(d*cos(thetaarray));

E=KEw+KEs+PEs;
dE=E-E0*ones(1,length(E));



%%
%plotting
figure(1)
subplot(2,1,1)
plot(tarray, rad2deg(thetaarray),'b',LineWidth=1.4)%,tarray,rad2deg(phiarray),'r',LineWidth=1.5);


%{
hold on
%y=csvread('reading10to60.csv');
y=csvread('reading2.csv');
t=linspace(0,0.58,203);
%t=y(:,1);
theta=y(:,2);
%plot(t,theta,'r.')
%}
title('Lean angle vs time')
ylabel('$\theta$ $in$ $degrees$')
legend('\theta')
xlabel('$t(s)$')
movegui("northwest")
shg

%
%wheel angle
figure(1)
subplot(2,1,2)
plot(tarray,rad2deg(phiarray),'r',LineWidth=1.4)
title('Wheel angle','FontWeight','bold')
ylabel('$\phi$ $in$ $degrees$','FontWeight','bold')
legend('\phi')
xlabel('$t$','FontWeight','bold')
%movegui("southwest")
shg
%}

%
%theta integrator
%{
figure(2)
%subplot(3,1,3)
plot(tarray,rad2deg(theta_I_array),'r')
title('Integrator term')
ylabel('$\theta_I$')
legend('\theta_I')
xlabel('$t$')
movegui("northeast")
shg
%}


%energy

figure(2)
plot(tarray,dE,'r')
title('$E_0-E$')
ylabel('$\Delta E$')
%legend('\theta_I')
xlabel('$t$')
movegui("southwest")
shg
%}



noise_g=deg2rad(5/(4*sqrt(2)))*(sin(100*tarray)+cos(100*tarray) ...
                            + sin(200*tarray)+cos(200*tarray) ...
                            + sin(300*tarray)+cos(300*tarray) ...
                            + sin(400*tarray)+cos(400*tarray));


%{
figure(5)
plot(tarray,rad2deg(noise_g));
title('Noise vs time')
ylabel('$Noise$ $in$ $degrees$');
xlabel('$t$')
movegui('southeast')
%}

%animate(soln,stop,0.4);

