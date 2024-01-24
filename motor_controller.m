function torque=motor_controller(t,z,p)



%unpack z
theta = z(1); phi = z(2); thetadot= z(3); phidot= z(4); theta_I=z(5);

%unpack p
Is = p.Is; Iw = p.Iw;
R  = p.R;  d  = p.d; g  = p.g;
ms = p.ms; mw = p.mw;


%%
%controller

Kth=p.Kth; Dth=p.Dth; Kph=p.Kph; Dph=p.Dph; Ith=p.Ith;

noise=p.noise;

%{
noise=deg2rad(5/(4*sqrt(2)))*(sin(10*t)+cos(10*t) ...
                            + sin(20*t)+cos(20*t) ...
                            + sin(30*t)+cos(30*t) ...
                            + sin(40*t)+cos(40*t));

%}

%theta=theta+deg2rad(5);%+noise;
%dt=1e-3;
%pwm=0;

%{
if rem(t,dt)==0
    pwm = -Kth*(theta) - Dth* (thetadot) - Kph*phi  - Dph* phidot   % my controller
    %pwm=0;  % uncomment this line to turn off motor
end
%}

u = rad2deg(-Kth*(theta+noise) - Dth* (thetadot) - Kph*phi  - Dph* phidot-Ith*theta_I);   % my controller
    %u=0;  % uncomment this line to turn off motor




%default gains [20 0.1 1 0.1]

%%
%moter model;

 %gear ratio 21.5
omega=phidot;
R=2.7;
K=0.1569;   %  %calculated from datasheet %.4
b=0.002; %yet to be measured  k^2/R=0.0091


sg=sign(u);
pwm=norm(u);
pwm=sg*min(max(pwm,0),255); %bound pwm between (7 and 255) // it overcomes gearbox limit at pwm=7

%pwm=100*sin(1000*t);

volt=pwm*(12.6/255);
%volt=0;

%torque=u;
torque=((K/R)*volt-omega*(b+(K*K/R)));
%torque=0;
%{
figure(6)
hold on;
plot(t,torque,'r.')
movegui("southeast")
%}


end

