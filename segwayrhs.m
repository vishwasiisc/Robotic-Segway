function zdot = segwayrhs(t,z,p)
%unpack z
theta = z(1); phi = z(2); thetadot= z(3); phidot= z(4);



%{
noise=deg2rad(10/(4*sqrt(2)))*(sin(100*t)+cos(100*t) ...
                            + sin(200*t)+cos(200*t) ...
                            + sin(300*t)+cos(300*t) ...
                            + sin(400*t)+cos(400*t));
%}
noise=0;



%unpack p
Is = p.Is;
Iw = p.Iw;
R  = p.R;
d  = p.d;
g  = p.g;
ms = p.ms;
mw = p.mw;
p.noise=noise;

%Control code is all of the interesting stuff
%Mm = mycontroller(z,p);
Mm = motor_controller(t,z,p);

[A,b] = Abrhs(Is,Iw,Mm,R,d,g,ms,mw,theta,thetadot);

zddots = A\b;

thetaddot =zddots(1);
phiddot   =zddots(2);


zdot = [thetadot phidot thetaddot phiddot theta+noise]';
%zdot1= [thetadot phidot thetaddot+deg2rad(0.001) phiddot theta+noise]';
end

