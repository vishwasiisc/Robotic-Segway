function Mm = mycontroller(z,p);
%unpack z
theta = z(1); phi = z(2); thetadot= z(3); phidot= z(4);

%unpack p
Is = p.Is; Iw = p.Iw;
R  = p.R;  d  = p.d; g  = p.g;
ms = p.ms; mw = p.mw;

Kth=p.Kth; Dth=p.Dth; Kph=p.Kph; Dph=p.Dph;

Mm=0;
%Mm = -Kth*theta - Dth* thetadot - Kph*phi  - Dph* phidot;
end



