%SegwayDerive.m
%Derive EoM for Segway
%Andy and Vishvas on Dec 28, 2023


syms mw ms Iw Is R d g  Mm   real
syms theta thetadot thetaddot phi phidot phiddot    real


i  = [ 1 0 0]'; j = [0 1 0]';   k = [0 0 1]';

er     = cos(theta) * j - sin(theta)*i;
etheta = cross(k, er);

rG1relC  = R*j;
rG2relG1 = d*er;
rG2relC  = rG1relC + rG2relG1;

aG1      = -phiddot*R*i;   %due to pure rolling condition this models the friction
aG2relG1 =  thetaddot*d*etheta - thetadot^2 * d * er;
aG2      =  aG1 + aG2relG1;


MtotrelCsys    =   cross(rG2relC, -ms*g*j) ...  %Only gravity torques show up
                 + cross(rG1relC, -mw*g*j);
                  

MtotrelG1stick =   cross(rG2relG1, -ms*g*j) ...
                 + Mm*k;


HdotrelCsys    =   cross(rG1relC,aG1)*mw +  Iw*phiddot  *k ... % wheel
                 + cross(rG2relC,aG2)*ms +  Is*thetaddot*k;   % stick
          
HdotrelG1stick = cross(rG2relG1,aG2)*ms + Is*thetaddot*k; 

eqn1 = MtotrelCsys    - HdotrelCsys;
eqn2 = MtotrelG1stick - HdotrelG1stick;



eqn1 = eqn1(3);    
eqn2 = eqn2(3);


eqns = [eqn1 eqn2];    %right side of the eqns are zero //Mtotal = Hdot

vars = [thetaddot, phiddot];

[A,b] = equationsToMatrix(eqns,vars);

A = simplify(A);
b = simplify(b);

%}

%matlabFunction(A,b,'File','Abrhs2','Optimize',true);
