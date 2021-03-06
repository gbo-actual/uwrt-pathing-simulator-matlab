function [ xC,yC,thC ] = genMotionPrimative(delta,dt,tmax,v,L)
%GENMOTIONPRIMATIVE Summary of this function goes here
%   Generates a motion primative given an 
% xC is an array; same with yC and thC
% use the above fact to control the point generation

numSteps = floor(tmax/dt);

xCurve = zeros(numSteps,1);
yCurve = zeros(numSteps,1);
thCurve = zeros(numSteps,1);

x = 0;
y = 0;
th =0;


for(i=1:numSteps)
   
    %th = th + (v/L)*tan(delta)*dt;
  x = x+v*sin(th)*dt;
  y = y+v*cos(th)*dt;
  th = th + (v/L)*tan(delta)*dt;
  
  xCurve(i) = x;
  yCurve(i) = y;
  thCurve(i) = th;
    
end

xC = xCurve;
yC = yCurve;
thC = thCurve;

end

