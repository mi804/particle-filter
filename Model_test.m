% time = 0;
% endTime = 60; % second
%  dt = 0.1;
% nSteps = ceil((endTime - time)/dt);
% grid on
% x =[0 0 0]';
% u = [2,pi/2]';
% hold on
% for i = 0 : nSteps
%     time = time + dt;
%     u=doControl(time);
%     Delta = [ dt*cos(x(3)) 0;
%         dt*sin(x(3)) 0;
%         0 dt];
%     x = x + Delta * u;
%     plot(x(1),x(2),'o')
% end

% z = [1 0]
% z1 = [z;[1 2]]

% Rsigma=diag([0.1]).^2
% Qsigma=diag([0.1 toRadian(5)]).^2

% x=linspace(-5,5,50); %生成负五到五之间的五十个数，行矢量
% y=normpdf(x,0,1);
% plot(x,y,'k');
NP = 60;
xEst=[0 0 0]';
px=repmat(xEst,1,NP);  %3*NP

function radian = toRadian(degree)
    radian = degree/180*pi;
end

function [ u ] = doControl( time )
%DOCONTROL Summary of this function goes here
%   Detailed explanation goes here

    %Calc Input Parameter
    T=10; % [sec]

    % [V yawrate]
    V=1.0; % [m/s]
    yawrate = 5; % [deg/s]

    u =[ V*(1-exp(-time/T)) toRadian(yawrate)*(1-exp(-time/T))]';


end
