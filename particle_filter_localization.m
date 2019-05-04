function [] = particle_filter_localization()

close all;
clear all;
%parameters below are the testing mode
test_mode = 1;
if_using_prepared_data = 1;
% -------------------------------------------------------------------------
%test_mode = 1: normal mode,
    %particle number = 50, the program will generate the
    % localization animation in the whole process
% -------------------------------------------------------------------------
% for that this mode below will take a lot of time, so We set the parameter
    % if_using_prepared_data to shrink the run time by plot the prepared
    % data
% -------------------------------------------------------------------------
% test_mode = 2: test runtime mode.
    %particle number change from 50 to 500,
    % it will generate the runtime-particle number image
% -------------------------------------------------------------------------
% test_mode = 3: test average error mode.
    %particle number change from 50 to 500,
    % it will generate the average error-particle number image
% -------------------------------------------------------------------------
% test_mode = 4: error plot mode.
    %particle number change from 50 to 500,
    % when NP = 50 || 200 || 400, it will generate the whole
    % error-localization process time(from 0 to 60) and the program will pause.
    % input any key in the command window and the process will go on
    % Also, it will generate the average error-particle number image in the end.

%PARTICLE_FILTER_LOCALIZATION Summary of this function goes here
%   Detailed explanation goes here

% -------------------------------------------------------------------------
% TASK for particle filter localization
% for robotic class in 2018 of ZJU

% Preparartion:
% 1. you need to know how to code and debug in matlab
% 2. understand the theory of Monte Carlo

% Then complete the code by YOURSELF!
% -------------------------------------------------------------------------


disp('Particle Filter program start!!')

%% initialization
global dt;
dt = 0.1; % second
% Simulation parameter
global Qsigma        %(used for v and w)
Qsigma=diag([0.1 toRadian(5)]).^2;
global Rsigma        %
Rsigma=diag([0.1]).^2;
runtime = zeros(1,50);
avgerror = zeros(1,50);
it = 1;
if test_mode == 2 && if_using_prepared_data == 0
    disp('testing mode 2 without prepared data!!')
end
if test_mode == 3 && if_using_prepared_data == 0
    disp('testing mode 2 without prepared data!!')
end
if test_mode == 4
    disp('testing mode 4 without prepared data!!')
end
for NP = 10:10:500
    %test_mode 1
    if test_mode == 1
        disp('testing mode 1')
        NP = 50;
    end
    %test_mode 2 using prepared data
    if test_mode == 2 && if_using_prepared_data == 1
        disp('testing mode 2 with prepared data done')
        load runtime runtime
        NP = linspace(10,500,50);
        hold on
        plot(NP,runtime,'-b','linewidth', 4);
        title('run time chaning with particle numbers', 'fontsize', 12, 'fontname', 'times');
        xlabel('NP ', 'fontsize', 12, 'fontname', 'times');
        ylabel('localization time(s)', 'fontsize', 12, 'fontname', 'times');
        disp('test mode 2 with data prepared done!')
        break;
    end
    if test_mode == 3 && if_using_prepared_data == 1
        clf;
        NP = linspace(10,500,50);
        load avgerror avgerror
        hold on
        disp('test mode 3 with data prepared done!');
        %save avgerror avgerror
        plot(NP,avgerror,'-b','linewidth',4);
        title('average error of localization changing with particle numbers', 'fontsize', 12, 'fontname', 'times');
        xlabel('NP ', 'fontsize', 12, 'fontname', 'times');
        ylabel('average error(m)', 'fontsize', 12, 'fontname', 'times');
        break;
    end
    t1=clock;
    time = 0;
    endTime = 60; % second
    nSteps = ceil((endTime - time)/dt);
    
    localizer.time = [];
    localizer.xEst = [];
    localizer.xGnd = [];
    localizer.xOdom = [];
    localizer.z = [];
    localizer.PEst=[];
    localizer.u=[];
    
    % Estimated State [x y yaw]'
    xEst=[0 0 0]';
    % GroundTruth State
    xGnd = xEst;
    % Odometry-only = Dead Reckoning
    xOdom = xGnd;
    
    % Covariance Matrix for predict
    Q=diag([0.1 0.1 toRadian(3)]).^2;
    % Covariance Matrix for observation
    R=diag([1]).^2;% range:meter
    % landmark position
    landMarks=[10 0; 10 10; 0 15; -5 20];
    % longest observation confined
    MAX_RANGE=20;
    % Used in Resampling Step, a threshold
    NTh=NP/2.0;
    % particles produced
    px=repmat(xEst,1,NP);  %3*NP
    % weights of particles produced
    pw=zeros(1,NP)+1/NP;   %inital pw(1/NP)
    
    %% Main Loop
    for i=1 : nSteps
        
        time = time + dt;
        u=doControl(time);
        
        % do observation
        % z = [d1,landmark1x,landmark1y;
        %        d2,landmark1x,landmark1y; ...]
        [z,xGnd,xOdom,u]=doObservation(xGnd, xOdom, u, landMarks, MAX_RANGE);
        
        for ip=1:NP
            
            % process every particle
            x=px(:,ip);
            w=pw(ip);
            
            % do motion model and random sampling
            x=doMotion(x, u)+sqrt(Q)*randn(3,1);
            
            % calculate importance weight
            for iz=1:length(z(:,1))
                %gets the distance from eval and landmarks
                pz=norm(x(1:2)'-z(iz,2:3));
                dz=pz-z(iz,1);
                w=w*Gaussian(dz,0,sqrt(R));
            end
            px(:,ip)=x;
            pw(ip)=w;
            
        end
        
        pw=Normalization(pw,NP);
        [px,pw]=ResamplingStep(px,pw,NTh,NP);
        xEst=px*pw';
        
        % Simulation Result
        localizer.time=[localizer.time; time];
        localizer.xGnd=[localizer.xGnd; xGnd'];
        localizer.xOdom=[localizer.xOdom; xOdom'];
        localizer.xEst=[localizer.xEst;xEst'];
        localizer.u=[localizer.u; u'];
        % if mode 1, get animation
        if test_mode == 1
            %Animation (remove some flames)
            if rem(i,10)==0
                hold off;
                arrow=0.5;
                for ip=1:NP
                    quiver(px(1,ip),px(2,ip),arrow*cos(px(3,ip)),arrow*sin(px(3,ip)),'ok');hold on;
                end
                plot(localizer.xGnd(:,1),localizer.xGnd(:,2),'.b');hold on;
                plot(landMarks(:,1),landMarks(:,2),'pk','MarkerSize',10);hold on;
                if~isempty(z)
                    for iz=1:length(z(:,1))
                        ray=[xGnd(1:2)';z(iz,2:3)];
                        plot(ray(:,1),ray(:,2),'-r');hold on;
                    end
                end
                plot(localizer.xOdom(:,1),localizer.xOdom(:,2),'.k');hold on;
                plot(localizer.xEst(:,1),localizer.xEst(:,2),'.r');hold on;
                axis equal;
                grid on;
                drawnow;
            end
        end
        
    end
    
    % draw the final results of localizer, compared to odometry & ground truth
    if test_mode == 1
        drawResults(localizer);
        break;
    end
    %record the time of localization when particle number is NP
    if test_mode == 2
        t2 = clock;
        t = etime(t2 , t1);
        runtime(it) = t;
    end
    NPstr = num2str(NP);
    str1 = ['current particle number: ', NPstr];
    disp(str1);
    %record the avgerror of localization when particle num is NP
    if test_mode == 3 || test_mode == 4
        avgerror(it) = average_error(localizer.xGnd,localizer.xEst);
    end
    % plot error changing with time when particle num is 50 200 400
    if test_mode == 4
        if NP == 50 || NP == 200 || NP == 400
            drawErrors(localizer,NP);
            disp('pausing, input any key to contiune!!')
            pause();
        end
    end
    it = it + 1;
end


%% plot run time chaning with particle numbers
%
%
if test_mode == 2 && if_using_prepared_data == 0
    save runtime runtime
    NP = linspace(10,500,50);
    hold on
    plot(NP,runtime,'-b','linewidth', 4);
    title('run time chaning with particle numbers', 'fontsize', 12, 'fontname', 'times');
    xlabel('NP ', 'fontsize', 12, 'fontname', 'times');
    ylabel('localization time(s)', 'fontsize', 12, 'fontname', 'times');
end
%% plot average error of localization changing with particle numbers
if (test_mode == 3 && if_using_prepared_data == 0) || test_mode == 4
    clf;
    NP = linspace(10,500,50);
    hold on
    %save avgerror avgerror
    plot(NP,avgerror,'-b','linewidth',4);
    title('average error of localization changing with particle numbers', 'fontsize', 12, 'fontname', 'times');
    xlabel('NP ', 'fontsize', 12, 'fontname', 'times');
    ylabel('average error(m)', 'fontsize', 12, 'fontname', 'times');
end

end







%% Other functions

% degree to radian
function radian = toRadian(degree)
radian = degree/180*pi;
end

function []=drawResults(localizer)
%Plot Result

figure(1);
hold off;
x=[ localizer.xGnd(:,1:2) localizer.xEst(:,1:2)];
set(gca, 'fontsize', 12, 'fontname', 'times');
plot(x(:,1), x(:,2),'-.b','linewidth', 4); hold on;
plot(x(:,3), x(:,4),'r','linewidth', 2); hold on;
plot(localizer.xOdom(:,1), localizer.xOdom(:,2),'--k','linewidth', 4); hold on;

title('Localization Result', 'fontsize', 12, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 12, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 12, 'fontname', 'times');
legend('Ground Truth','Particle Filter','Odometry Only');
grid on;
axis equal;

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


%%  you need to complete

% do Observation model
function [z, xGnd, xOdom, u] = doObservation(xGnd, xOdom, u, landMarks, MAX_RANGE)
global Qsigma;
global Rsigma;

% Gnd Truth and Odometry
xGnd=doMotion(xGnd, u);% Ground Truth
u=u+sqrt(Qsigma)*randn(2,1); % add noise randomly
xOdom=doMotion(xOdom, u); % odometry only

%Simulate Observation
z=[];
for iz=1:length(landMarks(:,1))
    d = norm(xGnd(1:2)' - landMarks(iz,1:2));
    %d = sqrt((x(1) - landMarks(iz,1))^2 + (x(2) - landMarks(iz,2)^2));
    if d<MAX_RANGE
        z = [z;[d+sqrt(Rsigma)*randn(1,1) landMarks(iz,:)]];   % add observation noise randomly
    end
end
end


% do Motion Model
function [ x ] = doMotion( x, u)
global dt;

Delta = [ dt*cos(x(3)) 0;
    dt*sin(x(3)) 0;
    0 dt];
% this is the basic move model ode
x = x + Delta * u;

end

% Gauss function
function g = Gaussian(x,u,sigma)
g = normpdf(x,u,sigma);
end

% Normalization
function pw=Normalization(pw,NP)
pw = pw +1.e-300;      %avoid shink to zero
pw = pw / sum(pw); % normalize
end

% Resampling
function [px,pw]=ResamplingStep(px,pw,NTh,NP)
Neff=1/sum(pw.*pw);
if Neff>NTh %预先设定阈值
    
    w_cumu=pw;
    pxnew = px;
    for i = 2 : NP
        w_cumu(i) = w_cumu(i - 1) + w_cumu(i);
    end
    w_cumu(NP+1) = 1.001;
    for i = 1 : NP
        w0 = rand;
        if w0 <= w_cumu(1)
            pxnew(:,i) = px(:,1);
            continue;
        end
        for j = 2 : NP
            if w0 <= w_cumu(j) && w0 > w_cumu(j-1)
                pxnew(:,i) = px(:,j);
                break;
            end
        end
        pw(i) = 1/NP;
    end
    px = pxnew;
end
end

function avgerror = average_error(xGnd,xEst)
dx = xGnd - xEst;
x = dx(:,1:2);
delta = zeros(1,length(x(:,1)));
for i = 1 : length(x(:,1))
    delta(i) = sqrt(x(i,1)^2 + x(i,2)^2);
end
avgerror = mean(delta);

end

function []=drawErrors(localizer,NP)
%Plot Errors
clf;
NPstr = num2str(NP);
dx = localizer.xGnd - localizer.xEst;
x = dx(:,1:2);
delta = zeros(1,length(x(:,1)));
for i = 1 : length(x(:,1))
    delta(i) = sqrt(x(i,1)^2 + x(i,2)^2);
end
figure(1);

plot(localizer.time, delta,'-b','linewidth', 4); hold on;
titlestr = ['Localization Rrror when particle number is ' ,NPstr ];
title(titlestr, 'fontsize', 12, 'fontname', 'times');
xlabel('time (s)', 'fontsize', 12, 'fontname', 'times');
ylabel('error (m)', 'fontsize', 12, 'fontname', 'times');
grid on;
axis equal;
ylim([0 1])
end