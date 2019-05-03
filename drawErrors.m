function []=drawErrors(localizer,NP)
%Plot Errors
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

end