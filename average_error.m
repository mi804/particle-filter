function avgerror = average_error(xGnd,xEst)
    dx = xGnd - xEst;
    x = dx(:,1:2);
    delta = zeros(1,length(x(:,1)));
    for i = 1 : length(x(:,1))
        delta(i) = sqrt(x(i,1)^2 + x(i,2)^2);
    end
    avgerror = mean(delta);

end