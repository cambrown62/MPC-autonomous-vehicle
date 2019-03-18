%% system parameters

l_f = .460/2; % half length of vehicle
h = .1; % sample time
T = 1000; % length of simulation
stanley_gain = 500; % gain of the controller
del_upper_bnd = 25*(pi/180); % bounds on steering angle
del_lower_bnd = -del_upper_bnd;

% model for x, y, heading (psi), velocity of vehicle
x_kp1 = @(x, v, psi) x + v*cos(psi)*h;
y_kp1 = @(y, v, psi) y + v*sin(psi)*h;
psi_kp1 = @(psi, v, del) psi + (v/l_f)*del*h;
%v_kp1 = @(v) v; % assume constant velocity

% heading error and cross track error
%heading_err = @(desired_psi, psi) desired_psi - psi;
crosstrack_err = @(path_y, y, path_x, x) sqrt((path_x-x)^2 + (path_y-y)^2);

% coords for entire path (we are given this by motion planner)
path_x = [.01:.01:10];
path_y = -sqrt(10^2 - path_x.^2) + 10;
desired_psi = atan(path_x/sqrt(100-path_x.^2)); % heading of the path at every point

% initial vehicle states
x0 = -0.01;
y0 = -0.01;
v0 = .01;
%acc0 = 0;
psi0 = 0;
del0 = 0;
%%psi_err0 = heading_err(desired_psi(1), psi0);
closest_y = path_y(1);
closest_x = path_x(1);
cte0 = crosstrack_err(closest_y, y0, closest_x, x0);

% figure(1)
% plot(path_x, path_y, '*')

x = zeros(1,T); 
y = zeros(1,T);
%v_out = [v0 []];
%acc_out = [acc0 []];
psi = zeros(1,T);
del = zeros(1,T);
%psi_err = [psi_err0 []];
cte = zeros(1,T);

x(1) = -0.01;
y(1) = -0.01;
psi(1) = 0;
del(1) = 0;
cte(1) = cte0;
del(1) = psi(1) + atan(stanley_gain*cte(1)/v0);
if del(1) > del_upper_bnd
    del(1) = del_upper_bnd;
elseif del(1) < del_lower_bnd
    del(1) = del_lower_bnd;
end

for t = 2:T
    
    psi(t) = psi_kp1(psi(t-1), v0, del(t-1));
    x(t) = x_kp1(x(t-1), v0, psi(t-1));
    y(t) = y_kp1(y(t-1), v0, psi(t-1));
    
    if crosstrack_err(path_y(t), y(t), path_x(t), x(t)) < crosstrack_err(closest_y, y(t), closest_x, x(t))
        closest_y = path_y(t);
        closest_x = path_x(t);
    end
    cte(t) = crosstrack_err(closest_y, y(t), closest_x, x(t));
    rel_angle(t) = atan2(x(t)*closest_y - y(t)*closest_x, x(t)*closest_x + y(t)*closest_y);
    del(t) = sign( atan2(x(t)*closest_y - y(t)*closest_x, x(t)*closest_x + y(t)*closest_y) )*(psi(t) + atan(stanley_gain*cte(t)/v0));
    %del(t) = stanley_gain*cte(t);
    if del(t) > del_upper_bnd
        del(t) = del_upper_bnd;
    elseif del(t) < del_lower_bnd
        del(t) = del_lower_bnd;
    end
end

% figure(2)
% plot(1:T,cte)

figure(3)
plot(path_x, path_y, '*', x, y)

% figure(4)
% plot(1:T, rel_angle)
