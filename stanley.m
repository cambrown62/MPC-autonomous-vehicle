clear all;
clc;

%% system parameters

L = .46; %length of vehicle
l_f = .46/2; % half length of vehicle
h = .1; % sample time
T = 1600; % length of simulation
stanley_gain = 10; % gain of the controller
del_upper_bnd = 25*(pi/180); % bounds on steering angle
del_lower_bnd = -del_upper_bnd;

% model for x, y position of front axle, and heading (psi) of vehicle
x_kp1 = @(x, v, psi, del) x + v*cos(psi + del)*h;
y_kp1 = @(y, v, psi, del) y + v*sin(psi + del)*h;
psi_kp1 = @(psi, v, del) psi + (v/L)*sin(del)*h;
%v_kp1 = @(v) v; % assume constant velocity

% heading error and cross track error
%heading_err = @(desired_psi, psi) desired_psi - psi;
crosstrack_err = @(path_y, y, path_x, x) sqrt((path_x-x)^2 + (path_y-y)^2);

% coords for entire path (we are given this by motion planner)
path_x = [.01:.01:10];
path_y = -sqrt(10^2 - path_x.^2) + 10;
path_y = sin(path_x);
path_heading = atan(path_x./sqrt(100-path_x.^2)); % heading of the path at every point (the atan of the path's derivative at every point)
path_heading = cos(path_x);

%%psi_err0 = heading_err(desired_psi(1), psi0);
closest_y = path_y(1);
closest_x = path_x(1);
path_queue = [path_x(1:6); path_y(1:6)];
path_queue_start_index = 1;


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

% initial vehicle states
v0 = .5;
x(1) = -.1;
y(1) = -.1;
psi(1) = 0;
del(1) = 0;
cte(1) = crosstrack_err(closest_y, y(1), closest_x, x(1));
del(1) = (psi(1)-path_heading(1)) + atan(stanley_gain*cte(1)/v0);
if del(1) > del_upper_bnd
    del(1) = del_upper_bnd;
elseif del(1) < del_lower_bnd
    del(1) = del_lower_bnd;
end

for t = 2:T
    
    % update x, y and heading of vehicle
    psi(t) = psi_kp1(psi(t-1), v0, del(t-1));
    x(t) = x_kp1(x(t-1), v0, psi(t-1), del(t-1));
    y(t) = y_kp1(y(t-1), v0, psi(t-1), del(t-1));
    
    % search through queue of path points to see if any point is closer
    % than the current closest
    for i = 1:length(path_queue(1,:))
        if crosstrack_err(path_queue(2,i), y(t), path_queue(1,i), x(t)) < crosstrack_err(closest_y, y(t), closest_x, x(t))
            path_queue_start_index = path_queue_start_index + i - 1;
            closest_y = path_queue(2,i);
            closest_x = path_queue(1,i);
            
            if path_queue_start_index+5 > length(path_y)
                path_queue = [path_x(path_queue_start_index:end); path_y(path_queue_start_index:end)];
            else
                path_queue = [path_x(path_queue_start_index:path_queue_start_index+5); path_y(path_queue_start_index:path_queue_start_index+5)];
            end
            
            break
        end
    end
    
    % calculate crosstrack error then steering command
    cte(t) = crosstrack_err(closest_y, y(t), closest_x, x(t));
    rel_angle(t) = atan2(x(t)*closest_y - y(t)*closest_x, x(t)*closest_x + y(t)*closest_y);
    del(t) = sign( rel_angle(t) )*((psi(t) - path_heading(path_queue_start_index)) + atan(stanley_gain*cte(t)/v0));

    % constrain steering command
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
