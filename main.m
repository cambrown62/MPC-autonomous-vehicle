%% System Parameters

h = 0.1; % sample time
N = 10; % prediction horizon
L = .460; % length of vehicle
l_f = L/2; % half length of vehicle
del_upper_bnd = 25*(pi/180); % steering angle constraints
del_lower_bnd = -del_upper_bnd; 
acc_upper_bnd = 1; % acceleration constraints
acc_lower_bnd = -1;

%% Kinematic Bicycle Model

% model for x, y, heading (psi), velocity of vehicle, slip angle (beta) at CoG
x_kp1 = @(x, v, psi, beta) x + v*cos(psi + beta)*h;
y_kp1 = @(y, v, psi, beta) y + v*sin(psi + beta)*h;
psi_kp1 = @(psi, v, del, beta) psi + (v/L)*cos(beta)*tan(del)*h;
v_kp1 = @(v, a) v + a*h;
beta_kp1 = @(del) atan(l_f*tan(del)/L)

% heading error and cross track error
heading_err = @(desired_psi, psi) desired_psi - psi;
crosstrack_err = @(path_y, y, path_x, x) sqrt((path_x-x)^2 + (path_y-y)^2);

% coords for entire path (we are given this by motion planner)
path_x = [0:.1:10];
path_y = -sqrt(10^2 - path_x.^2) + 10;
desired_psi = atan(path_x/sqrt(100-path_x.^2)); % heading of the path at every point

% initial vehicle states
x0 = -0.01;
y0 = -0.01;
v0 = 1;
acc0 = 0;
psi0 = 0;
%%del0 = 0;
psi_err0 = heading_err(desired_psi(1), psi0);
cte0 = crosstrack_err(path_y(1), y0, path_x(1), x0);

x_out = [x0 []];
y_out = [y0 []];
%v_out = [v0 []];
acc_out = [acc0 []];
psi_out = [psi0 []];
del_out = [];
psi_err = [psi_err0 []];
cte_out = [cte0 []];

