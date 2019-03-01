%% System Parameters

h = 0.1; % sample time
N = 10; % prediction horizon
l_f = 460/2;
del_upper_bnd = 25*(pi/180);
del_lower_bnd = -del_upper_bnd;
acc_upper_bnd = 1;
acc_lower_bnd = -1;
v_target = 1;

%% Trajectory Model

% parameters for modelling path as 3rd order polynomial
a0 = 0;
a1 = .01;
a2 = .01;
a3 = .01;

% calculate y coordinates of the path and desired vehicle heading psi
traj_y = @(x) a3*x.^3 + a2*x.^2 + a1*x + a0;
desired_psi = @(x) atan(3*a3*x.^2 + 2*a2*x + a1*x);

%% Kinematic Bicycle Model

% model for x,y, heading, velocity of vehicle
x_kp1 = @(x, v, psi) x + v*cos(psi)*h;
y_kp1 = @(y, v, psi) y + v*sin(psi)*h;
psi_kp1 = @(psi, v, del) psi + (v/l_f)*del*h;
v_kp1 = @(v, acc) v + acc*h;

% heading error and cross track error
psi_err_kp1 = @(psi, desired_psi, v, del) psi + desired_psi * (v/l_f) * del*h;
crosstrack_err_kp1 = @(traj_y, y, v, psi) traj_y - y + v*sin(psi)*h;

%% Cost Function

% weights for the cost function
w_cte = 1;
w_psi_err = 1;
w_v = 1;
w_del = 1;
w_a = 1;
w_del_dot = 1;
w_jerk = 1;


% coords for entire path
path_x = [0:.1:10];
path_y = traj_y(path_x);
desired_psi = desired_psi(path_x);

% initial vehicle states
x0 = -0.01;
y0 = -0.01;
v0 = 0;
psi0 = 0;
del0 = 0;
cte0 = crosstrack_err_kp1(path_y(1), y0, v0, psi0);
psi_err0 = psi_err_kp1(psi0, desired_psi0(1), v0, del0);

figure(1)
plot(path_x0, path_y0, '*')




