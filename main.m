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
% traj_y = @(x) a3*x.^3 + a2*x.^2 + a1*x + a0;
% desired_psi = @(x) atan(3*a3*x.^2 + 2*a2*x + a1*x);

%% Kinematic Bicycle Model

% model for x, y, heading (psi), velocity of vehicle
x_kp1 = @(x, v, psi) x + v*cos(psi)*h;
y_kp1 = @(y, v, psi) y + v*sin(psi)*h;
psi_kp1 = @(psi, v, del) psi + (v/l_f)*del*h;
v_kp1 = @(v, acc) v + acc*h;

% heading error and cross track error
heading_err = @(psi, desired_psi, v, del) psi + desired_psi * (v/l_f) * del*h;
crosstrack_err = @(path_y, y, v, psi_err) path_y - y + v*sin(psi_err)*h;

%% Cost Function

% weights for the cost function
w_cte = 1;
w_psi_err = 1;
w_v = 1;
w_del = 1;
w_a = 1;
w_del_dot = 1;
w_jerk = 1;


% coords for entire path (we are given this by motion planner)
path_x = [0:.1:10];
path_y = -sqrt(10^2 - path_x.^2) + 10;
desired_psi = atan(path_x/sqrt(100-path_x.^2)); % heading of the path at every point

% initial vehicle states
x0 = -0.01;
y0 = -0.01;
v0 = 0;
acc0 = 0;
psi0 = 0;
del0 = 0;
psi_err0 = heading_err(psi0, desired_psi(1), v0, del0);
cte0 = crosstrack_err(path_y(1), y0, v0, psi_err0);

x_out = [x0 []];
y_out = [y0 []];
v_out = [v0 []];
acc_out = [acc0 []];
psi_out = [psi0 []];
del_out = [del0 []];
psi_err = [psi_err0 []];
cte_out = [cte0 []];

x_pred = [x0 zeros(1,N-1)];
y_pred = [y0 zeros(1,N-1)];
psi_pred = [psi0 zeros(1,N-1)];
v_pred = [v0 zeros(1,N-1)];
cte_pred = [cte0 zeros(1,N-1)];
psi_err_pred = [psi_err0 zeros(1,N-1)];


% figure(1)
% plot(path_x, path_y, '*')

for t = 1:(length(path_x)-6)
    
    x_pred(1) = x_out(t);
    y_pred(1) = y_out(t);
    psi_pred(1) = psi_out(t);
    v_pred(1) = v_out(t);
    cte_pred(1) = cte_out(t);
    psi_err_pred(1) = psi_err_out(t);
    
    for i = 2:N
        x_pred(i) = x_kp1(x_pred(i-1), v_pred(i-1), psi_pred(i-1));
        y_pred(i) = y_kp1(y_pred(i-1), v_pred(i-1), psi_pred(i-1));
        psi_pred(i) = psi_kp1(psi_pred(i-1), v_pred(i-1), del);
        v_pred(i) = v_kp1(v_pred(i-1), acc);
        psi_err(i) = heading_err(psi_pred(i), desired_psi(), v_pred(i), del);
        cte_pred(i) = crosstrack_error(path_y(),y_pred(i), v_pred(i), psi_err(i));
    end
    
end

