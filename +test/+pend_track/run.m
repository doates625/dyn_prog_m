function run(horizon, interp_)
%RUN(horizon) Pendulum Tracking Dynamic Programming Test
%   
%   Inputs:
%   - horizon = Horizon [char]
%       'Infinite' (default)
%       'Finite'
%   - interp_ = Sim interpolation [char]
%       'Linear' (default)
%       'Nearest'
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('dyn_prog.test.pend_track.f');
import('dyn_prog.test.pend_track.g');
import('dyn_prog.test.pend_track.h');
import('dyn_prog.test.pend_track.set_get_dt');
import('dyn_prog.synw');
import('multi_array.Range');
import('controls.wrap');

% Default args
if nargin < 1,  horizon = 'Infinite'; end
if nargin < 2, interp_ = 'Linear'; end

% Title
clc
fprintf('Pendulum Tracking DP\n')
fprintf('%s Horizon\n\n', horizon)

% Time range [s]
t_min = 0;      % Min
t_max = 20;     % Max
t_size = 64;    % Steps

% Angle range [rad]
x1_min = -pi;   % Min
x1_max = +pi;   % Max
x1_size = 8;    % Steps

% Velocity range [rad/s]
x2_min = -pi;   % Min
x2_max = +pi;   % Max
x2_size = 8;    % Steps

% Torque range [rad/s^2]
u1_min = -10;  % Min
u1_max = +10;  % Max
u1_size = 4;   % Steps

% Setpoint range [rad]
w1_min = -pi;   % Min
w1_max = +pi;   % Max
w1_size = 8;    % Steps

% Configure dt
t = linspace(t_min, t_max, t_size);
dt_ctrl = t(2) - t(1);
set_get_dt(dt_ctrl);

% Dynamic programming
fprintf('Calculating...\n');
x_rng = Range([x1_min, x2_min], [x1_max, x2_max], [x1_size, x2_size]);
u_rng = Range(u1_min, u1_max, u1_size);
w_rng = Range(w1_min, w1_max, w1_size);
[u_opts, j_mins] = synw(@f, @g, @h, t_size, x_rng, u_rng, w_rng, horizon, 100);

% Override unstable equilibrium
u = u_rng.vals_max;
x1 = x_rng.get_arr(1);
for sub_x1 = 1 : length(x1)
    x = [x1(sub_x1); 0];
    w = wrap(x(1) - pi, -pi, +pi);
    xw = [x; w];
    switch horizon
        case 'Finite'
            for k = 1 : t_size   
                xn = f(x, u, w, k);
                xwn = [xn; w];
                u_opts(k).set(xw, u);
                jn = j_mins(k).get(xwn);
                j = g(x, u, w, k) + jn;
                j_mins(k).set(xw, j);
            end
        case 'Infinite'
            xn = f(x, u, w, 1);
            xwn = [xn; w];
            u_opts.set(xw, u);
            jn = j_mins.get(xwn);
            j = g(x, u, w, 1) + jn;
            j_mins.set(xw, j);
    end
end

% Simulation
fprintf('Simulating...\n');

% Simulation logs
dt_sim = set_get_dt(0.01);
t_sim = t_min : dt_sim : t_max;
t_size = length(t_sim);
x_sim = zeros(2, t_size);
u_sim = zeros(1, t_size);
w_sim = zeros(1, t_size);
j_sim = zeros(1, t_size);
x_sim(:, 1) = [0; 0];

% Setpoint vector
w_sim(t_sim > 05) = +pi/2;
w_sim(t_sim > 10) = -pi/2;
w_sim(t_sim > 15) = pi;

% Run simulation
for ks = 1:t_size
    k = ceil(ks * dt_sim / dt_ctrl);
    x = x_sim(:, ks);
    w = w_sim(:, ks);
    xw = [x; w];
    switch horizon
        case 'Finite'
            u = u_opts(k).get(xw, interp_);
            j_sim(ks) = j_mins(k).get(xw, interp_);
        case 'Infinite'
            u = u_opts.get(xw, interp_);
            j_sim(ks) = j_mins.get(xw, interp_);
    end
    x_sim(:, ks+1) = f(x, u, w, k);
    u_sim(:, ks) = u;
end
x_sim = x_sim(:, 1 : t_size);

% Plotting
fprintf('Plotting...\n');
figure(1)

% Angle plot
subplot(2, 1, 1)
hold on, grid on
title('Angle Control')
xlabel('Time [s]')
ylabel('Angle [rad]')
plot(t_sim, w_sim(1, :), 'k--')
plot(t_sim, x_sim(1, :), 'b-')
legend('Cmd', 'Val')

% Control plot
subplot(2, 1, 2)
hold on, grid on
title('Control Acceleration')
xlabel('Time [s]')
ylabel('Acceleration [rad/s^2]')
plot(t_sim, u_sim, 'r-')

% Final display
fprintf('Complete!\n\n')

end