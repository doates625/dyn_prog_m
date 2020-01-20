function run(horizon, interp_)
%RUN(horizon) Pendulum Balancing Dynamic Programming Test
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
import('dyn_prog.test.pend_bal.f');
import('dyn_prog.test.pend_bal.g');
import('dyn_prog.test.pend_bal.h');
import('dyn_prog.test.pend_bal.set_get_dt');
import('dyn_prog.syn');
import('multi_array.Range');

% Default args
if nargin < 1,  horizon = 'Infinite'; end
if nargin < 2, interp_ = 'Linear'; end

% Title
clc
fprintf('Pendulum Balance DP\n')
fprintf('%s Horizon\n\n', horizon)

% Time range [s]
t_min = 0;      % Min
t_max = 15;     % Max
t_size = 64;    % Steps

% Angle range [rad]
x1_min = -pi;   % Min
x1_max = +pi;   % Max
x1_size = 16;   % Steps

% Velocity range [rad/s]
x2_min = -pi;   % Min
x2_max = +pi;   % Max
x2_size = 16;   % Steps

% Torque range [rad/s^2]
u1_min = -0.4;  % Min
u1_max = +0.4;  % Max
u1_size = 8;    % Steps

% Configure dt
t = linspace(t_min, t_max, t_size);
dt_ctrl = t(2) - t(1);
set_get_dt(dt_ctrl);

% Dynamic programming
fprintf('Calculating...\n');
x_rng = Range([x1_min, x2_min], [x1_max, x2_max], [x1_size, x2_size]);
u_rng = Range(u1_min, u1_max, u1_size);
[u_opts, j_mins] = syn(@f, @g, @h, t_size, x_rng, u_rng, horizon, 100);

% Override instability
x = [0; 0];
u = u_rng.vals_max;
switch horizon
    case 'Finite'
        for k = 1 : t_size
            xn = f(x, u, k);
            u_opts(k).set(x, u);
            jn = j_mins(k).get(xn);
            j = g(x, u, k) + jn;
            j_mins(k).set(x, j);
        end
    case 'Infinite'
        xn = f(x, u, 1);
        u_opts.set(x, u);
        jn = j_mins.get(xn);
        j = g(x, u, 1) + jn;
        j_mins.set(x, j);
end

% Simulation
fprintf('Simulating...\n');

% Simulation logs
dt_sim = set_get_dt(0.01);
t_sim = t_min : dt_sim : t_max;
t_size = length(t_sim);
x_sim = zeros(2, t_size);
u_sim = zeros(1, t_size);
j_sim = zeros(1, t_size);
x_sim(:, 1) = [0; 0];

% Run simulation
for ks = 1:t_size
    k = ceil(ks * dt_sim / dt_ctrl);
    x = x_sim(:, ks);
    switch horizon
        case 'Finite'
            u = u_opts(k).get(x, interp_);
            j_sim(ks) = j_mins(k).get(x, interp_);
        case 'Infinite'
            u = u_opts.get(x, interp_);
            j_sim(ks) = j_mins.get(x, interp_);
    end
    x_sim(:, ks+1) = f(x, u, k);
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
plot(t_sim, x_sim(1, :), 'b-')

% Control plot
subplot(2, 1, 2)
hold on, grid on
title('Control Acceleration')
xlabel('Time [s]')
ylabel('Acceleration [rad/s^2]')
plot(t_sim, u_sim, 'r-')

% Final displau
fprintf('Complete!\n\n')

end