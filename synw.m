function [u_opts, j_mins] = synw(f, g, h, N, x_rng, u_rng, w_rng, horiz, j_init)
%[u_opts, j_mins] = SYNW(f, g, h, N, x_rng, u_rng, w_rng, horiz, j_init)
%   Dynamic programming synthesis with exogeneous inputs
%   
%   Inputs:
%   - f = State transition [function_handle]
%   - g = Running cost [function_handle]
%   - h = Terminal cost [function_handle]
%   - N = Number of discrete samples [int]
%   - x_rng = Admissible states [multi_array.Range]
%   - u_rng = Admissible controls [multi_array.Range]
%   - w_rng = Exogeneous input range [multi_array.Range]
%   - horiz = Horizon ['Finite', 'Infinite']
%   - j_init = Initial cost of all states [double]
%   
%   Outputs:
%   - u_opts = Optimal control LUT(s) [multi_array.LUT]
%   - j_mins = Minimum cost LUT(s) [multi_array.LUT]
%   
%   Given the discrete-time dynamic system:
%       
%       x[k+1] = f(x[k], u[k], w[k], k)
%       J = h(x[N+1]) + sum(g(x[k], u[k], w[k], k))
%       k = 1 ... N
%   
%   SYNW estimates the optimal controls u as a function of state x, input w,
%   and sampl k which minimize the cost functioal J. For 'Finite' horizon,
%   SYNW returns a [1 x N] LUT array of optimal controls and [1 x N+1] LUT array
%   of minimum costs corresponding to samples k = 1 ... N, with the minimum
%   costs at sample N+1 corresponding to the terminal costs. For 'Infinite'
%   horizon, Only the LUTs for sample k = 1 are returned for both.
%   
%   See also: SYN
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('dyn_prog.syn');
import('multi_array.Range');

% Augmented dynamic system
n = x_rng.rank_;
f_xw = @(xw, u, k) [f(xw(1:n), u, xw(n+1:end), k); xw(n+1:end)];
g_xw = @(xw, u, k) g(xw(1:n), u, xw(n+1:end), k);
h_xw = @(xw, u, k) h(xw(1:n));

% Augmented input range
xw_min = [x_rng.vals_min; w_rng.vals_min];
xw_max = [x_rng.vals_max; w_rng.vals_max];
xw_size = [x_rng.size_, w_rng.size_];
xw_rng = Range(xw_min, xw_max, xw_size);

% Solve augmented problem
[u_opts, j_mins] = syn(f_xw, g_xw, h_xw, N, xw_rng, u_rng, horiz, j_init);

end