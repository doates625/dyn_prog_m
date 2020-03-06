function [u_opts, j_mins] = syn(f, g, h, N, x_rng, u_rng, horiz, j_init)
%[u_opts, j_mins] = SYN(f, g, h, N, x_rng, u_rng, horizon, j_init)
%   Dynamic programming synthesis
%   
%   Inputs:
%   - f = State transition [function_handle]
%   - g = Running cost [function_handle]
%   - h = Terminal cost [function_handle]
%   - N = Number of discrete samples [int]
%   - x_rng = Admissible states [multi_array.Range]
%   - u_rng = Admissible controls [multi_array.Range]
%   - horiz = Horizon ['Finite', 'Infinite']
%   - j_init = Initial cost of all states [double]
%   
%   Outputs:
%   - u_opts = Optimal control LUT(s) [multi_array.LUT]
%   - j_mins = Minimum cost LUT(s) [multi_array.LUT]
%   
%   Given the discrete-time dynamic system:
%       
%       x[k+1] = f(x[k], u[k], k)
%       J = h(x[N+1]) + sum(g(x[k], u[k], k))
%       k = 1 ... N
%   
%   SYN estimates the optimal controls u as a function of state x and sample k
%   which minimize the cost functioal J. For 'Finite' horizon, SYN returns a
%   [1 x N] LUT array of optimal controls and [1 x N+1] LUT array of minimum
%   costs corresponding to samples k = 1 ... N, with the minimum costs at
%   sample N+1 corresponding to the terminal costs. For 'Infinite' horizon,
%   Only the LUTs for sample k = 1 are returned for both.
%   
%   See also: SYNW
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('timing.ProgDisp');
import('multi_array.Range');
import('multi_array.LUT');

% Generate LUTs
u_opts = cell(N, 1);
j_mins = cell(N+1, 1);

% Progress printing
prog_disp = ProgDisp(1.0);
prog_N = x_rng.numel_ + N * x_rng.numel_ * u_rng.numel_;
prog_i = 1;

% Terminal costs
j_mins{N+1} = LUT(x_rng, 1);
for ind_x = 1 : x_rng.numel_
    
    % Compute cost
    x = x_rng.get(ind_x, 'Ind');
    j_mins{N+1}.set(ind_x, h(x), 'Ind');
    
    % Update progress
    prog_disp.update(prog_i / prog_N);
    prog_i = prog_i + 1;
end

% Running costs
u_dim = u_rng.rank_;
for k = N : -1 : 1
    
    % Create LUTs
    u_opts{k} = LUT(x_rng, u_dim);
    j_mins{k} = LUT(x_rng, 1);
    
    % For each admissible state x
    for ind_x = 1 : x_rng.numel_
       
        % Init search
        x = x_rng.get(ind_x, 'Ind');
        u_opt = 0;
        j_min = j_init;
        
        % For each admissible control u
        for ind_u = 1 : u_rng.numel_
            u = u_rng.get(ind_u, 'Ind');
            xn = f(x, u, k);
            
            % If next state admissible
            if x_rng.has(xn)
                jn = j_mins{k+1}.get(xn, 'Linear');
                j = g(x, u, k) + jn;
                if j < j_min
                    u_opt = u;
                    j_min = j;
                end
            end
            
            % Update progress
            prog_disp.update(prog_i / prog_N);
            prog_i = prog_i + 1;
        end
        
        % Update LUTs
        u_opts{k}.set(ind_x, u_opt, 'Ind');
        j_mins{k}.set(ind_x, j_min, 'Ind');
    end
end

% Convert cells to LUT arrays
if strcmp(horiz, 'Finite')
    u_opts_cell = u_opts;
    j_mins_cell = j_mins;
    u_opts = LUT.empty(1, 0);
    j_mins = LUT.empty(1, 0);
    for k = 1:N
        j_mins(1, k) = j_mins_cell{k};
        u_opts(1, k) = u_opts_cell{k};
    end
    j_mins(1, N+1) = j_mins_cell{N+1};
elseif strcmp(horiz, 'Infinite')
    u_opts = u_opts{1};
    j_mins = j_mins{1};
else
    error('Invalid horizon: ''%s''', horiz)
end

end