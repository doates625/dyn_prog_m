function dj = g(x, ~, ~)
%dj = G(x, u, k) Running cost function

import('dyn_prog.test.pend_bal.set_get_dt');
import('controls.wrap');
dj_dt = wrap(x(1) - pi, -pi, +pi)^2;
dt = set_get_dt();
dj = dj_dt * dt;

end