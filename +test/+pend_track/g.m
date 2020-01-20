function dj = g(x, ~, w, ~)
%dj = G(x, u, w, k) Running cost function

import('dyn_prog.test.pend_bal.set_get_dt');
import('controls.wrap');
dj_dt = wrap(x(1) - w(1), -pi, +pi)^2;
dt = set_get_dt();
dj = dj_dt * dt;

end