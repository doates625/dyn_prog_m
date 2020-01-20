function xn = f(x, u, ~, ~)
%xn = F(x, u, w, k) State transition function

import('dyn_prog.test.pend_track.set_get_dt');
import('controls.wrap');
dx_dt = [x(2); u(1) - sin(x(1))];
dt = set_get_dt();
dx = dx_dt * dt;
xn = x + dx;
xn(1) = wrap(xn(1), -pi, +pi);

end