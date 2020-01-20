function jt = h(x)
%jt = H(x) Terminal cost function

import('controls.wrap');
jt = 10 * (wrap(x(1) - pi, -pi, +pi)^2 + x(2)^2);

end