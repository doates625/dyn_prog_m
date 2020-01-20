function val = set_get_dt(val)
    %SET_GET_DT Set or get simulation time delta
    %   
    %   val = SET_GET_DT(val) Sets dt to val
    %   val = SET_GET_DT() Gets dt
    persistent dt;
    if nargin
        dt = val;
    end
    val = dt;
end