function x_next = kinematicBicycle(x, u, ts, params)
% function x_next = kinematicBicycle(x, u, ts, params)
%     The nonlinear kinematic bicycle model for simulation
%     State: [px py v]
%     Computed control:  [a vLat]
%     Converted control: [a theta] (theta = asin(vLat/v))
    vLat = u(2);
    v    = x(3);
    theta = asin(max(min(vLat/v,1),-1));
    
    x_next(1,1) = x(1) + ts*x(3)*cos(theta);
    x_next(2,1) = x(2) + ts*x(3)*sin(theta);
    x_next(3,1) = x(3) + ts*(u(1) - params.talpha*x(2));
end