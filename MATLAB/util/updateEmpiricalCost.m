function J_em = updateEmpiricalCost(planner, J_em, xR, xH, uR)
% function J_em = updateEmpiricalCost(planner, J_em, xR, xH, uR)
%     Update the robot's empirical cost
    target_state = planner.xR_plan_des;
    target_state(3) = planner.vH+3;
    error_state = planner.E_6DtoRplan*[xR;xH] - target_state;
    J_em = J_em + error_state'*planner.Q_R*error_state +...
        uR'*planner.R_R*uR;
end

