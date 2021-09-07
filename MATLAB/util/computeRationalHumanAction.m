function uH = computeRationalHumanAction(xH, xR, uR, planner, theta)
% function uH = computeRationalHumanAction(xH, xR, uR, planner, theta)
%     Compute human's rational (most likey) action
    
    % Compute human's goal state
    xGoal = theta*planner.xH_des_cell{1}+(1-theta)*planner.xH_des_cell{2};
    
    % Compute human's most likely action
    H = planner.R_H+planner.Bd(2:3,:)'*planner.P_H_23*planner.Bd(2:3,:);
    f = (xH(2:3)'*planner.Ad(2:3,2:3)'*planner.P_H_23*planner.Bd(2:3,:)-...
        xGoal(2:3)'*planner.P_H_23*planner.Bd(2:3,:) )';
    uH = -inv(H)*f;
end