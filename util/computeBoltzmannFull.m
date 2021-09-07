function P = computeBoltzmannFull(xH, xR, uR, uH1_vec, uH2_vec,...
    planner, beta, theta)
% function P = computeBoltzmannFull(xH, xR, uR, uH1_vec, uH2_vec,...
%     planner, beta, theta)
%     Compute a distribution of human's action based on beta and theta

    Nu1 = length(uH1_vec);
    Nu2 = length(uH2_vec);
    
    QH = zeros(Nu1,Nu2);
    for i = 1:Nu1
        for j = 1:Nu2
            uH = [uH1_vec(i); uH2_vec(j)];
            xH_next = planner.Ad*xH + planner.Bd*uH;
            % Q1: lane changing to the right, and lane keeping with the
            % cruising speed (theta = 1)
            xH_err = xH - planner.xH_des_cell{1};
            xH_err = xH_err(2:3);
            xH_next_err = xH_next - planner.xH_des_cell{1};
            xH_next_err = xH_next_err(2:3);
            Q1 = xH_err'*planner.Q_H(2:3,2:3)*xH_err +...
                uH'*planner.R_H*uH +...
                xH_next_err'*planner.P_H_23*xH_next_err;
            % Q2: lane changing to the left, and lane keeping with the
            % cruising speed (theta = 0)
            xH_err = xH - planner.xH_des_cell{2};
            xH_err = xH_err(2:3);
            xH_next_err = xH_next - planner.xH_des_cell{2};
            xH_next_err = xH_next_err(2:3);
            Q2 = xH_err'*planner.Q_H(2:3,2:3)*xH_err +...
                uH'*planner.R_H*uH +...
                xH_next_err'*planner.P_H_23*xH_next_err;
            QH(i,j) = [theta 1-theta]*[Q1;Q2];
        end
    end
    
    QH_zero_mean = QH - mean(QH(:)); % Avoid numerical overflow
    P = exp(-beta*QH_zero_mean);
    P = P/sum(P(:));
    
    if any(isnan(P))
        error('The P matrix contains NaN! Check for numerical overflow!')
    end
    
end