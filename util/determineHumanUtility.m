function [Q_H_opt, R_H_opt] = determineHumanUtility(XH, UH, planner)
% function [Q_H_opt, R_H_opt] = determineHumanUtility(XH, UH, planner)
%     "Learn" the human's cost matrices from her trajectory data

    % Initialize human's cost matrices entries
    NQ  = 10;
    NR1 = 5;
    NR2 = 5;
    Q_H_33_vec = linspace(0.01, 1, NQ);
    R_H_11_vec = linspace(1, 50, NR1);
    R_H_22_vec = linspace(1, 50, NR2);

    % Human control discretization
    NuH = [5,5];
    uH1_vec = linspace(planner.ulb_H(1),planner.uub_H(1),NuH(1));
    uH2_vec = linspace(planner.ulb_H(2),planner.uub_H(2),NuH(2));
    
    % Initialize cost matrices
    Q_H_opt = NaN;
    R_H_opt = NaN;
    pred_err_opt = inf;

    for Q_H_33 = Q_H_33_vec
        for R_H_11 = R_H_11_vec
            for R_H_22 = R_H_22_vec

                Q_H = blkdiag(0.1, 0.1, Q_H_33);
                R_H = blkdiag(R_H_11, R_H_22);

                % Design LQR cost-to-go (H only regulates p_y and v)
                [~,P_H_23,~] = dlqr(planner.Ad(2:3,2:3),...
                    planner.Bd(2:3,:), Q_H(2:3,2:3), R_H);
                
                % Initialize the prediction error
                pred_err = 0;

                for t = 1:size(UH,2)

                    beta = 1;

                    if UH(2,t)<=0 % If target is the right lane
                        theta = 1;
                    else % If target is the left lane
                        theta = 0;
                    end

                    planner.Q_H = Q_H;
                    planner.R_H = R_H;
                    planner.P_H_23 = P_H_23;

                    P_uH = computeBoltzmannFull(XH(:,t), [], [],...
                        uH1_vec, uH2_vec, planner, beta, theta);
                    [~,idx] = max(P_uH(:));
                    [idx1,idx2] = ind2sub([NuH(1), NuH(2)], idx);

                    uH_pred = [uH1_vec(idx1); uH2_vec(idx2)];
                    
                    % Update the prediction error
                    pred_err = pred_err + norm(uH_pred-UH(:,t));
                end

                if pred_err < pred_err_opt
                    pred_err_opt = pred_err;
                    Q_H_opt = Q_H;
                    R_H_opt = R_H;
                end
            end
        end
    end
end