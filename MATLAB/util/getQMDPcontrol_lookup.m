function uOptR = getQMDPcontrol_lookup(xR, xH, param_distr, planner,...
    QMDP, isProj)
% function uOptR = getQMDPcontrol_lookup(xR, xH, param_distr, planner,...
%     QMDP, isProj)
%     Compute the QMDP policy via table look-up

    % Whether to project the input back to the QMDP grid
    if nargin < 6
        isProj = 1;
    end

    % Change coordinates
    if isfield(QMDP,'z7vec')
        xPlan = planner.E_6DtoPlan5D*[xR;xH];
    else
        xPlan = planner.E_6DtoPlan4D*[xR;xH];
    end

    % Compute beta and theta via MAP
    [~,idx] = max(param_distr(:));
    [idx1,idx2] = ind2sub([planner.Nbeta, planner.Ntheta], idx);
    beta  = planner.beta_vec(idx1);
    theta = planner.theta_vec(idx2);

    % Augmented state
    z = [xPlan; log10(beta); theta];

    % Project z back to the QMDP grid
    if isProj
        z = min(max(z,QMDP.g.min),QMDP.g.max);
    end

    % Get the control input via table look-up
    u1 = eval_u(QMDP.g, QMDP.U1Opt, z);
    if isnan(u1)
        uOptR = NaN;
        return
    end

    u2 = eval_u(QMDP.g, QMDP.U2Opt, z);
    if isnan(u2)
        uOptR = NaN;
        return
    end

    uOptR = [u1;u2];
end