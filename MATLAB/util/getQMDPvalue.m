function VOptR = getQMDPvalue(xR, xH, param_distr, planner, QMDP, mode,...
    isProj)
% function VOptR = getQMDPvalue(xR, xH, param_distr, planner, QMDP, mode,...
%     isProj)
%     Compute the QMDP optimal value using two methods: 1. MAP and 
%     2. computing the expected value

    % Initialize the optimal cost-to-go value
    VOptR = 0;

    % Change coordinates
    if isfield(QMDP,'z7vec')
        xPlan = planner.E_6DtoPlan5D*[xR;xH];
    else
        xPlan = planner.E_6DtoPlan4D*[xR;xH];
    end
    
    % Expected value
    if nargin < 6
        mode = 'expectation';
        isProj = 1;
    end
    
    % Whether to project the input back to the QMDP grid
    if nargin < 7
        isProj = 1;
    end

    if strcmp(mode, 'MAP')
        % MAP to get beta and theta
        [~,idx] = max(param_distr(:));
        [idx1,idx2] = ind2sub([planner.Nbeta, planner.Ntheta],idx);
        beta  = planner.beta_vec(idx1);
        theta = planner.theta_vec(idx2);
        % Augmented state
        z = [xPlan; log10(beta); theta];
        % Project z back to the QMDP grid
        if isProj
            z = min(max(z,QMDP.g.min),QMDP.g.max);
        end
        % Find the optimal value via table-lookup
        VOptR = eval_u(QMDP.g, QMDP.VOpt, z);
        if isnan(VOptR)
            VOptR = Inf;
        end
    elseif strcmp(mode, 'expectation')
        % Augmented state
        z = [xPlan; log10(planner.beta_vec(1)); planner.theta_vec(1)];
        % Project z back to the QMDP grid
        if isProj
            z = min(max(z,QMDP.g.min),QMDP.g.max);
        end
        % Check if current z is in the QMDP gird
        Vz = eval_u(QMDP.g, QMDP.VOpt, z);
        if isnan(Vz)
            VOptR = Inf;
            return
        end

        % Main loop for computing the expected value 
        for i = 1:planner.Nbeta
            for j = 1:planner.Ntheta
                z = [xPlan; log10(planner.beta_vec(i)); planner.theta_vec(j)];
                if isProj
                    z = min(max(z,QMDP.g.min),QMDP.g.max);
                end
                Vz = eval_u(QMDP.g, QMDP.VOpt, z);
                Pz = param_distr(i,j);
                VOptR = VOptR + Pz*Vz;
            end
        end
    else
        error('Invalid mode!')
    end
end