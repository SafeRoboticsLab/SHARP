function [uSol, VSol, stime] = getSMPCcontrol(xR, xH, param_distr, HJ,...
                                       planner, QMDP, tree_params, verbose)
% function [uSol, VSol, stime] = getSMPCcontrol(xR, xH, param_distr, HJ,...
%                                      planner, QMDP, tree_params, verbose)
%     Compute the SHARP-SMPC policy
%
% ----- How to use this function -----
%
% Inputs:
%   xR              - Robot's state
%   xH              - Human's state
%   param_distr     - Joint distribution of beta and theta
%   HJ              - HJ pre-computation results
%   planner         - The planner object
%   QMDP            - QMDP pre-computation results
%   tree_params     - Scenario tree parameters
%   verbose         - Verbose indicator
%
% Outputs:
%   uSol            - Optimal control input
%   VSol            - Optimal cost-to-go
%   stime           - Solving time
%
%
% Author: Haimin Hu (last modified 2021.9.6)

    yalmip('clear')

    % Define solver settings
    options = sdpsettings('verbose', 0, 'solver', 'mosek', 'usex0', 0,...
        'cachesolvers', 1);

    % Compute robot's target planning state
    xR_plan_des = planner.xR_plan_des;
    yH = xH(2);
    if yH >= 0
        xR_plan_des(2) = -planner.lc;
    else
        xR_plan_des(2) = planner.lc;
    end

    %----------------------------------------------------------------------
    % Constructing a scenario tree
    %----------------------------------------------------------------------
    % Define the root node
    root_node = node(1, 0, xR, xH, param_distr, NaN, NaN, 1, NaN, 'OT');
    
    % Initialize a scenario tree
    stree = tree(root_node, HJ, QMDP, planner, tree_params.Cmax,...
                 tree_params.max_per_l, tree_params.Nbranch,...
                 tree_params.NuR, tree_params.NuH, tree_params.QMDP_K,...
                 tree_params.ts_params, tree_params.N_skip,...
                 tree_params.branch_count_max,...
                 tree_params.cost_diff_pct_thresh, tree_params.time_out);

    % Construct a dense tree [Bernardini and Bemporad]
%     stree = stree.constructDense(tree_params.Nnode, verbose);

    % Construct a sparse LQG tree
%     stree = stree.constructSparse(tree_params.Nnode,...
%         tree_params.N_max, verbose);

    % Construct a sparse LQG tree with CBF-based shielding constraints
    stree = stree.constructSparseShield(tree_params.Nnode,...
        tree_params.N_max, verbose);
    
    % In case of a tree with only the root node, use the QMDP policy
    if length(stree.N) == 1
        isProj = true;
        uSol = stree.N(1).getQMDPcontrol(planner, QMDP, isProj);
        VSol = getQMDPvalue(xR, xH, param_distr, planner, QMDP);
        stime = [];
        return
    end

    %----------------------------------------------------------------------
    % Objective function
    %----------------------------------------------------------------------
    objective = 0;
    for i = 1:length(stree.N)
        xR_plan = planner.E_6DtoRplan*[stree.N(i).xR_opt;stree.N(i).xH];
        % Terminal costs
        if stree.N(i).IsLeaf
            objective = objective +...
                stree.N(i).path_prob*(stree.N(i).lambda*stree.N(i).Vf');
        % Stage costs
        else
            objective = objective + stree.N(i).path_prob*...
             ((xR_plan-xR_plan_des)'*planner.Q_R*(xR_plan-xR_plan_des) +...
             stree.N(i).uR_opt'*planner.R_R*stree.N(i).uR_opt);
        end
    end

    %----------------------------------------------------------------------
    % Inputs bounds
    %----------------------------------------------------------------------
    constraint = [];
    for i = 1:length(stree.N)
        if ~stree.N(i).IsLeaf
            constraint = [constraint,...
                planner.ulb_R <= stree.N(i).uR_opt <= planner.uub_R];
        end
    end

    %----------------------------------------------------------------------
    % Dynamics of the robot
    %----------------------------------------------------------------------
    A = planner.Ad;
    B = planner.Bd;
    for i = 1:length(stree.N)
        if stree.N(i).idx ~= 1
            ip = stree.N(i).pre_node_idx;
            constraint = [constraint, stree.N(i).xR_opt ==...
                A*stree.N(ip).xR_opt + B*stree.N(ip).uR_opt];
        end
    end

    %----------------------------------------------------------------------
    % Terminal set constraints
    %----------------------------------------------------------------------
    for i = 1:length(stree.N)
        if stree.N(i).IsLeaf
            constraint = [constraint stree.N(i).lambda(:) >= 0];
            constraint = [constraint...
                stree.N(i).lambda*ones(length(stree.N(i).lambda),1) == 1];
            constraint = [constraint...
                stree.N(i).xR_opt == stree.N(i).Xf*stree.N(i).lambda'];
        end
    end

    constraint_sh = [];
    %----------------------------------------------------------------------
    % Convex approximate HJ constraints [K. Leung et al]
    %----------------------------------------------------------------------
%     for i = 1:length(stree.Ns)
%         idx = stree.Ns(i);
%         xRel = planner.E_6DtoPlan4D*[stree.N(idx).xR; stree.N(idx).xH];
%         [uR_sh, dVdx] = computeShielding(xRel, HJ);
%         theta_MAP = stree.N(idx).computeThetaMAP(planner);
%         uH = computeRationalHumanAction(stree.N(idx).xH, [], [],...
%             planner, theta_MAP);
%         uH = min(max(uH, planner.ulb_H), planner.uub_H);
%         H_sh = dVdx'*planner.pf4DpuR;
%         h_sh = dVdx'*(planner.AcRel*xRel+...
%             planner.BcRel*[uR_sh;uH])-H_sh*uR_sh;
%         constraint_sh = [constraint_sh,...
%             -(H_sh*stree.N(idx).uR_opt + h_sh) <= 0];
%     end

    %----------------------------------------------------------------------
    % Convex CBF-based approximate shielding constraints
    %----------------------------------------------------------------------
    gamma = 0.1;
    for i = 1:length(stree.Ns)
        idx = stree.Ns(i);
        xRel = planner.E_6DtoPlan4D*[stree.N(idx).xR; stree.N(idx).xH];
        % Compute robot's shielding policy
        [uR_sh, ~] = computeShielding(xRel, HJ);
        theta_MAP = stree.N(idx).computeThetaMAP(planner);
        % Compute human's rational action
        uH = computeRationalHumanAction(stree.N(idx).xH, [], [],...
                                                       planner, theta_MAP);
        uH = min(max(uH, planner.ulb_H), planner.uub_H);
        xRel_Next = planner.AdRel*xRel + planner.BdRel*[uR_sh; uH];
        % Compute the normal vector
        n_sh = xRel_Next - xRel;
        % Set up the CBF constraint
        constraint_sh = [constraint_sh,...
            n_sh'*((planner.AdRel+(gamma-1)*eye(4))*...
            planner.E_6DtoPlan4D*[stree.N(idx).xR_opt;stree.N(idx).xH] +...
            planner.BdRel*[stree.N(idx).uR_opt; uH]) >= 0];
    end

    %----------------------------------------------------------------------
    % Solve the ST-SMPC problem
    %----------------------------------------------------------------------
    tic
    status = optimize([constraint, constraint_sh], objective, options);
    stime = toc;
    isSolved = (status.problem==0);
    if ~isSolved
        message = ['solver info: ', status.info ];
        warning(message);
    end

    if verbose
        disp(['QP solving time: ', num2str(stime)])
    end

    %----------------------------------------------------------------------
    % Solve a relaxed problem if the original one is infeasible
    %----------------------------------------------------------------------
    if ~isSolved
        tic
        status = optimize(constraint, objective, options);
        stime = toc;
        isSolved = (status.problem==0);
        if ~isSolved
            message = ['solver info: ', status.info ];
            warning(message);
        end

        if verbose
            disp(['QP solving time: ', num2str(stime)])
        end
    end

    %----------------------------------------------------------------------
    % Return values
    %----------------------------------------------------------------------
    uSol = value(stree.N(1).uR_opt);
    VSol = value(objective);
    clear root_node stree
end