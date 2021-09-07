function planner = initializePlanner(planner)
% function planner = initializePlanner(planner)
%     Set problem parameters and store them in the planner object

    %% Human uncertainty model parameter discretization
    % ----- beta -----
    Nbeta = 5; planner.Nbeta = Nbeta;
    Nbeta_DP = 3; planner.Nbeta_DP = Nbeta_DP;
    beta_vec = logspace(-1,1,Nbeta); planner.beta_vec = beta_vec;

    % ----- theta -----
    % Q1: lane changing to right, and lane keeping with cruising speed 
    % Q2: lane changing to left, and lane keeping with cruising speed
    % QH = theta*Q1 + (1-theta)*Q2
    Ntheta = 6; planner.Ntheta = Ntheta;
    Ntheta_DP = 3; planner.Ntheta_DP = Ntheta_DP;
    theta_vec = linspace(0,1,Ntheta); planner.theta_vec = theta_vec;

    % Human's tracking states
    xH_des_cell = {[nan; -planner.lc; planner.vH],... % Q1
                   [nan; planner.lc; planner.vH],...  % Q2
                   [nan; 0; planner.vH-2]};           % Unmodeled goal
    planner.xH_des_cell = xH_des_cell;

    % Initial joint distribution of beta and theta
    param_distr_init = (1/(Nbeta*Ntheta))*ones(Nbeta,Ntheta);
    planner.param_distr_init = param_distr_init;
    
    %% Vehicle discrete-time linearized model (for planning)
    % states = [p_x, p_y, v]; controls = [a, vLat]
    Ac = [0 0 1; 0 0 0; 0 0 -planner.params.talpha];
    Bc = [0 0; 0 1; 1 0];
    sysc = ss(Ac,Bc,eye(3),[]);
    sysd = c2d(sysc,planner.ts);
    Ad = sysd.A; planner.Ad = Ad;
    Bd = sysd.B; planner.Bd = Bd;
    
    %% Relative (joint) dynamics
    % states = [x_r, y_R, y_H, v_r]; controls = [a_R, vLat_R, a_H, vLat_H]
    AcRel = [0 0 0 1; 0 0 0 0; 0 0 0 0; 0 0 0 -planner.params.talpha];
    planner.AcRel = AcRel;
    BcRel = [0 0 0 0; 0 1 0 0; 0 0 0 1; 1 0 -1 0]; planner.BcRel = BcRel;
    syscRel = ss(AcRel,BcRel,eye(4),[]);
    sysdRel = c2d(syscRel,planner.ts);
    AdRel = sysdRel.A; planner.AdRel = AdRel;
    BdRel = sysdRel.B; planner.BdRel = BdRel;
    
    %% Partial deriv. for the shielding constraint
    pf4DpuR = [0 0; 0 1; 0 0; 1 0]; % partial(f4D)/partial(uR)
    planner.pf4DpuR = pf4DpuR;
    
    %% Lifting (change-of-coordinate) matrices
    planner = generateLiftingMatrices(planner);
    
    %% Robot's target states
    xR_plan_des = [1.2*planner.params.xr_tar_overtake; planner.lc;...
        planner.vDes];
    planner.xR_plan_des = xR_plan_des;

    %% Nominal MPC parameters
    % Cost matrices of the robot
    Q_R = blkdiag(1, 10, 5); planner.Q_R = Q_R;
    R_R = blkdiag(50, 50); planner.R_R = R_R;

    % Control bounds of the robot and human
    uub_R = [planner.params.accMax_R; planner.params.vLatMax_R];
    planner.uub_R = uub_R; 
    ulb_R = -uub_R;
    planner.ulb_R = ulb_R;
    uub_H = [planner.params.accMax_H; planner.params.vLatMax_H];
    planner.uub_H = uub_H;
    ulb_H = -uub_H;
    planner.ulb_H = ulb_H;

    % Velocity bounds
    vlb_R = 0;  planner.vlb_R = vlb_R;
    vub_R = 25; planner.vub_R = vub_R;
    vlb_H = 0;  planner.vlb_H = vlb_H;
    vub_H = 25; planner.vub_H = vub_H;

    % Human's cost matrices (learned from data)
    Q_H = blkdiag(0.1, 0.1, 0.001); planner.Q_H = Q_H;
    R_H = blkdiag(13.25, 13.25);    planner.R_H = R_H;

    % Compute DLQR-based cost-to-go
    [~,P_R,~] = dlqr(Ad, Bd, Q_R, R_R);
    % H regulates full states: p_x, p_y and v
    [~,P_H,~] = dlqr(Ad, Bd, Q_H, R_H);
    % H only regulates p_y and v
    [~,P_H_23,~] = dlqr(Ad(2:3,2:3), Bd(2:3,:), Q_H(2:3,2:3), R_H);
    % H only regulates p_y and v
    [~,P_H_3,~] = dlqr(Ad(3,3), Bd(3,1), Q_H(3,3), R_H(1,1));
    planner.P_R = P_R;
    planner.P_H = P_H;
    planner.P_H_23 = P_H_23;
    planner.P_H_3 = P_H_3;
    
    %% SHARP-SMPC parameters
    % - Shared parameters
    tree_params.Nnode = 100;
    tree_params.NuR = [7,7];
    tree_params.NuH = [5,5];
    tree_params.QMDP_K = 10; % unused
    tree_params.ts_params.Ns = 500;
    tree_params.ts_params.dpxR_sh = 0.5;
    tree_params.ts_params.dpyR_sh = 0.5;
    tree_params.ts_params.dvR_sh  = 0.5;
    tree_params.ts_params.Nmax_sh = 15;
    tree_params.ts_params.dpxR_nsh = 1.5;
    tree_params.ts_params.dpyR_nsh = 1.5;
    tree_params.ts_params.dvR_nsh  = 1.5;
    tree_params.ts_params.Nmax_nsh = 35;
    % - Dense tree parameters
    tree_params.Nbranch = 7;
    tree_params.Cmax = 15;
    tree_params.max_per_l = 25;
    tree_params.N_skip = 0;
    % - Sparse LQG tree parameters
    tree_params.N_max = 10; 
    tree_params.branch_count_max = 3;
    tree_params.cost_diff_pct_thresh = 0.03;
    tree_params.time_out = 1.0;
    planner.tree_params = tree_params;
end