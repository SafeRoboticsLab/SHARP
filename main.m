% Main script of SHARP: Shielding-Aware Robust Planning for safe and
% efficient human-robot interaction
close all; clear all; clc
addpath('./util')
addpath('./data')
addpath('./car_figures')
load HJ_precomp
load QMDP_precomp
load human_lane_keeping
load all_states_cell
load all_controls_cell
load all_headings_cell


%% Problem paramters
ts = 0.2; planner.ts = ts;                  % Sampling time
Nmpc = 5;                                   % MPC prediction horizon
vH = 12.5; planner.vH = vH;                 % Human's nominal velocity                 
vDes = planner.vH + 3; planner.vDes = vDes; % Robot's desired velocity
lc = params.rd_bd_max/2; planner.lc = lc;   % lane center position
planner.params = params;                    % Shielding design parameters
planner.MPC_human = MPC_human;              % Human's lane keeping policy
planner = initializePlanner(planner);       % Initialize the planner object


%% Main planning loop

% ================ Choose your planner ================
method = 'SMPC'; % 'SMPC', 'QMDP', 'bMPC'
% =====================================================

% Performance metrics
J_em_vec    = [];     % Vector of empirical costs
sh_freq_vec = [];     % Vector of shielding frequencies

for i = 1:length(all_states_cell)
    rng('default')
    % Human's trajectory
    XH = all_states_cell{i};
    UH = all_controls_cell{i};
    HH = all_headings_cell{i};
    XH = XH(:,1:length(UH));
    HH = HH(:,1:length(UH));
    T_data = size(UH,2);

    % Initial states
    xR_init = [-12; -lc; XH(3,1)];
    xH_init = XH(:,1);
    xRel_init = planner.E_6DtoSh*[xR_init; xH_init];

    % Initialize simulation
    xR = xR_init;
    xH = xH_init;
    xRel = xRel_init;
    param_distr = planner.param_distr_init;
    uH = UH(:,1);

    % Initialize trajectory vectors
    XR = xR;
    UR = [];
    XRel = xRel;
    BETA  = [];
    THETA = [];
    PARAM_DISTR = {param_distr};
    mode = {};  % Indicates which policy is used at each time

    % Empirical cost of the robot
    J_em = 0;

    % Shielding count
    sh_cnt = 0;

    % Main loop
    t = 1;
    while (XRel(1,t) <= params.xr_tar_overtake)
        disp(['simulation time: t = ', num2str(t)])

        % ========== Robot Block ==========
        % Threshold for using the nominal MPC policy
        disengage_pxR_ub = 2;
        
        % Decide which QMDP policy to use
        QMDP = scheduleQMDP(xH, uH, QMDP_left, QMDP_right);

        % Obtain a nominal policy
        if xRel(1) <= disengage_pxR_ub && strcmp(method,'QMDP')
            % ------ SHARP-QMDP -------------------------------------------
            uR = getQMDPcontrol_lookup(xR, xH, param_distr, planner, QMDP);
            if ~isnan(uR(1))
                mode{end+1} = 'QMDP';
            end
        elseif xRel(1) <= disengage_pxR_ub && strcmp(method,'SMPC')
            % ------ SHARP-SMPC -------------------------------------------
            verbose = true;
            [uR, ~] = getSMPCcontrol(xR, xH, param_distr, HJ, planner,...
                QMDP, planner.tree_params, verbose);
            if ~isnan(uR(1))
                mode{end+1} = 'SMPC';
            end
        else
            % ------ Nominal MPC ------------------------------------------
            uR = NaN(2,1); 
        end

        % Plan with nominal MPC if SHARP planners cannot give a solution
        if isnan(uR(1)) 
            xR_des_MPC = [getTargetLgtPos(xH, vH, ts*Nmpc, NaN, params);
                lc; planner.xR_plan_des(3)];
            [~, uMPC_R, ~, ~] = nominalMPC(xR, xR_des_MPC, Nmpc,...
                planner.Ad, planner.Bd, planner.Q_R, planner.R_R,...
                planner.ulb_R, planner.uub_R,...
                planner.vlb_R, planner.vub_R, planner);
            uR = uMPC_R(:,1);
            mode{end+1} = 'bMPC';
        end

        % ------- Shielding: Override with safe policy if necessary -------
        value_BRS = eval_u(HJ.BRSData.grid, HJ.BRS, xRel);
        value_SS  = eval_u(HJ.SafeSetData.grid, HJ.SafeSet, xRel);
        if value_BRS <= 0 || value_SS > 0
            uR = computeShielding(xRel, HJ);
            mode{end} = 'shielding';
            if xRel(1) <= disengage_pxR_ub
                sh_cnt = sh_cnt + 1;
            end
        end
        % -----------------------------------------------------------------

        % ========== Human Block ==========
        % Estimate human's costs
        if t > 1 && ~strcmp(mode{end},'bMPC')
            [Q_H_est, R_H_est] = determineHumanUtility(XH(:,1:t-1),...
                UH(:,1:t-1), planner);
            planner.Q_H = Q_H_est;
            planner.R_H = R_H_est;
        end
        
        % Observe human's action
        if t > T_data
            % Extend data with a lane keeping policy
            EMPC_H_m1 = MPC_human.mode1.feval(xH, 'primal');
            uH = [EMPC_H_m1(1,1); EMPC_H_m1(2,1)];
            uH = max(min(uH,planner.uub_H),planner.ulb_H);
            uH_now = uH;
        else
            uH = UH(:,t);
            uH_now = uH;
        end

        % ========== Update beta and theta ==========
        uH_now = max(min(uH_now,planner.uub_H),planner.ulb_H);
        param_distr = updateBetaTheta(xH, uH_now, [], [],...
            [11;11], planner, param_distr);
        % Determine current value of beta and theta with MAP
        [~, param_idx] = max(param_distr(:));
        [idx1,idx2] = ind2sub([planner.Nbeta,planner.Ntheta],param_idx);
        beta  = planner.beta_vec(idx1);
        BETA  = [BETA beta];
        theta = planner.theta_vec(idx2);
        THETA = [THETA theta];
        PARAM_DISTR{end+1} = param_distr;

        % ========== Update States ==========
        % Robot: evolve system
        xR = kinematicBicycle(xR, uR, ts, params);
        
        % Human: evolve system
        if t >= T_data
            xH = kinematicBicycle(xH, uH, ts, params);
        else
            xH = XH(:,t+1);
        end

        % Update relative states
        xRel = planner.E_6DtoSh*[xR;xH];

        % Store results
        XR = [XR xR];
        UR = [UR uR];
        if t >= T_data
            XH = [XH xH];
            HH = [HH asin(max(min(uH(2)/xH(3),1),-1))];
        end
        if t > T_data
            UH = [UH uH];
        end
        XRel = [XRel xRel];

        % Update the empirical cost
        J_em = updateEmpiricalCost(planner, J_em, xR, xH, uR);
        
        % Update time
        t = t + 1;
        if t > 180
            break
        end
    end
    disp(['Simulation finished! Empirical cost = ', num2str(J_em),...
          ', shielding count = ', num2str(sh_cnt)])  

    if XRel(1,t) >= 0.95*params.xr_tar_overtake % Target reached
        J_em_vec    = [J_em_vec J_em];
        sh_freq_vec = [sh_freq_vec sh_cnt/t];
    else
        J_em_vec    = [J_em_vec NaN];
        sh_freq_vec = [sh_freq_vec NaN];
    end

end


%% Visualization
option.car_plot   = 'fancy'; % 'fancy', 'casual'
option.coordinate = 'rel';   % 'rel', 'abs'
option.plot_ls    = 1;
option.keep_traj  = 1;
option.t_skip     = 1;
option.t_start    = [];
option.t_end      = [];
option.pause      = 0.0;
plotSim(HJ, planner, XR, UR, XH, HH, UH, XRel, BETA, THETA, mode, option)