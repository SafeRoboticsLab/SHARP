classdef node
    % Class for a node in the scenario tree
    %   Author: Haimin Hu
    %   Created: 2021-01-04, Last modified: 2021-09-06
    
    properties
        idx                 % Node index
        t                   % Time step
        xR                  % Robot's state
        xH                  % Human's state
        param_distr         % Joint distribution of beta and theta
        uH_MAP              % Human's MAP action
        uR_pre              % uR of the pre node
        uH_pre              % uH of the pre node
        Xf                  % Terminal set of states
        Vf                  % Terminal cost function
        lambda              % Multiplier for terminal set and cost
        IsLeaf              % Indicator of a leaf node
        IsShield            % Indicator of a shielding node
        IsOutOfGrid         % Indicator of an out-of-grid leaf node
        BranchCount         % Branching count
        path_prob           % Path transition probability of the node
        pre_node_idx        % Index of the pre node
        xR_opt              % Robot's state (yet to be optimized in SMPC)
        uR_opt              % Robot's control (yet to be optimized in SMPC)
        scenario            % 'OT' (overtake), 'IS' (intersection)
        extraArg            % Extra input arguments
    end
    
    methods
        function obj = node(idx, t, xR, xH, param_distr, uR_pre,...
                       uH_pre, path_prob, pre_node_idx, scenario, extraArg)
            % Constructor for the node
            obj.idx = idx;
            obj.t   = t;
         	obj.xR  = xR;
            obj.xH  = xH;
            obj.param_distr = param_distr;
            obj.uR_pre = uR_pre;
            obj.uH_pre = uH_pre;
            obj.IsLeaf = true;
            obj.IsShield = false;
            obj.IsOutOfGrid = false;
            obj.BranchCount = 0;
            obj.path_prob = path_prob;
            obj.pre_node_idx = pre_node_idx;
            if idx == 1
                obj.xR_opt = xR;
            else
                obj.xR_opt = [];
            end
            obj.uR_opt = [];
            obj.scenario = scenario;
            if nargin < 11
                obj.extraArg = [];
            else
                obj.extraArg = extraArg;
            end
        end
        
        
        function [xR_next, xH_next, param_distr_next] = propagate(obj,...
                uR, uH, planner, NuH)
            % Propagate the state and belief state for one time step
            if strcmp(obj.scenario,'OT') % For Scenario: Overtake
                xR_next = planner.Ad*obj.xR + planner.Bd*uR;
                xH_next = planner.Ad*obj.xH + planner.Bd*uH;
                param_distr_next = updateBetaTheta(obj.xH, uH, [], [],...
                    NuH, planner, obj.param_distr);
        	elseif strcmp(obj.scenario,'IS') % For Scenario: Intersection
                xR_next = planner.Ad*obj.xR + planner.Bd*uR;
                xH_next = planner.AHd*obj.xH + planner.BHd*uH;
                param_distr_next = updateBetaTheta(obj.xH, uH, [], [],...
                    NuH, planner, obj.param_distr);
            end
        end
        
        
        function [xR_next, xH_next] = propagate_state(obj, uR, uH, planner)
            % Propagate the state for one time step
            if strcmp(obj.scenario,'OT') % For Scenario: Overtake
                xR_next = planner.Ad*obj.xR + planner.Bd*uR;
                xH_next = planner.Ad*obj.xH + planner.Bd*uH;
        	elseif strcmp(obj.scenario,'IS') % For Scenario: Intersection
                xR_next = planner.Ad*obj.xR + planner.Bd*uR;
                xH_next = planner.AHd*obj.xH + planner.BHd*uH;
            end
        end
        
        
        function Vf = computeTerminalCost(obj, planner, QMDP, HJ, NuH)
            % Compute the terminal cost for a leaf node using QMDP
            %   TODO: Merge the two cases
            if strcmp(obj.scenario,'OT') % For Scenario: Overtake
                Vf = getQMDPvalue(obj.xR, obj.xH, obj.param_distr,...
                    planner, QMDP, 'MAP');
                % Keep applying shielding until we can find a QMDP value
                if isinf(Vf)
%                     warning(['Cannot compute Vf for node ',...
%                         num2str(obj.idx), '! Trying HJ...'])
                    Vf = 0;
                    Vf_QMDP = Inf;
                    node_tmp = obj;
                    timeout = 5;
                    time = 0;
                    while isinf(Vf_QMDP)
                        xRel = planner.E_6DtoSh*[node_tmp.xR; node_tmp.xH];
                        xR_plan = planner.E_6DtoRplan*[node_tmp.xR;...
                            node_tmp.xH];
                        deriv = eval_u(HJ.SafeSetData.grid,...
                            HJ.SafeSetData.Deriv, xRel);
                        uR_sh = HJ.SafeSetData.dynSys.optCtrl([], [],...
                            deriv, HJ.SafeSetData.uMode);
                        uR_sh = cell2mat(uR_sh);
                        Vf = Vf + uR_sh'*planner.R_R*uR_sh +...
                            (xR_plan-planner.xR_plan_des)'*planner.Q_R*...
                            (xR_plan-planner.xR_plan_des);
                        [node_tmp, ~] = node_tmp.branch(uR_sh, NuH,...
                            planner, 1);
                        Vf_QMDP = getQMDPvalue(node_tmp.xR, node_tmp.xH,...
                            node_tmp.param_distr, planner, QMDP, 'MAP');
                        time = time + 1;
                        if time >= timeout
                            xR_plan = planner.E_6DtoRplan*[node_tmp.xR;...
                                node_tmp.xH];
                            Vf_QMDP = (xR_plan-planner.xR_plan_des)'*...
                                planner.P_R*(xR_plan-planner.xR_plan_des);
                            warning(['Still cannot compute Vf for node ',...
                                num2str(obj.idx), '! Using the LQR cost.'])
                            break
                        end
                    end
                    Vf = Vf + Vf_QMDP;
                end
          	elseif strcmp(obj.scenario,'IS') % For Scenario: Intersection
            	Vf = getQMDPvalue(obj.xR, obj.xH, obj.param_distr,...
                    planner, QMDP, 'MAP');
                % Keep applying shielding until we can find a QMDP value
                if isinf(Vf)
%                     warning(['Cannot compute Vf for node ',...
%                         num2str(obj.idx), '! Trying HJ...'])
                    Vf = 0;
                    Vf_QMDP = Inf;
                    node_tmp = obj;
                    timeout = 5;
                    time = 0;
                    while isinf(Vf_QMDP)
                        xRel = planner.E_5Dto4D*[node_tmp.xR; node_tmp.xH];
                        xR_plan = planner.E_5DtoRplan*[node_tmp.xR;...
                            node_tmp.xH];
                        deriv = eval_u(HJ.SafeSetData.grid,...
                            HJ.SafeSetData.Deriv, xRel);
                        uR_sh = HJ.SafeSetData.dynSys.optCtrl([], [],...
                            deriv, HJ.SafeSetData.uMode);
                        uR_sh = cell2mat(uR_sh);
                        Vf = Vf + uR_sh'*planner.R_R*uR_sh +...
                            (xR_plan-planner.xR_plan_des)'*planner.Q_R*...
                            (xR_plan-planner.xR_plan_des);
                        [node_tmp, ~] = node_tmp.branch(uR_sh, NuH,...
                            planner, 1);
                        Vf_QMDP = getQMDPvalue(node_tmp.xR, node_tmp.xH,...
                            node_tmp.param_distr, planner, QMDP, 'MAP');
                        time = time + 1;
                        if time >= timeout
                            xR_plan = planner.E_5DtoRplan*[node_tmp.xR;...
                                node_tmp.xH];
                            Vf_QMDP = (xR_plan-planner.xR_plan_des)'*...
                                planner.P_R*(xR_plan-planner.xR_plan_des);
                            warning(['Still cannot compute Vf for node ',...
                                num2str(obj.idx), '! Using the LQR cost.'])
                            break
                        end
                    end
                    Vf = Vf + Vf_QMDP;
                end
            end
        end
        
        
        function obj = computeTerminalCostAndSet(obj, planner, HJ, QMDP,...
                                               NuR, NuH, QMDP_K, ts_params)
            % Construct data-dependent terminal cost and set by random
            % sampling around obj.xR
            
            % Parameters for sampling
            Ns = ts_params.Ns;
            % Case 1: shielding leaf node
            if obj.IsShield && obj.IsLeaf 
                dpxR = ts_params.dpxR_sh;
                dpyR = ts_params.dpyR_sh;
                dvR  = ts_params.dvR_sh;
                Nmax = ts_params.Nmax_sh;
            % Case 2: non-shielding leaf node
            elseif ~obj.IsShield && obj.IsLeaf 
                dpxR = ts_params.dpxR_nsh;
                dpyR = ts_params.dpyR_nsh;
                dvR  = ts_params.dvR_nsh;
                Nmax = ts_params.Nmax_nsh;
            % Sanity check
            else
                error(['Node ', num2str(obj.idx), ...
                    ' is neither a shielding node nor a leaf node.'])
            end
            
            % Define the deviation vector
            D = [dpxR; dpyR; dvR];
            
            % Initialize terminal cost and set vectors
            Vf_vec = obj.computeTerminalCost(planner, QMDP, HJ, NuH);
            Xf_vec = obj.xR;
            
            % Construct the terminal sets and costs
            n = 1;
            for i = 1:Ns
                if n >= Nmax
                    break
                end
                
                % Randomly sample a state in a neighbourhood of obj.xR
                xR_d = obj.xR + D.*(1-2*rand(3,1));  
                
                % Check sample status
                node_d = obj;
                node_d.xR = xR_d;
                [~, status_d] = node_d.shieldingCheck(planner, HJ, QMDP);
                
                if (strcmp(status_d,'shield') && obj.IsShield && obj.IsLeaf) ||... % case 1: shielding leaf node
                   (strcmp(status_d,'non-shield') && ~obj.IsShield && obj.IsLeaf)  % case 2: non-shielding leaf node
                    V_d = getQMDPvalue(node_d.xR, node_d.xH,...
                        node_d.param_distr, planner, QMDP, 'MAP');
                    if ~isinf(V_d)
                        Vf_vec = [Vf_vec V_d];
                        Xf_vec = [Xf_vec xR_d];
                        n = n + 1;
                    end
                end
            end
            
            % Store results
            if n < Nmax/2
                warning(['Insufficient terminal samples for node ',...
                    num2str(obj.idx), ', # = ', num2str(n)])
            end
            obj.Vf = Vf_vec;
            obj.Xf = Xf_vec;
            obj.lambda = sdpvar(1, length(Vf_vec));
        end
        
        
        function [obj, status] = shieldingCheck(obj, planner, HJ, QMDP)
            % Check node status (shield/non-shield/unsafe/not-in-grid)
            
            if strcmp(obj.scenario,'OT') % For Scenario: Overtake
                xRel = planner.E_6DtoSh*[obj.xR;obj.xH];
            elseif strcmp(obj.scenario,'IS') % For Scenario: Intersection
                xRel = planner.E_5Dto4D*[obj.xR;obj.xH];
            end

            uR = obj.getQMDPcontrol(planner, QMDP, 1);
            % Current state not in the QMDP grid - mark as shielding node
            if isnan(uR) 
                obj.IsShield = true;
                obj.IsOutOfGrid = true;
                status = 'not-in-grid';
                return
            else
                value_BRS = eval_u(HJ.BRSData.grid, HJ.BRS, xRel);
            end

            value_SS  = eval_u(HJ.SafeSetData.grid, HJ.SafeSet, xRel);
            % Current state requires shielding - mark as shielding node
            if value_BRS <= 0
                obj.IsShield = true;
                status = 'shield';
            % Current state is unsafe - mark as shielding node
            elseif value_SS > 0
                obj.IsShield = true;
                obj.IsOutOfGrid = true;
                status = 'unsafe';
            % Current state does not require shielding
            else
                status = 'non-shield';
            end
        end
        
        
        function uR = getQMDPcontrol(obj, planner, QMDP, isProj)
            % Compute QMDP policy for the node
            if nargin < 4
                isProj = false;
            end
            % Get QMDP control
            uR = getQMDPcontrol_lookup(obj.xR, obj.xH, obj.param_distr,...
                planner, QMDP, isProj);
        end
        
        
        function [node_vec, P_vec] = branch(obj, uR, NuH, planner, Nbranch)
            % Branch this node to obtain descendants
            node_vec = [];
            
            % uH discretization
            uH1_vec = linspace(planner.ulb_H(1),planner.uub_H(1),NuH(1));
            uH2_vec = linspace(planner.ulb_H(2),planner.uub_H(2),NuH(2));

            % Dimension of the joint uncertainties [beta theta uH_1 uH_2]
            Nd = [planner.Nbeta, planner.Ntheta, NuH(1), NuH(2)];
            if Nbranch > prod(Nd)
                Nbranch = prod(Nd);
            end

            % Compute the joint probability
            P_jnt = NaN(Nd);
            for i = 1:planner.Nbeta
                for j = 1:planner.Ntheta
                    P_uH = computeBoltzmannFull(obj.xH, [], [],...
                        uH1_vec, uH2_vec, planner, planner.beta_vec(i),...
                        planner.theta_vec(j));
                    P_jnt(i,j,:,:) =...
                        obj.path_prob*obj.param_distr(i,j)*P_uH;
                end
            end

            % Pick the Nbranch-largest probabilities
            [P_vec,I] = maxk(P_jnt(:),Nbranch);
            [~,~,IuH1,IuH2] = ind2sub(Nd,I);

            for i = 1:Nbranch
                % Propagate (belief) states
                uH = [uH1_vec(IuH1(i)); uH2_vec(IuH2(i))];
                [xR_next, xH_next, param_distr_next] =...
                    obj.propagate(uR, uH, planner, NuH);
                % Update time
                t_next = obj.t + 1;
                % Create new node
                candidate_node = node([], t_next, xR_next, xH_next,...
                    param_distr_next, uR, uH, P_vec(i), obj.idx,...
                    obj.scenario);
                % Update the descendant node vector
                node_vec = [node_vec candidate_node];
            end
            P_vec = P_vec';
        end
        
        
        function node_extend = extend(obj, uR, theta_MAP, planner, NuH)
            % Extend this node to obtain descendants (without branching)
            
            if nargin < 5
                NuH = [5;5];
            end
            
            % Compute rational (most likey) uH
            uH = computeRationalHumanAction(obj.xH, [], [], planner,...
                theta_MAP);
            uH = min(max(uH, planner.ulb_H), planner.uub_H);
            % Compute next states
            [xR_next, xH_next] = obj.propagate_state(uR, uH, planner);
            param_distr_next = obj.param_distr;
%             [xR_next, xH_next, param_distr_next] = obj.propagate(uR, uH,...
%                 planner, NuH);
            % Update time
            t_next = obj.t + 1;
            % Create a new node (keeping path_prob, param_distr unchanged)
            node_extend = node([], t_next, xR_next, xH_next,...
                param_distr_next, uR, uH, obj.path_prob, obj.idx,...
                obj.scenario);
        end
        
        
        function theta_MAP = computeThetaMAP(obj, planner)
            % Compute an MAP value of theta
            [~,I] = maxk(obj.param_distr(:),1);
            [~,Itheta] = ind2sub([planner.Nbeta, planner.Ntheta],I);
            theta_MAP = planner.theta_vec(Itheta);
        end
        
        
        function obj = init_YALMIP_variables(obj, nx, nu)
            % Initialize decesion variables
            if obj.idx ~= 1
                obj.xR_opt = sdpvar(nx,1);
            end
            obj.uR_opt = sdpvar(nu,1);
        end
    end % end methods
end % end class