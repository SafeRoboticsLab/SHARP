classdef tree
    % Class for a scenario tree
    %   Author: Haimin Hu
    %   Created: 2021-01-04, Last modified: 2021-09-06
    
    properties
        % Sets of indices
        Layer       % Index set of nodes in each layer
        Branch      % Index set of nodes in each branch
        L           % Index set of leaf nodes
        Ls          % Index set of shielding leaf nodes
        
        % Sets of nodes
        N           % Node set
        Ns          % Shielding node set (for constructSparseShield only)
        C           % Candidate set
        
        % Sets of probabilities
        P_vec_N     % Path transition probabilities of the nodes in N
        P_vec_C     % Path transition probabilities of the nodes in C
        
        % Lookup tables
        HJ          % HJ pre-computation
        QMDP        % QMDP pre-computation
        
        % Parameters
        % - Shared
        QMDP_K      % Number of samples for computing QMDP policy (unused)
        planner     % The planner object
        NuR         % uR discretization resolution (row vector)
        NuH         % uH discretization resolution (row vector)
        ts_params   % Terminal sets parameters
        layer_max   % Max number of layers (t=0 is Layer #1)
        % - Dense tree [Bernardini and Bemporad]
        Nbranch     % Max number of scenarios branched from a parental node 
                    % (when updating the candidate set)
        Cmax        % Max number of nodes in the candidate set
        max_per_l   % Max number of nodes in each layer
        N_skip      % Number of time steps skipped before branching
        % - Sparse LQG tree
        branch_count_max        % Max number of scenarios branched from a 
                                % parental node
        cost_diff_pct_thresh    % Min pecentage of cost difference when 
                                % adding a new branch
        time_out                % Max compuation time for tree construction
    end
    
    methods
        function obj = tree(root_node, HJ, QMDP, planner, Cmax,...
                  max_per_l, Nbranch, NuR, NuH, QMDP_K, ts_params,...
                  N_skip, branch_count_max, cost_diff_pct_thresh, time_out)
            % Constructor for the scenario tree
            obj.Branch = {};
            obj.Layer = cell(1,100);  % pre-allocated cell size
            obj.Layer{1} = 1;
            if isnan(root_node.idx)
                root_node.idx = 1;
            end
            if root_node.t ~= 0
                root_node.t = 0;
            end
            obj = obj.addNewNode(root_node);
            obj.HJ      = HJ;
            obj.QMDP    = QMDP;
            obj.planner = planner;
            obj.NuR  = NuR;
            obj.NuH  = NuH;
            obj.Cmax = Cmax;
            obj.QMDP_K = QMDP_K;
            obj.Nbranch = Nbranch;
            obj.max_per_l = max_per_l;
            obj.ts_params = ts_params;
            obj.N_skip    = N_skip;
            obj.layer_max = 1;
            obj.branch_count_max = branch_count_max;
            obj.cost_diff_pct_thresh = cost_diff_pct_thresh;
            obj.time_out = time_out;
        end
        
        
        function obj = constructInit(obj)
            % Initialize tree construction
            
            % Root node
            root_node = obj.N(1);
            
            % Shielding check
            [root_node, status] = root_node.shieldingCheck(obj.planner,...
                obj.HJ, obj.QMDP);
            
            % Case 1: Current state in B^R, mark as shielding node
            if strcmp(status,'shield')
                obj.Ls = [obj.Ls root_node.idx];
                obj.N(1).IsShield = true;
            % Case 2: Current state is unsafe - mark as shielding node
            elseif strcmp(status,'unsafe')
                obj.Ls = [obj.Ls root_node.idx];
                obj.N(1).IsShield = true;
                warning('Root node is unsafe! Check for numerical issue or bugs.')
            % Case 3: Current state is not in the QMDP grid - mark as shielding node
            elseif strcmp(status,'not-in-grid')
                obj.Ls = [obj.Ls root_node.idx];
                obj.N(1).IsShield = true;
                warning('Root node is not in the QMDP gird. Mark as a shielding node.')
            % Case 4: Current state does not require shielding
            elseif strcmp(status,'non-shield')
                isProj = 1;
                uR = root_node.getQMDPcontrol(obj.planner, obj.QMDP, isProj);
                obj = obj.updateCandidateSet(root_node, uR);
            else
                error('Invalid status of the root node! Check for bugs!')
            end
        end
        
        
        function obj = addNewNode(obj, node)
            % Add a new node to the tree
            obj.N = [obj.N node];
            obj.P_vec_N = [obj.P_vec_N node.path_prob];
            
            % Sanity check
            if length(obj.N) ~= node.idx
                error('Node index incorrect! Check for bugs!')
            end
        end
        
        
        function obj = updateCandidateSet(obj, node, uR)
            % Update the candidate set using the given node

            if isnan(uR(1))
                error('uR is NaN. Cannot update candidate set!')
            end
            
            % Branch the given node
%             Nbranch = obj.Cmax - length(obj.C);
            [node_vec, P_vec] = node.branch(uR, obj.NuH, obj.planner,...
                obj.Nbranch);

            % Update the candidate set
            obj.C = [obj.C node_vec];
            obj.P_vec_C = [obj.P_vec_C P_vec];
            [obj.P_vec_C, idx] = maxk(obj.P_vec_C, obj.Cmax);
            obj.C = obj.C(idx);
        end
        
        
        function obj = removeCandidates(obj, layer_num)
            % Remove nodes in the candidate set with layer = layer_num
            idx_set = [];
            for i = 1:length(obj.C)
                if obj.C(i).t+1 == layer_num
                    idx_set = [idx_set i];
                end
            end
            obj.C(idx_set) = [];
            obj.P_vec_C(idx_set) = [];
        end
        
        
        function obj = normalizePathProb(obj)
            % After the tree is constructed, normalize path transition
            % probabilities for nodes in each layer
            for l = 1:obj.layer_max
                P_sum = 0;
                for idx = obj.Layer{l}
                    P_sum = P_sum + obj.N(idx).path_prob;
                end
                for idx = obj.Layer{l}
                    obj.N(idx).path_prob = (1/P_sum)*obj.N(idx).path_prob;
                    obj.P_vec_N(idx) = obj.N(idx).path_prob;
                end
            end
        end
        
        
        function obj = getLeafSet(obj, verbose)
            % Update the leaf set
            
            % Initialize the set to avoid repeated nodes
            obj.L = [];
            
            if verbose
                disp('Computing terminal costs and sets...')
            end
            
            % Find leaf nodes and compute their terminal costs
            for i = 1:length(obj.N)
                if obj.N(i).IsLeaf
                    obj.N(i) = obj.N(i).computeTerminalCostAndSet(...
                        obj.planner, obj.HJ, obj.QMDP, obj.NuR, obj.NuH,...
                        obj.QMDP_K, obj.ts_params);        
                    obj.L = [obj.L obj.N(i).idx];
                end
                
%                 % Sanity check
%                 if obj.N(i).IsShield && ~obj.N(i).IsLeaf
%                     error(['node ', num2str(obj.N(i).idx),...
%                         ' is a shielding node but not a leaf node. Check for bugs!'])
%                 end
            end
        end
        
        
        function cost = computeBranchCost(obj, branch)
            % Compute the accumulated cost along the given branch
            cost = 0;
            % Start with the second node
            for i = 2:length(branch.idx_set) 
                node = obj.N(branch.idx_set(i));
                % The terminal node uses QMDP value
                if i == length(branch.idx_set)
                    Vf = getQMDPvalue(node.xR, node.xH,...
                        node.param_distr, obj.planner, obj.QMDP, 'MAP');
                    cost = cost + Vf;
                else
                    cost = updateEmpiricalCost(obj.planner, cost,...
                                            node.xR, node.xH, node.uR_pre);
                end
            end
        end
        
        
        function cost = computeNodeCost(obj, node)
            % Compute the accumulated cost associated with the given node
            
            cost = 0;
            
            % Backtrack costs of all parents
            pre_node_idx = node.pre_node_idx;
            while true
                if isnan(pre_node_idx)
                    break
                end
                pre_node = obj.N(pre_node_idx);
                if isnan(pre_node.uR_pre)
                    break
                end
                cost = updateEmpiricalCost(obj.planner, cost,...
                                pre_node.xR, pre_node.xH, pre_node.uR_pre);
                pre_node_idx = obj.N(pre_node_idx).pre_node_idx;
            end
            
            % Add QMDP cost-to-go
            Vf = getQMDPvalue(node.xR, node.xH, node.param_distr,...
                                      obj.planner, obj.QMDP, 'MAP');
            cost = cost + Vf;
        end
        
        
        function flag = IsDistinctScenario(obj, cost)
            % Determine if the given cost is sufficiently different from
            % costs of other branches
            flag = true;
            for i = 1:length(obj.Branch)
                cost_diff_pct = ...
                    abs((cost-obj.Branch{i}.cost)/obj.Branch{i}.cost);
                if cost_diff_pct < obj.cost_diff_pct_thresh
                    flag = false;
                    return
                end
            end
        end
        
        
        function [start_node, flag] = findStartNode(obj)
            % Find a new node to start a new branch 
            tStart = toc;
            
            flag = true;
            
            uH1_vec = linspace(obj.planner.ulb_H(1), ...
                obj.planner.uub_H(1), obj.NuH(1));
            uH2_vec = linspace(obj.planner.ulb_H(2), ...
                obj.planner.uub_H(2), obj.NuH(2));
            
            for l = 1:obj.layer_max % Prioritize smaller layer
              for idx = obj.Layer{l}
                candidate_node = obj.N(idx);
                if (candidate_node.BranchCount < obj.branch_count_max)...
                        && ~candidate_node.IsOutOfGrid
                  uR = candidate_node.getQMDPcontrol(obj.planner,...
                      obj.QMDP, 1);
                  if isnan(uR)
                      continue
                  end

                  for uH1 = uH1_vec
                    for uH2 = uH2_vec
                      % Find a deviated human control
                      uH = [uH1; uH2];
                      [xR_next, xH_next, param_distr_next] =...
                          candidate_node.propagate(uR, uH, obj.planner,...
                          obj.NuH);
                      % Update time
                      t_next = candidate_node.t + 1;
                      % Create a new node
                      next_node = node([], t_next, xR_next, xH_next,...
                        param_distr_next, uR, uH, NaN,...
                        candidate_node.idx, candidate_node.scenario);
                      % Compute the cost
                      cost = obj.computeNodeCost(next_node);
                      if isnan(cost)
                          continue
                      end
                      % If the candidate_node leads to a distinct scenario, 
                      % then use it the as the next start_node
                      if obj.IsDistinctScenario(cost)
                          candidate_node.param_distr =...
                              next_node.param_distr;
                          start_node = candidate_node;
                          return
                      end
                      
                      tEnd = toc;
                      if tEnd - tStart > obj.time_out
                          start_node = NaN;
                          flag = false;
                          return
                      end
                    end
                  end
                end
              end
            end
            start_node = NaN;
            flag = false;
        end
        
        
        function obj = constructDense(obj, M, verbose)
            % Construct a dense scenario tree with a maximum of M nodes
            % using the method proposed in [Bernardini and Bemporad]. The
            % shielding nodes are considered as terminal nodes.
            
            % Initialize tree construction
            obj = obj.constructInit();
           
            % Initialize node counter (root node is already in the tree)
            m = 2;
            
            % Main loop
            % ------- Begin tree construction -------
            tic
            if verbose
                disp('Constructing the scenario tree...')
            end
            while m<=M && ~isempty(obj.C)
%                 if verbose
%                     disp(['Tree construction progress: m = ',...
%                         num2str(m), '/', num2str(M)])
%                 end
                
                % Pick the node in C with the greatest path transition
                % probability
                [~, idx] = maxk(obj.P_vec_C,1);
                node_elect = obj.C(idx);
                
                % Index the elected node
                node_elect.idx = m;
                
                % Remove the elected node from C
                obj.C(idx) = [];
                obj.P_vec_C(idx) = [];
                
                % Get MAP estimate of theta (can be used across all i =
                % 0:obj.N_skip since param_distr does not change)
                theta_MAP = node_elect.computeThetaMAP(obj.planner);
                
                % Check shielding, extend the tree and update C
                break_flag = false;
                for i = 0:obj.N_skip
                    if i == 0
                        node_extend = node_elect;
                    else
                        % Extend the tree without branching
                        uR = node_extend.getQMDPcontrol(obj.planner,...
                            obj.QMDP, 1);
                        node_extend = node_extend.extend(uR, theta_MAP,...
                            obj.planner, obj.NuH);
                        node_extend.idx = m;
                    end
                    
                    % Shielding check
                    [node_extend, status] = node_extend.shieldingCheck(...
                        obj.planner, obj.HJ, obj.QMDP);
                    % Case 1: Current state in B^R - mark as shielding node
                    if strcmp(status,'shield')
                        obj.Ls = [obj.Ls node_extend.idx];
                        break_flag = true;
                    % Case 2: Current state is unsafe - mark as shielding node
                    elseif strcmp(status,'unsafe')
                        obj.Ls = [obj.Ls node_extend.idx];
%                         warning(['Node ', num2str(node_extend.idx),...
%                             ' is unsafe! Check for numerical issues or bugs. Layer = ',...
%                             num2str(node_elect.t)])
                        break_flag = true;
                    % Case 3: Current state is not in the QMDP grid - mark as shielding node
                    elseif strcmp(status,'not-in-grid')
                        obj.Ls = [obj.Ls node_elect.idx];
%                         warning(['Node ', num2str(node_elect.idx),...
%                             ' is not in the QMDP gird. Mark as a shielding node. Layer = ',...
%                             num2str(node_elect.t)])
                        break_flag = true;
                    % Case 4: Current state does not require shielding
                    elseif strcmp(status,'non-shield')
                        % Update the candidate set for the next branching
                        if i == obj.N_skip
                            isProj = 1;
                            uR = node_extend.getQMDPcontrol(obj.planner,...
                                obj.QMDP, isProj);
                            obj = obj.updateCandidateSet(node_extend, uR);
                        end
                    else
                        error('Invalid status of the elected node!')
                    end
                    
                    % Add the extended node to the tree
                    obj = obj.addNewNode(node_extend);
                    obj.N(node_extend.pre_node_idx).IsLeaf = false;
                    m = m + 1;

                    % Append the index of the extended node to Layers
                    layer_idx = node_extend.t+1;
                    obj.Layer{layer_idx} = [obj.Layer{layer_idx}...
                        node_extend.idx];
                    
                    % Update layer_max if necessary
                    if node_extend.t+1 > obj.layer_max
                        obj.layer_max = node_extend.t+1;
                    end
                    
                    % Remove nodes in C if necessary
                    for l = 1:obj.layer_max
                        if length(obj.Layer{l}) >= obj.max_per_l
                            obj = obj.removeCandidates(l);
                        end
                    end
                    
                    if m>M || break_flag
                        break
                    end
                end              
            end
            toc
            % ------- End tree construction -------
            
            if length(obj.N) > 1
                % Normalize the probabilities of nodes in each layer such
                % that they sum up to 1
                obj = obj.normalizePathProb();

                % Identify the leaf set and compute terminal sets & costs
                % for leaf nodes
                obj = obj.getLeafSet(verbose);
            end
            
            % Initialize decision variables
            for i = 1:length(obj.N)
                nx = size(obj.planner.Bd,1);
                nu = size(obj.planner.Bd,2);
            	obj.N(i) = obj.N(i).init_YALMIP_variables(nx, nu);
            end
            
            % Report construction results
            if verbose
                disp('Tree construction completed!')
                for l = 1:obj.layer_max
                    disp(['Layer #', num2str(l), ': ',...
                        num2str(length(obj.Layer{l})), ' node(s)'])
                end
                disp(['Number of leaf node(s) in L: ',...
                    num2str(length(obj.L))])
                disp(['Number of shielding leaf node(s) in Ls: ',...
                    num2str(length(obj.Ls))])
            end
        end
        
        
        function obj = constructSparse(obj, M, N_max, verbose)
            % Construct a sparse LQG scenario tree with a maximum of M
            % nodes and a maximum of N_max layers.
            
            % Initialize tree construction
            obj = obj.constructInit();
            
            % Initialize node counter (root node is already in the tree)
            m = 2;
            
            % Main loop
            % ------- Begin tree construction -------
            tic
            if verbose
                disp('Constructing the scenario tree...')
            end
            start_node = obj.N(1);
            while m<=M && ~obj.N(1).IsShield
%                 if verbose
%                     disp(['Tree construction progress: m = ',...
%                         num2str(m), '/', num2str(M)])
%                 end
                
                % Initialize a new branch
                branch.idx_set = start_node.idx;
                if ~isnan(start_node.pre_node_idx)
                    % Add all parents of start_node to the branch
                    idx_tmp = start_node.idx;
                    while true
                        idx_tmp = obj.N(idx_tmp).pre_node_idx;
                        branch.idx_set = [idx_tmp branch.idx_set];
                        if isnan(obj.N(idx_tmp).pre_node_idx)
                            break
                        end
                    end
                end
                
                % Get MAP estimate of theta (can be used across all future
                % time steps in forward simulations since param_distr does
                % not change). Alternatively we can compute the expectation
                % of costs
                theta_MAP = start_node.computeThetaMAP(obj.planner);
                
                % Forward simulation
                current_node = start_node;
                break_flag = false;
                for t = start_node.t:N_max-1
                    % Extend the tree without branching
                  	uR = current_node.getQMDPcontrol(obj.planner,...
                        obj.QMDP, 1);
                    next_node = current_node.extend(uR, theta_MAP,...
                        obj.planner, obj.NuH);
                    next_node.idx = m;
                    
                    % Shielding check
                    [next_node, status] = next_node.shieldingCheck(...
                        obj.planner, obj.HJ, obj.QMDP);
                    % Case 1: Current state in B^R - mark as shielding node
                    if strcmp(status,'shield')  
                        next_node.IsShield = false;
                    % Case 2: Current state is unsafe - mark as shielding node
                    elseif strcmp(status,'unsafe')
                        obj.Ls = [obj.Ls next_node.idx];
%                         warning(['Node ', num2str(next_node.idx),...
%                             ' is unsafe! Check for numerical issues or bugs. Layer = ',...
%                             num2str(next_node.t)])
                        break_flag = true;
                    % Case 3: Current state is not in the QMDP grid - mark as shielding node
                    elseif strcmp(status,'not-in-grid')
                        obj.Ls = [obj.Ls next_node.idx];
%                         warning(['Node ', num2str(next_node.idx),...
%                             ' is not in the QMDP gird. Mark as a shielding node. Layer = ',...
%                             num2str(next_node.t)])
                        break_flag = true;
                    elseif ~strcmp(status,'non-shield')
                        error('Invalid status of the elected node!')
                    end
                    
                    % Add the new node to the tree
                    obj = obj.addNewNode(next_node);
                    obj.N(next_node.pre_node_idx).IsLeaf = false;
                    m = m + 1;
                    
                    % Append the index of the new node to Layers 
                    layer_idx = next_node.t+1;
                    obj.Layer{layer_idx} = [obj.Layer{layer_idx}...
                        next_node.idx];
                    
                    % Update layer_max if necessary
                    if next_node.t+1 > obj.layer_max
                        obj.layer_max = next_node.t+1;
                    end
                    
                    % Append the index of the new node to the branch
                    branch.idx_set = [branch.idx_set next_node.idx];
                    
                    if m>M || break_flag
                        break
                    end
                    
                	current_node = next_node;
                end
                
                % Compute cost alone the branch
                branch.cost = obj.computeBranchCost(branch);
                
                % Add the branch to the cell
                obj.Branch{end+1} = branch;
                
                % Find a new node to start a new branch
                [start_node, success_flag] = obj.findStartNode();
                if ~success_flag
                    break
                end
                
                % Update branch count of the start_node
                obj.N(start_node.idx).BranchCount =...
                    obj.N(start_node.idx).BranchCount + 1;
                
                tree_construct_time = toc;
                if tree_construct_time > obj.time_out
                    break
                end
            end
            toc
            % ------- End tree construction -------
            
            if length(obj.N) > 1
                % Normalize the probabilities of nodes in each layer such
                % that they sum up to 1
                obj = obj.normalizePathProb();

                % Identify the leaf set and compute terminal sets & costs
                % for leaf nodes
                obj = obj.getLeafSet(verbose);
            end
            
            % Initialize decision variables
            for i = 1:length(obj.N)
                nx = size(obj.planner.Bd,1);
                nu = size(obj.planner.Bd,2);
            	obj.N(i) = obj.N(i).init_YALMIP_variables(nx, nu);
            end
            
            % Report construction results
            if verbose
                disp('Tree construction completed!')
                for l = 1:obj.layer_max
                    disp(['Layer #', num2str(l), ': ',...
                        num2str(length(obj.Layer{l})), ' node(s)'])
                end
                disp(['Number of leaf node(s) in L: ',...
                    num2str(length(obj.L))])
                disp(['Number of shielding leaf node(s) in Ls: ',...
                    num2str(length(obj.Ls))])
            end
        end
        
        
        function obj = constructSparseShield(obj, M, N_max, verbose)
            % Construct a sparse LQG scenario tree with a maximum of M
            % nodes and a maximum of N_max layers. This tree allows for
            % incorporation of convex CBF-based shielding constraint.
            
            % Initialize tree construction
            obj = obj.constructInit();
            
            % Initialize node counter (root node is already in the tree)
            m = 2;
            
            % Main loop
            % ------- Begin tree construction -------
            tic
            if verbose
                disp('Constructing the scenario tree...')
            end
            start_node = obj.N(1);
            while m<=M && ~obj.N(1).IsShield
%                 if verbose
%                     disp(['Tree construction progress: m = ',...
%                         num2str(m), '/', num2str(M)])
%                 end
                
                % Initialize a new branch
                branch.idx_set = start_node.idx;
                if ~isnan(start_node.pre_node_idx)
                    % Add all parents of start_node to the branch
                    idx_tmp = start_node.idx;
                    while true
                        idx_tmp = obj.N(idx_tmp).pre_node_idx;
                        branch.idx_set = [idx_tmp branch.idx_set];
                        if isnan(obj.N(idx_tmp).pre_node_idx)
                            break
                        end
                    end
                end
                
                % Get MAP estimate of theta (can be used across all future
                % time steps in forward simulations since param_distr does
                % not change). Alternatively we can compute the expectation
                % of costs
                theta_MAP = start_node.computeThetaMAP(obj.planner);
                
                % Forward simulation
                current_node = start_node;
                break_flag = false;
                for t = start_node.t:N_max-1
                    % Extend the tree without branching
                  	uR = current_node.getQMDPcontrol(obj.planner,...
                        obj.QMDP, 1);
                    next_node = current_node.extend(uR, theta_MAP,...
                        obj.planner, obj.NuH);
                    next_node.idx = m;

                    % Shielding check
                    [next_node, status] = next_node.shieldingCheck(...
                        obj.planner, obj.HJ, obj.QMDP);
                    % Case 1: Current state in B^R - mark as shielding node
                    if strcmp(status,'shield')
                        obj.Ns = [obj.Ns next_node.idx];
                    % Case 2: Current state is unsafe - mark as shielding node
                    elseif strcmp(status,'unsafe')
                        obj.Ls = [obj.Ls next_node.idx];
%                         warning(['Node ', num2str(next_node.idx),...
%                             ' is unsafe! Check for numerical issues or bugs. Layer = ',...
%                             num2str(next_node.t)])
                        break_flag = true;
                    % Case 3: Current state is not in the QMDP grid - mark as shielding node
                    elseif strcmp(status,'not-in-grid')
                        obj.Ls = [obj.Ls next_node.idx];
                        break_flag = true;
                    elseif ~strcmp(status,'non-shield')
                        error('Invalid status of the elected node!')
                    end
                    
                    % Add the new node to the tree
                    obj = obj.addNewNode(next_node);
                    obj.N(next_node.pre_node_idx).IsLeaf = false;
                    m = m + 1;
                    
                    % Append the index of the new node to Layers 
                    layer_idx = next_node.t+1;
                    obj.Layer{layer_idx} = [obj.Layer{layer_idx}...
                        next_node.idx];
                    
                    % Update layer_max if necessary
                    if next_node.t+1 > obj.layer_max
                        obj.layer_max = next_node.t+1;
                    end
                    
                    % Append the index of the new node to the current branch
                    branch.idx_set = [branch.idx_set next_node.idx];
                    
                    if m>M || break_flag
                        break
                    end
                    
                    current_node = next_node;
                    
                end
                
                % Compute cost alone the branch
                branch.cost = obj.computeBranchCost(branch);
                
                % Add the branch to the cell
                obj.Branch{end+1} = branch;
                
                % Find a new start_node
                [start_node, success_flag] = obj.findStartNode();
                if ~success_flag
                    break
                end
                obj.N(start_node.idx).BranchCount =...
                                     obj.N(start_node.idx).BranchCount + 1;
                
                tree_construct_time = toc;
                if tree_construct_time > obj.time_out
                    break
                end
            end
            if verbose
                toc
            end
            % ------- End tree construction -------
            
            if length(obj.N) > 1
                % Normalize the probabilities of nodes in each layer such
                % that they sum up to 1
                obj = obj.normalizePathProb();

                % Identify the leaf set and compute terminal sets & costs
                % for leaf nodes
                obj = obj.getLeafSet(verbose);
            end
            
            % Initialize decision variables
            for i = 1:length(obj.N)
                nx = size(obj.planner.Bd,1);
                nu = size(obj.planner.Bd,2);
            	obj.N(i) = obj.N(i).init_YALMIP_variables(nx, nu);
            end
            
            % Report construction results
            if verbose
                disp('Tree construction completed!')
                for l = 1:obj.layer_max
                    disp(['Layer #', num2str(l), ': ',...
                        num2str(length(obj.Layer{l})), ' node(s)'])
                end
                disp(['Number of shielding node(s) in Ns: ',...
                    num2str(length(obj.Ns))])
                disp(['Number of leaf node(s) in L: ',...
                    num2str(length(obj.L))])
                disp(['Number of shielding leaf node(s) in Ls: ',...
                    num2str(length(obj.Ls))])
            end
        end % end constructSparseShield
    end % end methods
end % end class