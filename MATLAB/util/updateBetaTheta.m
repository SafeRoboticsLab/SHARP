function param_distr_next = updateBetaTheta(xH, uH, xR, uR, NuH,...
    planner, param_distr)
% function param_distr_next = updateBetaTheta(xH, uH, xR, uR, NuH,...
%     planner, param_distr)
%     Update the distribution of beta and theta with Bayesian estimation
     
    % Model update (epsilon-static)
    eps = 0.01;
    param_distr_next_tmp=(1-eps)*param_distr+eps*planner.param_distr_init;
    
	% Model update (constant)
%     param_distr_next_tmp  = param_distr;

    % Update distributions of beta and theta
    switch length(uH)
        case 1
            % uH discretization
            uH_vec = linspace(planner.ulb_H,planner.uub_H,NuH);
            UH_grid = createGrid(planner.ulb_H-0.01,planner.uub_H+0.01,NuH);
            
            % Measurement update
            param_distr_next = NaN(planner.Nbeta,planner.Ntheta);
            for i = 1:planner.Nbeta
                for j = 1:planner.Ntheta
                    beta  = planner.beta_vec(i);
                    theta = planner.theta_vec(j);
                    P = computeBoltzmannFull(xH, xR, uR, uH_vec,...
                        planner, beta, theta);
                    P_uH_param = eval_u(UH_grid, P, uH);
                    param_distr_next(i,j) =...
                        P_uH_param * param_distr_next_tmp(i,j);
                end
            end
            param_distr_next = param_distr_next/sum(param_distr_next(:));
        case 2
            % uH discretization
            uH1_vec = linspace(planner.ulb_H(1),planner.uub_H(1),NuH(1));
            uH2_vec = linspace(planner.ulb_H(2),planner.uub_H(2),NuH(2));
            UH_grid = createGrid(planner.ulb_H-0.01,planner.uub_H+0.01,NuH);
            
            % Measurement update
            param_distr_next = NaN(planner.Nbeta,planner.Ntheta);
            for i = 1:planner.Nbeta
                for j = 1:planner.Ntheta
                    beta  = planner.beta_vec(i);
                    theta = planner.theta_vec(j);
                    P = computeBoltzmannFull(xH, xR, uR,...
                        uH1_vec, uH2_vec, planner, beta, theta);
                    P_uH_param = eval_u(UH_grid, P, uH);
                    param_distr_next(i,j) =...
                        P_uH_param * param_distr_next_tmp(i,j);
                end
            end
            param_distr_next = param_distr_next/sum(param_distr_next(:));
    end
    
    % Sanity check (with epsilon tolerance for numerical inaccuracies)
    if ~(0.95<= sum(param_distr_next(:))<=1.01)
        error('probabilities of beta and theta must sum up to 1!')
    end
end