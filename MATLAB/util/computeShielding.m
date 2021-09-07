function [uR_sh, deriv] = computeShielding(xRel, HJ, isProj)
% function [uR_sh, deriv] = computeShielding(xRel, HJ, isProj)
%     Computes the shielding policy given the optimal value function
%     represented by (g, data), associated dynamics given in dynSys
%
% Author: Haimin Hu (last modified 2021.8.10)

    if nargin < 3
        isProj = true;
    end
    
     % Project the state back to the grid
    if isProj
        xRel = max(min(xRel, HJ.grid_max),  HJ.grid_min);
    end

    % Compute the HJ-based shielding policy
    deriv = eval_u(HJ.SafeSetData.grid, HJ.SafeSetData.Deriv, xRel);
    uR_sh = HJ.SafeSetData.dynSys.optCtrl([], [], deriv,...
        HJ.SafeSetData.uMode);
    uR_sh = cell2mat(uR_sh);
    if isnan(uR_sh(1))
        warning('xRel out of the HJ grid range, uR_sh is set to 0!')
    end
end