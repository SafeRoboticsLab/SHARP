function flag = shieldCheck(xRel, uR, planner, HJ, NuH, AdRel, BdRel)
% function flag = shieldCheck(xRel, HJ, NuH)
%   Checking whether shielding is needed
    
    % Indicator (true = needs shielding)
    flag = false;

    switch length(NuH)
        case 1
            % uH discretization
            uH_vec = linspace(planner.ulb_H(1), planner.uub_H(1), NuH(1));
            for i = 1:NuH(1)
                xRel_next = AdRel*xRel + BdRel*[uR; uH_vec(i)];
                value_SS  = eval_u(HJ.SafeSetData.grid, HJ.SafeSet,...
                    xRel_next);
                if value_SS >= 0
                    flag = true;
                    return
                end
            end
        case 2
            % uH discretization
            uH1_vec = linspace(planner.ulb_H(1), planner.uub_H(1), NuH(1));
            uH2_vec = linspace(planner.ulb_H(2), planner.uub_H(2), NuH(2));

            for i = 1:NuH(1)
                for j = 1:NuH(2)
                    xRel_next = AdRel*xRel +...
                        BdRel*[uR; uH1_vec(i); uH2_vec(j)];
                    value_SS  = eval_u(HJ.SafeSetData.grid, HJ.SafeSet,...
                        xRel_next);
                    if value_SS >= 0
                        flag = true;
                        return
                    end
                end
            end
    end
end