function x_tar = getTargetLgtPos(xH, vH_Des, Nmpc, XH_pred, params)
% function x_tar = getTargetLgtPos(xH, vH_Des, Nmpc, XH_pred, params)
%     Compute robot's target longitudinal position given human's vDes or
%     predicted trajectory XH
    if isnan(XH_pred)
        x_tar = xH(1) + vH_Des*Nmpc + 1.2*params.xr_tar_overtake;
    else
        x_tar = XH_pred(1,end) + 1.2*params.xr_tar_overtake;
    end
end