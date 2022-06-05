function dOpt = optDstb(obj, ~, ~, deriv, dMode)
% dOpt = optCtrl(obj, t, y, deriv, dMode)
%     Dynamics of the DubinsCar4D_HRI
%         \dot{x}_1 = x4
%         \dot{x}_2 = u2
%         \dot{x}_3 = d2
%         \dot{x}_4 = u1 - d1 - talpha * x4

%% Input processing
if nargin < 5
  dMode = 'max';
  disp('Setting disturbance mode to max')
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% Optimal disturbance
if strcmp(dMode, 'max')
    accOpt_H  = (deriv{4}>=0)*(-obj.accMax_H)  + (deriv{4}<0)*obj.accMax_H;
    vLatOpt_H = (deriv{3}<=0)*(-obj.vLatMax_H) + (deriv{3}>0)*obj.vLatMax_H;
    dOpt = {accOpt_H;vLatOpt_H};
elseif strcmp(dMode, 'min')
    accOpt_H  = (deriv{4}>=0)*obj.accMax_H  + (deriv{4}<0)*(-obj.accMax_H);
    vLatOpt_H = (deriv{3}<=0)*obj.vLatMax_H + (deriv{3}>0)*(-obj.vLatMax_H);
    dOpt = {accOpt_H;vLatOpt_H};
else
    error('Unknown dMode!')
end

end