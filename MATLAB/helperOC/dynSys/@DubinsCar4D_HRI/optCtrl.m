function uOpt = optCtrl(obj, ~, ~, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)
%     Dynamics of the DubinsCar4D_HRI
%         \dot{x}_1 = x4
%         \dot{x}_2 = u2
%         \dot{x}_3 = d2
%         \dot{x}_4 = u1 - d1 - talpha * x4

%% Input processing
if nargin < 5
  uMode = 'min';
  disp('Setting control mode to min')
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% Optimal control
if strcmp(uMode, 'min')
    accOpt_R  = (deriv{4}>=0)*(-obj.accMax_R)  + (deriv{4}<0)*obj.accMax_R;
    vLatOpt_R = (deriv{2}>=0)*(-obj.vLatMax_R) + (deriv{2}<0)*obj.vLatMax_R;
    uOpt = {accOpt_R;vLatOpt_R};
elseif strcmp(uMode, 'max')
    accOpt_R  = (deriv{4}>=0)*obj.accMax_R  + (deriv{4}<0)*(-obj.accMax_R);
    vLatOpt_R = (deriv{2}>=0)*obj.vLatMax_R + (deriv{2}<0)*(-obj.vLatMax_R);
    uOpt = {accOpt_R;vLatOpt_R};
else
    error('Unknown uMode!')
end

end