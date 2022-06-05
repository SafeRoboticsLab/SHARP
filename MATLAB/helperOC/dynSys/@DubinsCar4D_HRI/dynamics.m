function dx = dynamics(obj, ~, x, u, d)
%   Dynamics of the DubinsCar4D_HRI
%       \dot{x}_1 = x4
%       \dot{x}_2 = u2
%       \dot{x}_3 = d2
%       \dot{x}_4 = u1 - d1 - talpha * x4
%
% Haimin Hu, 2020-10-06

if iscell(x)
  dx = cell(length(obj.dims), 1);
  
  for i = 1:length(obj.dims)
    dx{i} = dynamics_cell_helper(obj, x, u, d, obj.dims, obj.dims(i));
  end
else
  dx = zeros(obj.nx, 1);
  
  dx(1) = x(4);
  dx(2) = u(2);
  dx(3) = d(2);
  dx(4) = u(1) - d(1) - obj.talpha * x(4);
end
end

function dx = dynamics_cell_helper(obj, x, u, d, dims, dim)

switch dim
  case 1
    dx = x{4};
  case 2
    dx = u{2};
  case 3
    dx = d{2};
  case 4
    dx = u{1} - d{1} - obj.talpha * x{4};
  otherwise
    error('Only dimension 1-4 are defined for dynamics of DubinsCar!')
end
end