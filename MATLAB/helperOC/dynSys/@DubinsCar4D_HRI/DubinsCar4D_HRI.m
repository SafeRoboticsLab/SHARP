classdef DubinsCar4D_HRI < DynSys
  properties
    % Controls (Robot)
    accMax_R    % acceleration bounds (u1)
    vLatMax_R   % lateral velocity bounds (u2)
    
    % Disturbances (Human)
    accMax_H    % acceleration bounds (d1)
    vLatMax_H   % lateral velocity bounds (d2)
    
    % Linearized friction parameter
    talpha
    
    % Dimensions that are active
    dims
  end
  
  methods
    function obj = DubinsCar4D_HRI(x, accMax_R, accMax_H, vLatMax_R, vLatMax_H, talpha, dims)
      % obj = DubinsCar(x, accMax_R, accMax_H, vLatMax_R, vLatMax_H, dims)
      %     DubinsCar4D_HRI class
      %
      % Dynamics:
      %    \dot{x}_1 = x4
      %    \dot{x}_2 = u2
      %    \dot{x}_3 = d2
      %    \dot{x}_4 = u1 - d1 - talpha * x4
      %         u1 \in [-accMax_R, accMax_R]
      %         u2 \in [-vLatMax_R, vLatMax_R]
      %         d1 \in [-accMax_H, accMax_H]
      %         d2 \in [-vLatMax_H, vLatMax_H]
      %
      % Inputs:
      %   x      - state: [x_r; y_R; y_H; v_r]
      %   accMax_i, i={R,H}  - maximum acceleration
      %   vLatMax_i, i={R,H} - maximum lateral velocity
      %   talpha - velocity damping
      %
      % Output:
      %   obj       - a DubinsCar4D_HRI object
      
      if numel(x) ~= obj.nx
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
      
      % Basic vehicle properties
      obj.nx = length(dims);
      obj.nu = 2;
      obj.nd = 2;
      
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.accMax_R = accMax_R;
      obj.accMax_H = accMax_H;
      obj.vLatMax_R = vLatMax_R;
      obj.vLatMax_H = vLatMax_H;
      obj.talpha = talpha;
      obj.dims = dims;
    end
    
  end % end methods
end % end classdef
