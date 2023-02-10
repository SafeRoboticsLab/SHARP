close all; clear all; clc
% Computes the safe set and controller using HJ Reachability for SHARP.


%% Static parameters
% sampling time of the planner
ts = 0.2;

% road width
params.rd_bd_min = -3.7;
params.rd_bd_max = 3.7;

% relative longitudinal distance
params.rd_len_lb = -18;
params.rd_len_ub = 12;

% relative velocity bounds
params.v_rel_lb = -10;
params.v_rel_ub = 10;

% target set specs
params.xr_tar_overtake = 10;
params.xr_tar_lanekeep = params.rd_len_lb+3;

% avoid set specs
params.avoid.lgt_lb = -5.5;
params.avoid.lgt_ub = 5.5;
params.avoid.lat_bd = 2.0;

% input bounds and model parameters
params.accMax_R  = 3;
params.vLatMax_R = 3;
params.accMax_H  = 1;
params.vLatMax_H = 1;
params.accMax_R_sh  = 3;
params.vLatMax_R_sh = 3;
params.accMax_H_sh  = 1;
params.vLatMax_H_sh = 1;
params.talpha    = 0.01;


%% Create grid 
% states    = [x_r               y_R               y_H               v_r]
HJ.grid_min = [params.rd_len_lb; params.rd_bd_min; params.rd_bd_min; params.v_rel_lb];     % Lower corner of computation domain
HJ.grid_max = [params.rd_len_ub; params.rd_bd_max; params.rd_bd_max; params.v_rel_ub];     % Upper corner of computation domain

HJ.N = [41; 15; 15; 31];              % Number of grid points per dimension
HJ.pdDims = [];                       % Periodic dimensions
g = createGrid(HJ.grid_min, HJ.grid_max, HJ.N, HJ.pdDims);


%% Shape target and avoid sets
% going off road boundaries - Robot
rd_bd_left_R  = shapeRectangleByCorners(g,...
    [-inf; params.rd_bd_max-0.5; -inf; -inf], [inf; inf; inf; inf]);
rd_bd_right_R = shapeRectangleByCorners(g,...
    [-inf; -inf; -inf; -inf], [inf; params.rd_bd_min+0.5; inf; inf]);
D_compl_R = shapeUnion(rd_bd_left_R, rd_bd_right_R);

% going off road boundaries - Human
rd_bd_left_H  = shapeRectangleByCorners(g,...
    [-inf; -inf; params.rd_bd_max; -inf], [inf; inf; inf; inf]);
rd_bd_right_H = shapeRectangleByCorners(g,...
    [-inf; -inf; -inf; -inf], [inf; inf; params.rd_bd_min; inf]);
D_compl_H = shapeUnion(rd_bd_left_H, rd_bd_right_H);

% avoid set - Robot
HJ.avoid = shapeRobotAvoid(g, params);
HJ.avoid = shapeUnion(HJ.avoid, D_compl_R);

% target set - Robot
target_ot = shapeRectangleByCorners(g,...
    [params.xr_tar_overtake; 0; -inf; -inf],...
    [inf; params.rd_bd_max; inf; inf]); % overtake
target_lk = shapeRectangleByCorners(g,...
    [-inf; params.rd_bd_min; -inf; -inf],...
    [params.xr_tar_lanekeep; params.rd_bd_max; inf; inf]); % lanekeep
HJ.target = shapeUnion(target_ot, target_lk);
HJ.target = shapeUnion(HJ.target, D_compl_H);

% overall feasible set
HJ.X = shapeRectangleByCorners(g, HJ.grid_min, HJ.grid_max);

clear rd_bd_left_R rd_bd_right_R D_compl_R rd_bd_left_H
clear rd_bd_right_H D_compl_H target_ot target_lk


%% Compute the Reach-Avoid set (safe set)
% define dynamic system
dims = 1:4;
dCar = DubinsCar4D_HRI([0,0,0,0], params.accMax_R_sh,...
    params.accMax_H_sh, params.vLatMax_R_sh, params.vLatMax_H_sh,...
    params.talpha, dims);

% put grid and dynamic systems into schemeData
SafeSetData.grid     = g;
SafeSetData.dynSys   = dCar;
SafeSetData.accuracy = 'medium';
SafeSetData.uMode    = 'min';
SafeSetData.dMode    = 'max';
SafeSetData.minWith  = 'minVOverTime';

HJIextraArgs.visualize = false;
HJIextraArgs.fig_num = 1;
HJIextraArgs.deleteLastPlot = false;
HJIextraArgs.visualizeTarget = true;
HJIextraArgs.obstacles = HJ.avoid;
HJIextraArgs.stopConverge = true;

% time vector
tau = 0:0.05:15;

% solve the HJI PDE
[SafeSet, ~, ~] = ...
  HJIPDE_solve(HJ.target, tau, SafeSetData, SafeSetData.minWith,...
  HJIextraArgs);
HJ.SafeSet = SafeSet(:,:,:,:,end);

% get the lookup table of spatial derivatives
SafeSetData.Deriv = computeGradients(g, HJ.SafeSet);

HJ.SafeSetData = SafeSetData;


%% Compute the one-step BRS of the unsafe set (BRS)
% define dynamic system
dims = 1:4;
dCar_BRS = DubinsCar4D_HRI([0,0,0,0], params.accMax_R, params.accMax_H,...
    params.vLatMax_R, params.vLatMax_H, params.talpha, dims);

% put grid and dynamic systems into schemeData
BRSData.grid     = g;
BRSData.dynSys   = dCar_BRS;
BRSData.accuracy = 'medium';
BRSData.uMode    = 'min';
BRSData.dMode    = 'min';
BRSData.minWith  = 'minVOverTime';

% time vector
tau = 0:0.05:ts;

% get the unsafe set by taking complement of the SafeSet
HJ.UnsafeSet = shapeComplement(HJ.SafeSet);

% solve the HJI PDE
[BRS, ~, ~] = HJIPDE_solve(HJ.UnsafeSet, tau, BRSData, BRSData.minWith);
HJ.BRS = shapeIntersection(BRS(:,:,:,:,end),HJ.SafeSet);

% get the lookup table of spatial derivatives
BRSData.Deriv = computeGradients(g, HJ.BRS);

HJ.BRSData = BRSData;

clear ts dims dCar dCar_BRS g HJIextraArgs tau SafeSet SafeSetData BRS BRSData

save workspace


%% Functions
function data = shapeRobotAvoid(grid, params)
% find the implicit surface of the Robot's avoid set: 
% -x_r + lgt_lb <= 0        (data1), and
%  x_r - lgt_ub <= 0        (data2), and
%  y_R - y_H - lat_bd <= 0  (data3), and
% -y_R + y_H - lat_bd <= 0  (data4)
% state vector = [x_r, y_R, y_H, v_r]
%
% NOTICE: 
% This function assumes zero sublevel set, i.e. negative inside,
% positive outside. Add a negative sign if using this as an avoid set.

    % set specifications
    lgt_lb = params.avoid.lgt_lb;
    lgt_ub = params.avoid.lgt_ub;
    lat_bd = params.avoid.lat_bd;

    % data1: -x_r + lgt_lb <= 0
    data1 = -grid.xs{1} + lgt_lb;

    % data2: x_r - lgt_ub <= 0
    data2 = grid.xs{1} - lgt_ub;

    % data3: y_R - y_H - lat_bd <= 0
    data3 = grid.xs{2} - grid.xs{3} - lat_bd;

    % data4: -y_R + y_H - lat_bd <= 0
    data4 = -grid.xs{2} + grid.xs{3} - lat_bd;

    % the final data is just the intersection of the four
    data = shapeIntersection(data1, data2);
    data = shapeIntersection(data,  data3);
    data = shapeIntersection(data,  data4);
end
