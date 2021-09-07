function [xSol, uSol, VSol, status] = nominalMPC(x0, xF, N, A, B, Q,...
                        R, ulb, uub, vlb, vub, planner)
% function [xSol, uSol, VSol, status] = nominalMPC(x0, xF, N, A, B, Q,...
%                         R, ulb, uub, vlb, vub, planner)
%   Nominal MPC planner
%
% ----- How to use this function -----
%
% Inputs:
%   x0          - Robot's initial state
%   xF          - Robot's target state
%   N           - Prediction horizon
%   A, B        - Robot's dynamics
%   Q, R        - Robot's cost matrices
%   ulb, uub    - Robot's control bounds
%   vlb, vub    - Robot's velocity bounds
%   planner     - The planner object
%
% Outputs:
%   xSol            - Optimal state trajectory
%   uSol            - Optimal control input
%   VSol            - Optimal cost-to-go
%   status          - Solver status
%
%
% Author: Haimin Hu (last modified 2021.9.6)

    yalmip('clear')
    params = planner.params;

    % Define solver settings
    options = sdpsettings('verbose', 0, 'solver', 'mosek', 'usex0', 0,...
        'cachesolvers', 1);

    %----------------------------------------------------------------------
    % Define decision variables
    %----------------------------------------------------------------------
    % States
    x = sdpvar(3,N+1);
    
    % Inputs
    u = sdpvar(2,N);

    %----------------------------------------------------------------------
    % Objective function
    %----------------------------------------------------------------------
    % DLQR-based terminal cost
    [~,P,~] = dlqr(A,B,Q,R);
    
    % Case 1: No tracking objectives for p_x
    if isnan(xF(1)) 
        objective = (x(2:3,N+1)-xF(2:3))'*P(2:3,2:3)*(x(2:3,N+1)-xF(2:3));
        for i = 1:N
           objective = objective + u(:,i)'*R*u(:,i) +...
               (x(2:3,i)-xF(2:3))'*Q(2:3,2:3)*(x(2:3,i)-xF(2:3));
        end
    % Case 2: Tracking the full states
    else
        objective = (x(:,N+1)-xF)'*P*(x(:,N+1)-xF);
        for i = 1:N
           objective = objective + u(:,i)'*R*u(:,i)+...
               (x(:,i)-xF)'*Q*(x(:,i)-xF);
        end
    end

    %----------------------------------------------------------------------
    % Bounds on the states and inputs
    %----------------------------------------------------------------------
    constraint = [];
    % Input constraints
    for i = 1:N
        constraint = [constraint ulb<=u(:,i)<=uub];
    end
    
    % State constraints
    for i = 1:N+1
        constraint = [constraint vlb<=x(3,i)<=vub,...    % velocity bounds
            params.rd_bd_min<=x(2,i)<=params.rd_bd_max]; % road boundaries
    end

    %----------------------------------------------------------------------
    % Initial state constraints
    %----------------------------------------------------------------------
    constraint = [constraint x(:,1)==x0];

    %----------------------------------------------------------------------
    % Dynamics constraints
    %----------------------------------------------------------------------
    for i = 1:N
        constraint = [constraint x(:,i+1) == A*x(:,i)+B*u(:,i)];
    end

    %----------------------------------------------------------------------
    % Solve the MPC problem
    %----------------------------------------------------------------------
    status = optimize(constraint, objective, options);
    isSolved = (status.problem==0);
    if ~isSolved
        message = ['solver info: ', status.info ];
        warning(message);
    end

    %----------------------------------------------------------------------
    % Return values
    %----------------------------------------------------------------------
    xSol = value(x);
    uSol = value(u);
    VSol = value(objective);
end