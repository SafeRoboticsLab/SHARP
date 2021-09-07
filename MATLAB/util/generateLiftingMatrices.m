function planner = generateLiftingMatrices(planner)
% function planner = generateLiftingMatrices(planner)
%     Define the lifting (change-of-coordinate) matrices

    % 6D joint system states:    [px_R py_R vR  px_H py_H vH]
    %                             <--- xR --->  <--- xH --->

    % Compute the 5D planning states based on the 6D joint states
    %   5D planning system states: [px_Rel py_R py_H vR vH]
    E_6DtoPlan5D = [1 0 0 -1 0 0; 0 1 0 0 0 0; 0 0 0 0 1 0; 0 0 1 0 0 0;
        0 0 0 0 0 1]; 
    planner.E_6DtoPlan5D = E_6DtoPlan5D;

    % Compute the 4D planning states based on the 6D joint states
    %   4D planning system states: [px_Rel py_R py_H vR]
    E_6DtoPlan4D = [1 0 0 -1 0 0; 0 1 0 0 0 0; 0 0 0 0 1 0; 0 0 1 0 0 0]; 
    planner.E_6DtoPlan4D = E_6DtoPlan4D;

    % Compute the 4D shielding states based on the 6D joint states
    %   4D shielding system states: [px_Rel py_R py_H v_Rel]
    E_6DtoSh = [1 0 0 -1 0 0; 0 1 0 0 0 0; 0 0 0 0 1 0; 0 0 1 0 0 -1]; 
    planner.E_6DtoSh = E_6DtoSh;

    % Compute the 3D robot states based on the 6D joint states
    %   3D robot subsystem states: [px_R py_R vR]
    E_6DtoR = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0]; 
    planner.E_6DtoR = E_6DtoR;

    % Compute the 3D robot planning states based on the 6D joint states
    %   3D robot planning subsystem states: [px_Rel py_R vR]
    E_6DtoRplan = [1 0 0 -1 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0]; 
    planner.E_6DtoRplan = E_6DtoRplan;

    % Compute the 3D human states based on the 6D joint states
    %   3D human subsystem states: [px_H py_H vH]
    E_6DtoH = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1]; 
    planner.E_6DtoH = E_6DtoH;

    % Compute the 4D shielding states based on the 5D planning states
    E_Plan5DtoSh = [1 0 0 0 0; 0 1 0 0 0; 0 0 1 0 0; 0 0 0 1 -1]; 
    planner.E_Plan5DtoSh = E_Plan5DtoSh;

    % Compute the 3D robot planning states based on the 5D planning states
    E_Plan5DtoRplan = [1 0 0 0 0; 0 1 0 0 0; 0 0 0 1 0]; 
    planner.E_Plan5DtoRplan = E_Plan5DtoRplan;

    % Compute the 3D human planning states based on the 5D planning states
    E_Plan5DtoHplan = [1 0 0 0 0; 0 0 1 0 0; 0 0 0 0 1]; 
    planner.E_Plan5DtoHplan = E_Plan5DtoHplan;
end