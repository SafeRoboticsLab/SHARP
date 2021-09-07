function plotSim(HJ, planner, XR, UR, XH, HH, UH, XRel, BETA, THETA,...
    mode, option)
% function plotSim(HJ, planner, XR, UR, XH, HH, UH, XRel, BETA, THETA,...
%     mode, option)
%     Plot simulation trajectories
%
% ----- How to use this function -----
%
% Inputs:
%   HJ              - HJ pre-computation results
%   planner         - The planner object
%   XR              - Robot's state trajectory
%   UR              - Robot's input trajectory
%   XH              - Human's state trajectory
%   HH              - Human's heading trajectory
%   UH              - Human's input trajectory
%   XRel            - Trajectory of relative states
%   BETA            - Trajectory of beta
%   THETA           - Trajectory of theta
%   mode            - Vector indicating which policy is used at each time
%   option          - Plotting options
%
%
% Author: Haimin Hu (last modified 2021.9.6)

    params = planner.params;
    
    % Extend the road
    params.rd_len_ub = params.rd_len_ub*1.0;
    option.scale = 1;
    
    % Image rotation to align the front with the world-frame x-axis 
    option.rotation = 0;
    
    % Image coordinates of the centre of the back wheels
    option.centre = [348; 203];
    
    % Real world length for scaling
    option.length = 4.5;
    
    option.fps = Inf;
    visSetextraArg.LineWidth = 2;
    
    % Start and end time
    if ~isempty(option.t_start)
        t_start = option.t_start;
    else
        t_start = 1;
    end
    if ~isempty(option.t_end)
        t_end = option.t_end;
    else
        t_end = length(UR);
    end

    % Transparency settings
    if option.keep_traj
%         alpha_vec = linspace(0.1,1,t_end-t_start+1);
        alpha_vec = logspace(-1,0,t_end-t_start+option.t_skip);
    else
        alpha_vec = linspace(1,1,length(UR));
    end
    
    % Pre-compute projected safe sets associated with the trajectory (for
    % faster animation)
    if option.plot_ls
        skip_step = 1;
        disp('Computing the projected safe sets...')
        gSS_cell    = cell(length(UR));
        dataSS_cell = cell(length(UR));
        for t = 1:skip_step:length(UR)
            [gSafeProj, dataSafeProj] = proj(HJ.SafeSetData.grid,...
                HJ.SafeSet, [0 0 0 1], XRel(4,t));
            [gSafeProj_R, dataSafeProj_R] = proj(gSafeProj,...
                dataSafeProj, [0 0 1], XH(2,t));
            for k = t:t+skip_step-1
                gSS_cell{k}    = gSafeProj_R;
                dataSS_cell{k} = dataSafeProj_R;
            end
        %     disp(['progess: t = ', num2str(t)])
        end
    end

    % Figure setup
    fs = 25;
    f = figure('Color','white');
    
    % Plot in the relative coordinate
    if strcmp(option.coordinate,'rel')
        f.Position = [1050 660 1500 665];
    
    % Plot in the absolute coordinate
 	elseif strcmp(option.coordinate,'abs')
        f.Position = [0 660 3000 665];
    else
        error('Invalid coordinate option')
    end
    set(gca,'FontSize',fs)
%     set(gca,'Visible','off')
    hold on
    daspect([1,1,1])
    
    % Initial (px,py) limits
    if strcmp(option.coordinate,'rel')
        xlimSpan = [params.rd_len_lb, params.rd_len_ub];
    elseif strcmp(option.coordinate,'abs')
        xlimSpan = [params.rd_len_lb + XH(1,t_start),...
            params.rd_len_ub + XR(1,t_start)];
    end
    ylimSpan = [params.rd_bd_min, params.rd_bd_max];
    xlim(xlimSpan)
    ylim(ylimSpan)
    
    % Plot the roads
    if strcmp(option.coordinate,'rel')
        % Road color
    	fill([xlimSpan(1),xlimSpan(1),xlimSpan(2),xlimSpan(2)],...
             [ylimSpan(1),ylimSpan(2),ylimSpan(2),ylimSpan(1)],...
             [191,191,191]/255,'LineStyle','none');
        % Road boundaries
        plot(linspace(params.rd_len_lb,params.rd_len_ub,2),...
             linspace(params.rd_bd_min,params.rd_bd_min,2),...
             'k-','LineWidth',5)
        plot(linspace(params.rd_len_lb,params.rd_len_ub,2),...
             linspace(params.rd_bd_max,params.rd_bd_max,2),...
             'k-','LineWidth',5) 
        % Center line of the road
        plot(linspace(params.rd_len_lb,params.rd_len_ub,2),...
             linspace(0,0,2),'--','LineWidth',8,'Color',[255,255,255]/255) 
    elseif strcmp(option.coordinate,'abs')
        road_end = XR(1,end)+params.rd_len_ub;
        % Road color
        fill([xlimSpan(1),xlimSpan(1),road_end,road_end],...
             [ylimSpan(1),ylimSpan(2),ylimSpan(2),ylimSpan(1)],...
             [191,191,191]/255);
        % Road boundaries
        plot(linspace(params.rd_len_lb,road_end,2),...
             linspace(params.rd_bd_min,params.rd_bd_min,2),...
             'k-','LineWidth',5)
        plot(linspace(params.rd_len_lb,road_end,2),...
             linspace(params.rd_bd_max,params.rd_bd_max,2),...
             'k-','LineWidth',5)
        % Center line of the road
        plot(linspace(params.rd_len_lb,road_end,2),...
             linspace(0,0,2),'--','LineWidth',8,'Color',[255,255,255]/255)
    end
     
    % Plot the target set
%     if option.plot_ls
%         if strcmp(option.coordinate,'abs')
%             error('Cannot plot level sets in absolute coordinate!')
%         end
%         [gTarProjR, dataTarProjR] = proj(HJ.SafeSetData.grid,...
%             HJ.target, [0 0 1 1]);
%         visSetIm(gTarProjR, dataTarProjR, 'g', 0, visSetextraArg);
%     end

    % Plot vehicle movements
    cnt = 1;
    for t = t_start:t_end
        
        if mod(t-t_start,option.t_skip)~=0 &&...
                t~=t_end && ~strcmp(mode{t},'shielding')
            continue
        end
        
        % Human uncertainty model parameters
        beta_t  = BETA(t);
        theta_t = THETA(t);
        
        % Show the current safe set
        if option.plot_ls && strcmp(option.coordinate,'rel')
            visSetextraArgSS.LineWidth = 2.0*visSetextraArg.LineWidth;
            hSS = visSetIm(gSS_cell{t}, dataSS_cell{t}, 'b',...
                0, visSetextraArgSS);
        end

        % Top-down view of the cars in the relative coordinate
        if strcmp(option.coordinate,'rel')
            % Casual plots
            if strcmp(option.car_plot, 'casual')
                if strcmp(mode{t},'shielding')
                    hR = plotCar(XRel(1,t),XR(2,t),XR(3,t),UR(2,t),'r');
                elseif strcmp(mode{t},'SHARP')
                    hR = plotCar(XRel(1,t),XR(2,t),XR(3,t),UR(2,t),'g');
                else
                    hR = plotCar(XRel(1,t),XR(2,t),XR(3,t),UR(2,t),'y');
                end
                if ~option.keep_traj && t~=t_end && t>t_start
                    delete(hH)
                end
                hH = plotCar(0, XH(2,t), XH(3,t), UH(2,t), 'w');
            % Fancy plots
            elseif strcmp(option.car_plot, 'fancy')
                xR_plt = xyv2xyt([XRel(1,t); XR(2:3,t)], UR(:,t));
                if strcmp(mode{t},'shielding')
                 [option.image,~,option.alpha] = imread('car_robot_r.png');
                elseif strcmp(mode{t},'bMPC-CA')
                 [option.image,~,option.alpha] = imread('car_robot_o.png');
                elseif strcmp(mode{t},'QMDP')
                 [option.image,~,option.alpha] = imread('car_robot_b.png');
                elseif strcmp(mode{t},'SMPC')
                 [option.image,~,option.alpha] = imread('car_robot_g.png');
                else
                 [option.image,~,option.alpha] = imread('car_robot_y.png');
                end
                % Transparent snapshots
                if ~strcmp(mode{t},'shielding')
                    try
                        option.alpha = option.alpha*alpha_vec(cnt);
                    catch
                        option.alpha = option.alpha*alpha_vec(end);
                    end
                end
                [~, hR] = plot_vehicle(xR_plt', 'model', option);
                xH_plt = [0; XH(2,t); HH(t)];
                [option.image, ~, option.alpha] = imread('car_human.png');
                if ~option.keep_traj && t~=t_end && t>t_start
                    delete(hH)
                end
                try
                    option.alpha = option.alpha*alpha_vec(cnt);
                catch
                    option.alpha = option.alpha*alpha_vec(end);
                end
                [~, hH] = plot_vehicle(xH_plt', 'model', option);
            end
        % Top-down view of the cars in the absolute coordinate
        elseif strcmp(option.coordinate,'abs')
            % Casual plots
            if strcmp(option.car_plot,'casual')
                error('Casual plots in abs. coordinate is not unavailable')
            % Fancy plots
            elseif strcmp(option.car_plot, 'fancy')
                if t>t_start
                    xlimSpan(1) = xlimSpan(1) + 0.8*(XH(1,t) -...
                        XH(1,t-option.t_skip));
                    xlimSpan(2) = xlimSpan(2) + XR(1,t) -...
                        XR(1,t-option.t_skip);
                    xlim(xlimSpan)
                end
                xR_plt = xyv2xyt(XR(:,t), UR(:,t));
                if strcmp(mode{t},'shielding')
                 [option.image,~,option.alpha] = imread('car_robot_r.png');
                elseif strcmp(mode{t},'bMPC-CA')
                 [option.image,~,option.alpha] = imread('car_robot_o.png');
                elseif strcmp(mode{t},'QMDP')
                 [option.image,~,option.alpha] = imread('car_robot_b.png');
                elseif strcmp(mode{t},'SMPC')
                 [option.image,~,option.alpha] = imread('car_robot_g.png');
                else
                 [option.image,~,option.alpha] = imread('car_robot_y.png');
                end
                % Transparent snapshots
                option.alpha = option.alpha*alpha_vec(cnt);
                [~, hR] = plot_vehicle(xR_plt', 'model', option);
                xH_plt = [XH(1:2,t); HH(t)];
                [option.image, ~, option.alpha] = imread('car_human.png');
                if ~option.keep_traj && t~=t_end && t>t_start
                    delete(hH)
                end
                option.alpha = option.alpha*alpha_vec(cnt);
                [~, hH] = plot_vehicle(xH_plt', 'model', option);
            end
        end

        % Axis and title
        if strcmp(option.coordinate,'rel')
            xlabel('$p_x^r$','Interpreter','latex','FontSize',1.2*fs)
        elseif strcmp(option.coordinate,'abs')
            xlabel('$p_x$','Interpreter','latex','FontSize',1.2*fs)
        end
        ylabel('$p_y$','Interpreter','latex','FontSize',1.2*fs)
        title(['$t = $', num2str(t*planner.ts),...
            ' s, $\beta = $', num2str(beta_t),...
            ', $\theta =$ [', num2str(theta_t),...
            ',', num2str(1-theta_t),']'], 'Interpreter', 'latex',...
            'FontSize',1.2*fs)
        
        % Pausing settings
        if t == t_start
            pause(1.0)
        else
            pause(option.pause)
        end
        
        % Delete the last frame
        if  t~=t_end
            if ~option.keep_traj
                delete(hR)
            end
            if option.plot_ls && strcmp(option.coordinate,'rel')
                delete(hSS)
            end
        end
        cnt = cnt + option.t_skip;
    end
    if ~option.keep_traj
        delete(hH)
    end
end


function hV = plotCar(xr, y, v, vLat, color)
% hV = plotCar(xr, y, v, vLat, color)
%   Plot casual 2D car position and orientation

    % car sizes (in meters)
    w = 1.83;
    w = w/2;
    l = 4.83;
    l = l/2;

    % yaw angle
    phi = asin(vLat/v);

    % 2D position
    x  = [xr; y];

    point1 = rot(x,0.8*w,0.7*l,phi,[0.0;0.0]);
    hV(1) = fill(point1(1:5),point1(6:10),color);   % Main Body
    hold on
    point2 = rot(x,0.70*w,0.15*l,phi,[0.3;0.0]);
    hV(2) = fill(point2(1:5),point2(6:10),'k');     % Front window
    hold on
    point3 = rot(x,0.70*w,0.20*l,phi,[-0.4;0.0]);
    hV(3) = plot(point3(1:5),point3(6:10),'k');     % Frame
    hold on
    point4 = rot(x,0.70*w,0.10*l,phi,[-0.8;0.0]);
    hV(4) = fill(point4(1:5),point4(6:10),'k');     % Rear window
    hold on
    point5 = rot(x,0.15*w,0.10*l,phi,[1.5;0.55]);
    hV(5) = fill(point5(1:5),point5(6:10),color);   % Left headlight
    hold on
    point6 = rot(x,0.15*w,0.10*l,phi,[1.5;-0.55]);
    hV(6) = fill(point6(1:5),point6(6:10),color);   % Right headlight
    hold on
    point7 = rot(x,0.85*w,0.08*l,phi,[-1.6;0.0]);
    hV(7) = fill(point7(1:5),point7(6:10),color);   % Rear Wing


    function point = rot(x,a,b,phi,trans)
        v1 = x(1:2) + [cos(phi)*b;sin(phi)*b] +...
            [sin(phi)*a;-cos(phi)*a] +...
            [-sin(phi)*trans(2);cos(phi)*trans(2)] +...
            [cos(phi)*trans(1);sin(phi)*trans(1)];
        v2 = x(1:2) + [cos(phi)*b;sin(phi)*b] -...
            [sin(phi)*a;-cos(phi)*a] +...
            [-sin(phi)*trans(2);cos(phi)*trans(2)] +...
            [cos(phi)*trans(1);sin(phi)*trans(1)];
        v4 = x(1:2) - [cos(phi)*b;sin(phi)*b] +...
            [sin(phi)*a;-cos(phi)*a] +...
            [-sin(phi)*trans(2);cos(phi)*trans(2)] +...
            [cos(phi)*trans(1);sin(phi)*trans(1)];
        v3 = x(1:2) - [cos(phi)*b;sin(phi)*b] -...
            [sin(phi)*a;-cos(phi)*a] +...
            [-sin(phi)*trans(2);cos(phi)*trans(2)] +...
            [cos(phi)*trans(1);sin(phi)*trans(1)];
        point = [v1(1), v2(1), v3(1), v4(1), v1(1), v1(2), v2(2), v3(2),...
            v4(2), v1(2)];
    end

end


function state_xyt = xyv2xyt(state_xyv, u)
    theta = real(asin(u(2)/state_xyv(3)));
    state_xyt = [state_xyv(1:2); theta];
end