%% MRS Project - Plot Generator
% This script loads "simulation_data.mat" and generates report-ready plots.

clear; clc; close all;

%% 1. LOAD DATA
dataFileName = 'simulation_data.mat';

if exist(dataFileName, 'file')
    load(dataFileName);
    fprintf('Loaded simulation data from "%s" successfully.\n', dataFileName);
else
    error('File "%s" not found.\nPlease run "maefinal2.m" first to generate the data.', dataFileName);
end

%% 2. GENERATE REPORT PLOTS

% --- PLOT 1: Formation Error (Stability) ---
% Clean up NaN values for plotting (NaNs occur when no coop task is active)
if isfield(history, 'formation_error')
    err = history.formation_error;
    err(isnan(err)) = 0; % Zero out idle times for a cleaner continuous plot

    figure('Name', 'Formation Error', 'Position',[100 100 600 400]);
    plot(history.timeVec, err, 'r-', 'LineWidth', 2);
    grid on;
    xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Formation Error e(t) [m]', 'Interpreter', 'latex', 'FontSize', 12);
    title('Cooperative Formation Stability Analysis', 'FontSize', 14);

    % Add annotations to explain the phases
    x_spike = 4.5; 
    text(x_spike, 0.5, '\leftarrow Acquisition Phase (Transient)', 'FontSize', 10);
    text(10, 0.05, 'Steady State Transport \rightarrow', 'FontSize', 10);
    ylim([-0.2 1.5]);
else
    warning('Formation error data not found in history.');
end

% --- PLOT 2: Trajectories (Collision Avoidance) ---
figure('Name', 'Trajectories', 'Position',[750 100 600 500]);
hold on; grid on; axis equal;
xlim([0 14]); ylim([0 10]);
xlabel('X Position [m]', 'FontSize', 12);
ylabel('Y Position [m]', 'FontSize', 12);
title('Multi-Robot Trajectories & Collision Avoidance', 'FontSize', 14);

% Plot Paths for specific interesting robots
% We usually pick a Leader (R1), Follower (R2), and an Independent (R6)
colors = lines(N); % Generate a color map
robots_to_plot = [1, 2, 6]; 

for r = robots_to_plot
    % Extract path for robot r
    if r <= N
        path = squeeze(history.pos(:,r,:));
        
        % Determine line style based on role
        if r == 1
            lStyle = '--'; dispName = 'Robot 1 (Leader)';
        elseif r == 2
            lStyle = '-'; dispName = 'Robot 2 (Follower)';
        else
            lStyle = '-.'; dispName = sprintf('Robot %d (Single)', r);
        end
        
        plot(path(:,1), path(:,2), lStyle, 'LineWidth', 1.5, 'Color', colors(r,:), ...
            'DisplayName', dispName);
        
        % Mark the end position
        plot(path(end,1), path(end,2), 'o', 'MarkerFaceColor', colors(r,:), 'HandleVisibility','off');
    end
end
legend('Location','best');

% Add shelf/obstacle rectangles for context (Visualization only)
rectangle('Position',[11.6, 7.8, 0.8, 0.4], 'FaceColor',[0.9 0.9 0.9]);
rectangle('Position',[1.6, 1.8, 0.8, 0.4], 'FaceColor',[0.9 0.9 0.9]);

% --- PLOT 3: Gantt Chart (Efficiency) ---
figure('Name', 'Gantt Chart', 'Position',[400 600 800 300]);
imagesc(history.timeVec, 1:N, history.robot_state');

% Define Color Map: Blue=Idle, Green=Single, Red=Coop
cmap = [0.9 0.9 1;  % 0: Idle (Light Blue)
        0.6 1 0.6;  % 1: Single (Light Green)
        1 0.6 0.6]; % 2: Coop (Light Red)
colormap(cmap); 

xlabel('Time [s]', 'FontSize', 12);
ylabel('Robot ID', 'FontSize', 12);
title('Task Efficiency (Gantt Chart)', 'FontSize', 14);
yticks(1:N);

% Custom colorbar ticks to match the discrete states
cb = colorbar;
cb.Ticks = [0.33, 1, 1.66]; % Centered on the 3 color bands
cb.TickLabels = {'Idle', 'Single Task', 'Coop Task'};

disp('All plots generated successfully.');