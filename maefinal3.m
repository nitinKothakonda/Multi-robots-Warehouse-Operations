%% MRS Project – Dynamic + Collision Avoidance + Battery Charging
clear; clc; close all;

%% ------------------- WORLD SETTINGS -------------------
world_xmax = 14;   
world_ymax = 10;
% Charging Stations [x y]
charger_locs = [1 5; 13 5]; 
charger_radius = 0.8;

%% ------------------- ROBOTS INIT -------------------
N = 10;
robots.pos = [ ...
    2 2; 2 4; 2 8; 4 2; 4 6; 
    7 1; 7 5; 9 3; 11 2; 11 8]; 

% Battery Params
battery = 100 * ones(N,1);       % Start at 100%
chargingState = zeros(N,1);      % 0=Normal, 1=GoingToCharger, 2=Charging
bat_thresh_low = 30;             % Go to charger if < 30%
bat_thresh_high = 95;            % Stop charging if > 95%
bat_drain_move = 4.0;            % % per second per m/s speed
bat_drain_idle = 0.5;            % % per second (base cost)
bat_charge_rate = 15.0;          % % per second

%% ------------------- DYNAMIC TASK SETUP -------------------
tasks = struct('id',{},'type',{},'src',{},'dst',{}); 
taskStatus = []; % 0=pending, 1=active, 2=done
next_task_id = 1;

% Spawn Initial Batch
for i=1:5
    [tasks, next_task_id] = addRandomTask(tasks, next_task_id, world_xmax, world_ymax);
    taskStatus(end+1) = 0; 
end

%% ------------------- STATE VARIABLES -------------------
allocation.single = zeros(0, 2); % [robot_idx, task_idx]
allocation.coop   = zeros(0, 3); % [r1_idx, r2_idx, task_idx]

singleTaskOf = nan(N,1);
coopTaskOf   = nan(N,1);
leaderOfTask = []; 

singleStage = zeros(N,1);    
coopStage   = [];           

total_delivered = 0;

% Simulation Params
dt       = 0.05;    
T        = 90;      % Long duration to see battery cycles
steps    = round(T/dt);
v_single = 1.5;     
v_leader = 1.2;     
v_max    = max(v_single, v_leader);
k_form   = 3.0;     
d_star   = 1.2;     
eps_pos  = 0.15;    
smooth_alpha = 0.4;
pos      = robots.pos;
u_prev   = zeros(N,2);

last_spawn_time = 0;
spawn_interval  = 4.0; 

%% ------------- MAIN SIMULATION LOOP -------------------
figure('Name','Warehouse with Battery', 'Position', [100 100 900 600]);

for k = 1:steps
    t_now = (k-1)*dt;
    
    %% --- 1. DYNAMIC TASK GENERATION ---
    if t_now - last_spawn_time > spawn_interval
        [tasks, next_task_id] = addRandomTask(tasks, next_task_id, world_xmax, world_ymax);
        taskStatus(end+1) = 0; 
        coopStage(end+1)  = 0; 
        last_spawn_time   = t_now;
    end
    
    %% --- 2. CHECK COMPLETION & CLEANUP ---
    % Single
    del_idx = [];
    for i = 1:size(allocation.single, 1)
        r = allocation.single(i,1);
        t = allocation.single(i,2);
        if singleStage(r) == 2
            taskStatus(t) = 2; 
            total_delivered = total_delivered + 1;
            singleStage(r) = 0; 
            singleTaskOf(r) = NaN;
            del_idx = [del_idx; i]; %#ok<AGROW>
        end
    end
    allocation.single(del_idx,:) = []; 

    % Coop
    del_idx = [];
    for i = 1:size(allocation.coop, 1)
        t = allocation.coop(i,3);
        if t <= length(coopStage) && coopStage(t) == 2
            taskStatus(t) = 2; 
            total_delivered = total_delivered + 1;
            r1 = allocation.coop(i,1);
            r2 = allocation.coop(i,2);
            coopTaskOf(r1) = NaN;
            coopTaskOf(r2) = NaN;
            del_idx = [del_idx; i]; %#ok<AGROW>
        end
    end
    allocation.coop(del_idx,:) = [];
    
    %% --- 3. BATTERY CHECK (TRIGGER CHARGING) ---
    for r = 1:N
        % If free AND low battery, go to charge
        if isnan(singleTaskOf(r)) && isnan(coopTaskOf(r))
            if battery(r) < bat_thresh_low && chargingState(r) == 0
                chargingState(r) = 1; % Start going to charger
            end
        end
        % If Charging complete
        if chargingState(r) == 2 && battery(r) >= bat_thresh_high
            chargingState(r) = 0; % Back to work
        end
    end

    %% --- 4. DYNAMIC ALLOCATION ---
    pendingIdx = find(taskStatus == 0);
    
    % Identify unavailable robots
    busyRobots = [allocation.single(:,1); allocation.coop(:,1); allocation.coop(:,2)];
    chargingRobots = find(chargingState > 0);
    % Robots that are free but too low to start a new task
    lowBatRobots = find(battery < bat_thresh_low);
    
    unavail = unique([busyRobots; chargingRobots; lowBatRobots]);
    freeRobotsIdx = setdiff(1:N, unavail);
    
    if ~isempty(pendingIdx) && ~isempty(freeRobotsIdx)
        % A. Single Tasks
        singleTasks = pendingIdx([tasks(pendingIdx).type] == "single");
        for ti = 1:length(singleTasks)
            if isempty(freeRobotsIdx), break; end
            t_real = singleTasks(ti);
            
            dists = sqrt(sum((pos(freeRobotsIdx,:) - tasks(t_real).src).^2, 2));
            [~, bestLoc] = min(dists);
            r_real = freeRobotsIdx(bestLoc);
            
            allocation.single(end+1,:) = [r_real, t_real]; %#ok<AGROW>
            singleTaskOf(r_real) = t_real;
            singleStage(r_real) = 0;
            taskStatus(t_real) = 1; 
            freeRobotsIdx(bestLoc) = []; 
        end
        
        % B. Coop Tasks
        coopTasks = pendingIdx([tasks(pendingIdx).type] == "coop");
        for ti = 1:length(coopTasks)
            if length(freeRobotsIdx) < 2, break; end
            t_real = coopTasks(ti);
            
            r1 = freeRobotsIdx(1);
            r2 = freeRobotsIdx(2);
            
            allocation.coop(end+1,:) = [r1, r2, t_real]; %#ok<AGROW>
            coopTaskOf(r1) = t_real;
            coopTaskOf(r2) = t_real;
            leaderOfTask(t_real) = min(r1,r2);
            
            if t_real > length(coopStage), coopStage(t_real) = 0; else, coopStage(t_real)=0; end
            
            taskStatus(t_real) = 1; 
            freeRobotsIdx([1,2]) = [];
        end
    end
    
    %% --- 5. ROBOT CONTROL LOGIC ---
    u = zeros(N,2);
    for r = 1:N
        % === CHARGING LOGIC ===
        if chargingState(r) > 0
            if chargingState(r) == 1 % Going to charger
                % Find closest charger
                dists = sqrt(sum((pos(r,:) - charger_locs).^2, 2));
                [~, c_idx] = min(dists);
                target = charger_locs(c_idx,:);
                
                dir = target - pos(r,:);
                dist = norm(dir);
                if dist < 0.2
                    u(r,:) = [0 0];
                    chargingState(r) = 2; % Arrived, start charging
                else
                    u(r,:) = v_single * dir / (dist+1e-6);
                end
            elseif chargingState(r) == 2 % Charging
                u(r,:) = [0 0];
                % Battery increases
                battery(r) = min(100, battery(r) + bat_charge_rate * dt);
            end
            
        % === SINGLE TASK ===
        elseif ~isnan(singleTaskOf(r))
            t_id = singleTaskOf(r);
            src = tasks(t_id).src; dst = tasks(t_id).dst;
            target = [];
            if singleStage(r)==0, target=src; elseif singleStage(r)==1, target=dst; end
            
            if ~isempty(target)
                dir = target - pos(r,:);
                dist = norm(dir);
                if dist <= eps_pos
                    if singleStage(r)==0, singleStage(r)=1;
                    elseif singleStage(r)==1, singleStage(r)=2; end
                    u(r,:) = [0 0];
                else
                    u(r,:) = v_single * dir / (dist+1e-6);
                end
            end
            
        % === COOP TASK ===
        elseif ~isnan(coopTaskOf(r))
            t_id = coopTaskOf(r);
            src = tasks(t_id).src; dst = tasks(t_id).dst;
            leader = leaderOfTask(t_id);
            pairRow = find(allocation.coop(:,3)==t_id);
            if ~isempty(pairRow)
                partners = allocation.coop(pairRow, 1:2);
                follower = partners(partners ~= leader);
                
                if coopStage(t_id) == 0 % To Src
                    target = src;
                    dir = target - pos(r,:);
                    if norm(dir) <= eps_pos, u(r,:) = [0 0];
                    else, u(r,:) = v_single * dir / (norm(dir)+1e-6); end
                    
                    if norm(pos(leader,:)-src)<=eps_pos && norm(pos(follower,:)-src)<=eps_pos
                        coopStage(t_id) = 1;
                    end
                elseif coopStage(t_id) == 1 % Carry
                    if r == leader
                        target = dst;
                        dir = target - pos(r,:);
                        if norm(dir) <= eps_pos, u(r,:) = [0 0]; coopStage(t_id)=2;
                        else, u(r,:) = v_leader * dir / (norm(dir)+1e-6); end
                    else
                        pL = pos(leader,:); pF = pos(r,:);
                        dirL = dst - pL;
                        uL = v_leader * dirL / (norm(dirL)+1e-6);
                        if norm(dirL) < 1e-6, uL = [0 0]; end
                        e = (norm(pF - pL)^2 - d_star^2);
                        u(r,:) = uL - k_form * e * (pF - pL);
                    end
                else
                    u(r,:) = [0 0];
                end
            end
        else
            % Idle
            u(r,:) = [0 0];
        end
        
        % === DRAIN BATTERY ===
        % Drain if moving + small base drain
        if chargingState(r) ~= 2
            current_drain = bat_drain_idle + bat_drain_move * (norm(u(r,:)) > 0.1);
            battery(r) = max(0, battery(r) - current_drain * dt);
        end
    end
    
    %% --- 6. COLLISION AVOIDANCE ---
    d_safe = 0.8; k_rep = 0.5;
    for i = 1:N
        for j = i+1:N
            % Check partners
            isPartners = false;
            if ~isnan(coopTaskOf(i)) && ~isnan(coopTaskOf(j))
                if coopTaskOf(i) == coopTaskOf(j), isPartners = true; end
            end
            if isPartners, continue; end
            
            diff = pos(i,:) - pos(j,:);
            dist = norm(diff);
            if dist < d_safe && dist > 1e-3
                rep_dir = diff / dist;
                F_val = k_rep * (1/dist - 1/d_safe) * (1/dist^2);
                F_val = min(F_val, 3.0);
                u(i,:) = u(i,:) + F_val * rep_dir;
                u(j,:) = u(j,:) - F_val * rep_dir;
            end
        end
    end
    
    %% --- 7. UPDATE PHYSICS ---
    for r = 1:N
        spd = norm(u(r,:));
        if spd > v_max, u(r,:) = v_max * u(r,:) / spd; end
        u(r,:) = (1-smooth_alpha)*u_prev(r,:) + smooth_alpha*u(r,:);
    end
    u_prev = u;
    pos = pos + dt*u;
    
    %% --- 8. VISUALIZATION ---
    clf; hold on; grid on; axis equal;
    xlim([-1 world_xmax+1]); ylim([0 world_ymax]);
    title(sprintf('Bat Warehouse (t=%.2f) | Delivered: %d', t_now, total_delivered));
    
    % Draw Chargers
    for c = 1:size(charger_locs,1)
        rectangle('Position',[charger_locs(c,1)-0.5, charger_locs(c,2)-0.5, 1, 1], ...
            'FaceColor',[1 1 0], 'EdgeColor','k', 'Curvature',0.2);
        text(charger_locs(c,1)-0.3, charger_locs(c,2), 'CHG', 'FontSize',8);
    end
    
    % Draw Tasks
    for t_id = 1:length(tasks)
        if taskStatus(t_id) == 2, continue; end
        d = tasks(t_id).dst;
        rectangle('Position',[d(1)-0.4, d(2)+0.3, 0.8, 0.4], 'FaceColor',[0.9 0.9 0.9], 'EdgeColor','none');
        
        currBoxPos = tasks(t_id).src;
        if tasks(t_id).type == "single"
            r = find(singleTaskOf == t_id, 1);
            if ~isempty(r) && singleStage(r)==1, currBoxPos = pos(r,:); end
            plot(currBoxPos(1), currBoxPos(2), 'ks', 'MarkerFaceColor','g', 'MarkerSize',10);
        else
            pair = find(allocation.coop(:,3) == t_id);
            if ~isempty(pair) && coopStage(t_id)==1
                r1=allocation.coop(pair,1); r2=allocation.coop(pair,2);
                currBoxPos = (pos(r1,:) + pos(r2,:))/2;
                plot([pos(r1,1) pos(r2,1)], [pos(r1,2) pos(r2,2)], 'k-', 'LineWidth',2);
            end
            plot(currBoxPos(1), currBoxPos(2), 'ko', 'MarkerFaceColor','r', 'MarkerSize',10);
        end
    end
    
    % Draw Robots
    for r = 1:N
        color = 'b'; % Idle
        if ~isnan(singleTaskOf(r)), color = 'g'; end
        if ~isnan(coopTaskOf(r)), color = 'r'; end
        if chargingState(r) > 0, color = 'y'; end % Charging overrides color
        
        plot(pos(r,1), pos(r,2), 'o', 'MarkerFaceColor', color, 'MarkerEdgeColor','k', 'MarkerSize',8);
        
        % Battery Text
        batt_str = sprintf('%d%%', round(battery(r)));
        text(pos(r,1)+0.3, pos(r,2)+0.2, sprintf('R%d',r), 'FontSize',8);
        
        % Color code battery text
        if battery(r) < 30
            text(pos(r,1)+0.3, pos(r,2)-0.2, batt_str, 'FontSize',7, 'Color','r', 'FontWeight','bold');
        else
            text(pos(r,1)+0.3, pos(r,2)-0.2, batt_str, 'FontSize',7, 'Color','k');
        end
    end
    
    drawnow limitrate;
end

%% ================= HELPER =================
function [tasks, next_id] = addRandomTask(tasks, next_id, X_MAX, Y_MAX)
    if rand > 0.6, type = "coop"; else, type = "single"; end
    pad = 1.5;
    src = [pad + (X_MAX-2*pad)*rand, pad + (Y_MAX-2*pad)*rand];
    dst = [pad + (X_MAX-2*pad)*rand, pad + (Y_MAX-2*pad)*rand];
    k = numel(tasks) + 1;
    tasks(k).id   = next_id;
    tasks(k).type = type;
    tasks(k).src  = src;
    tasks(k).dst  = dst;
    next_id = next_id + 1;
end