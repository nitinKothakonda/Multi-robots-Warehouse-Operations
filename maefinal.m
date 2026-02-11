%% MRS Project – Task Allocation + Cooperative Carrying (NO A*, fast + boxes + ant-style)
clear; clc; close all;

%% ------------------- WORLD SETTINGS -------------------
world_xmax = 14;   % x in [0,14] m
world_ymax = 10;   % y in [0,10] m

%% ------------------- ROBOTS & TASKS -------------------
N = 10;

robots.pos = [ ...
    1 1;   % R1
    1 4;   % R2
    2 7;   % R3
    4 2;   % R4
    4 6;   % R5
    7 1;   % R6
    7 5;   % R7
    9 3;   % R8
    11 2;  % R9
    11 6]; % R10

% ---- Tasks ----
k = 0;

% Single-robot tasks
k = k + 1;
tasks(k).id   = k;
tasks(k).type = "single";
tasks(k).src  = [2 9];
tasks(k).dst  = [10 9];

k = k + 1;
tasks(k).id   = k;
tasks(k).type = "single";
tasks(k).src  = [1 3];
tasks(k).dst  = [5 8];

k = k + 1;
tasks(k).id   = k;
tasks(k).type = "single";
tasks(k).src  = [3 1];
tasks(k).dst  = [10 2];

k = k + 1;
tasks(k).id   = k;
tasks(k).type = "single";
tasks(k).src  = [4 7];
tasks(k).dst  = [9 7];

% Cooperative (large-box) tasks – need 2 robots each
k = k + 1;
tasks(k).id   = k;
tasks(k).type = "coop";
tasks(k).src  = [6 6];
tasks(k).dst  = [12 6];

k = k + 1;
tasks(k).id   = k;
tasks(k).type = "coop";
tasks(k).src  = [6 3];
tasks(k).dst  = [12 3];

k = k + 1;
tasks(k).id   = k;
tasks(k).type = "coop";
tasks(k).src  = [8 1];
tasks(k).dst  = [13 1];

M = numel(tasks);

%% ------------ SHELVES AS OBSTACLES (VISUAL ONLY) -------------
shelf_width  = 0.8;
shelf_height = 0.4;
shelves      = [];

for t = 1:M
    xd = tasks(t).dst(1);
    yd = tasks(t).dst(2);

    xmin = xd - shelf_width/2;
    xmax = xd + shelf_width/2;
    ymin = yd + 0.3;
    ymax = ymin + shelf_height;

    shelves(end+1,:) = [xmin, xmax, ymin, ymax]; %#ok<SAGROW>
end

%% ------------------- COST MATRIX & TASK ALLOCATION ------------
C = computeCostMatrix(robots.pos, tasks);
allocation = allocateTasksGreedy(robots.pos, tasks, C);

disp('Single-robot task assignments [robot, task]:');
disp(allocation.single);
disp('Cooperative task assignments [robot_i, robot_j, task]:');
disp(allocation.coop);

%% -------------- SIMULATION PARAMETERS --------------
dt       = 0.05;    % time step (s)
T        = 15;      % total sim time (s)
steps    = round(T/dt);

v_single = 1.5;     % fast single robots
v_leader = 1.2;     % coop leader speed
v_max    = max(v_single, v_leader);

k_form   = 3.0;     % formation control gain
d_star   = 1.2;     % desired distance between coop robots
eps_pos  = 0.15;    % position tolerance

smooth_alpha = 0.4;

pos      = robots.pos;
pos_hist = zeros(steps,N,2);
u_prev   = zeros(N,2);

task_state = zeros(N, steps);     % for Gantt

%% -------- maps for quick lookup --------
singleTaskOf = nan(N,1);
for i = 1:size(allocation.single,1)
    r = allocation.single(i,1);
    t = allocation.single(i,2);
    singleTaskOf(r) = t;
end

coopTaskOf   = nan(N,1);
leaderOfTask = nan(M,1);
for i = 1:size(allocation.coop,1)
    r1 = allocation.coop(i,1);
    r2 = allocation.coop(i,2);
    t  = allocation.coop(i,3);
    coopTaskOf(r1) = t;
    coopTaskOf(r2) = t;
    leaderOfTask(t) = min(r1,r2);   % smaller index = leader
end

% singleStage(r): 0 = going to src, 1 = going to dst, 2 = done
singleStage = zeros(N,1);

% coopStage(t): 0=each to src, 1=carrying src->dst, 2=done
coopStage   = zeros(1,M);

% Track one cooperative task's formation error
trackedCoopTask = NaN;
if ~isempty(allocation.coop)
    trackedCoopTask = allocation.coop(1,3);
end
e_hist = zeros(steps,1);

%% ------------- MAIN SIMULATION LOOP -------------------
figure('Name','Warehouse simulation');
for k = 1:steps
    t_now = (k-1)*dt;
    u     = zeros(N,2);

    for r = 1:N
        % -------------------------------------------------
        % SINGLE-ROBOT TASK
        % -------------------------------------------------
        if ~isnan(singleTaskOf(r))
            t_id = singleTaskOf(r);
            src  = tasks(t_id).src;
            dst  = tasks(t_id).dst;

            if singleStage(r) == 0          % going to src
                target = src;
            elseif singleStage(r) == 1      % going to dst
                target = dst;
            else                            % done
                target = [];
            end

            if isempty(target)
                u(r,:) = [0 0];
                task_state(r,k) = 0;
            else
                if norm(pos(r,:) - target) <= eps_pos
                    % reached this target
                    if singleStage(r) == 0
                        singleStage(r) = 1;   % now go to dst
                    elseif singleStage(r) == 1
                        singleStage(r) = 2;   % task finished
                    end
                    u(r,:) = [0 0];
                    task_state(r,k) = 0;
                else
                    dir = target - pos(r,:);
                    u(r,:) = v_single * dir / (norm(dir)+1e-6);
                    task_state(r,k) = t_id;
                end
            end

        % -------------------------------------------------
        % COOPERATIVE TASK ROBOT
        % -------------------------------------------------
        elseif ~isnan(coopTaskOf(r))
            t_id = coopTaskOf(r);
            src  = tasks(t_id).src;
            dst  = tasks(t_id).dst;

            leader   = leaderOfTask(t_id);
            mates    = find(coopTaskOf == t_id);
            mates(mates == leader) = [];
            follower = mates;

            % ---- Stage 0: each robot individually goes to src ----
            if coopStage(t_id) == 0
                target = src;
                if norm(pos(r,:) - target) <= eps_pos
                    u(r,:) = [0 0];
                    task_state(r,k) = 0;
                else
                    dir = target - pos(r,:);
                    u(r,:) = v_single * dir / (norm(dir)+1e-6);
                    task_state(r,k) = t_id;
                end
                % once both at src, switch to coop carrying
                if norm(pos(leader,:)   - src) <= eps_pos && ...
                   norm(pos(follower,:) - src) <= eps_pos
                    coopStage(t_id) = 1;
                end

            % ---- Stage 1: cooperative carrying from src to dst ----
            elseif coopStage(t_id) == 1
                if r == leader
                    target = dst;
                    if norm(pos(r,:) - target) <= eps_pos
                        u(r,:) = [0 0];
                        task_state(r,k) = 0;
                        coopStage(t_id) = 2;   % done
                    else
                        dir = target - pos(r,:);
                        u(r,:) = v_leader * dir / (norm(dir)+1e-6);
                        task_state(r,k) = t_id;
                    end
                else
                    % follower: formation control relative to leader
                    pL = pos(leader,:);
                    pF = pos(follower,:);
                    dirL = dst - pL;
                    if norm(dirL) > 1e-6
                        uL   = v_leader * dirL / norm(dirL);
                    else
                        uL   = [0 0];
                    end
                    e    = (norm(pF - pL)^2 - d_star^2);
                    u(r,:) = uL - k_form * e * (pF - pL);
                    task_state(r,k) = t_id;
                end
            else
                % coopStage == 2 -> done
                u(r,:) = [0 0];
                task_state(r,k) = 0;
            end

        else
            % idle robot
            u(r,:) = [0 0];
            task_state(r,k) = 0;
        end
    end

    % --------- VELOCITY SATURATION + SMOOTHING ---------
    for r = 1:N
        speed = norm(u(r,:));
        if speed > v_max && speed > 0
            u(r,:) = v_max * u(r,:) / speed;
        end
        u(r,:) = (1-smooth_alpha)*u_prev(r,:) + smooth_alpha*u(r,:);
    end
    u_prev = u;

    % --- formation error for tracked coop task ---
    if ~isnan(trackedCoopTask) && coopStage(trackedCoopTask) >= 1
        idx = find(allocation.coop(:,3)==trackedCoopTask,1);
        r1 = allocation.coop(idx,1);
        r2 = allocation.coop(idx,2);
        d  = norm(pos(r2,:) - pos(r1,:));
        e_hist(k) = d^2 - d_star^2;
    else
        e_hist(k) = 0;
    end

    % integrate positions
    pos = pos + dt*u;
    pos_hist(k,:,:) = pos;

    %% --------- UPDATE BOX POSITIONS ---------
    boxPos = zeros(M,2);
    for t_id = 1:M
        if tasks(t_id).type == "single"
            row = find(allocation.single(:,2) == t_id, 1);
            if isempty(row)
                boxPos(t_id,:) = tasks(t_id).src;
            else
                rSingle = allocation.single(row,1);
                if singleStage(rSingle) == 0
                    boxPos(t_id,:) = tasks(t_id).src;
                elseif singleStage(rSingle) == 1
                    boxPos(t_id,:) = pos(rSingle,:);
                else
                    boxPos(t_id,:) = tasks(t_id).dst;
                end
            end
        else
            row = find(allocation.coop(:,3) == t_id, 1);
            if isempty(row)
                boxPos(t_id,:) = tasks(t_id).src;
            else
                r1 = allocation.coop(row,1);
                r2 = allocation.coop(row,2);
                if coopStage(t_id) == 0
                    boxPos(t_id,:) = tasks(t_id).src;
                elseif coopStage(t_id) == 1
                    boxPos(t_id,:) = 0.5 * (pos(r1,:) + pos(r2,:));
                else
                    boxPos(t_id,:) = tasks(t_id).dst;
                end
            end
        end
    end

   %% ------------- Live animation (no dotted circles) -----------------
clf; hold on; grid on; axis equal;
xlim([0 world_xmax]); ylim([0 world_ymax]);
title(sprintf('Warehouse – t = %.2f s', t_now));
xlabel('x'); ylabel('y');

% draw shelves (light grey)
for iS = 1:size(shelves,1)
    s = shelves(iS,:);
    rectangle('Position',[s(1), s(3), s(2)-s(1), s(4)-s(3)], ...
              'FaceColor',[0.9 0.9 0.9], 'EdgeColor',[0.5 0.5 0.5]);
end

% draw boxes (single = green square, coop = red circle)
for t_id = 1:M
    if tasks(t_id).type == "single"
        plot(boxPos(t_id,1), boxPos(t_id,2), 'ks', ...
            'MarkerSize',12, 'MarkerFaceColor',[0 0.8 0], ...
            'MarkerEdgeColor',[0 0.4 0], 'LineWidth',1.5);
    else
        plot(boxPos(t_id,1), boxPos(t_id,2), 'ko', ...
            'MarkerSize',12, 'MarkerFaceColor',[0.8 0 0], ...
            'MarkerEdgeColor',[0.4 0 0], 'LineWidth',1.5);
    end
end

% draw robots as blue dots
for r = 1:N
    plot(pos(r,1), pos(r,2), 'bo', ...
        'MarkerSize',8, 'MarkerFaceColor',[0 0.4 1]);
    text(pos(r,1)+0.2, pos(r,2), sprintf('R%d',r), ...
        'FontSize',8, 'Color',[0 0 0]);
end

% draw bars between robots carrying a box (optional – keep for coop)
if ~isempty(allocation.coop)
    for row = 1:size(allocation.coop,1)
        t_id = allocation.coop(row,3);
        if coopStage(t_id) >= 1   % currently carrying
            r1 = allocation.coop(row,1);
            r2 = allocation.coop(row,2);
            p1 = pos(r1,:); p2 = pos(r2,:);
            plot([p1(1) p2(1)], [p1(2) p2(2)], 'k-', 'LineWidth',2);
        end
    end
end

drawnow limitrate;

end

%% ----------------- FORMATION ERROR PLOT -----------------
if ~isnan(trackedCoopTask)
    figure;
    timeVec = (0:steps-1)*dt;
    plot(timeVec, e_hist, 'LineWidth',1.5);
    xlabel('time [s]');
    ylabel('formation error  e(t) = ||p_2-p_1||^2 - d_*^2');
    title('Cooperative carrying – formation error vs time');
    grid on;
end

%% ------------------- GANTT-LIKE PLOT --------------------
figure;
timeVec = (0:steps-1)*dt;
imagesc(timeVec, 1:N, task_state);
set(gca,'YDir','normal');
xlabel('time [s]');
ylabel('robot index');
title('Task execution (Gantt-like view)');
colorbar;
colormap(jet);

%% =============== LOCAL FUNCTIONS =========================

function C = computeCostMatrix(robotPos, tasks)
    N = size(robotPos,1);
    M = numel(tasks);
    C = inf(N,M);
    for r = 1:N
        for t = 1:M
            src = tasks(t).src;
            dst = tasks(t).dst;
            C(r,t) = norm(robotPos(r,:) - src) + norm(src - dst);
        end
    end
end

function allocation = allocateTasksGreedy(robotPos, tasks, C)
    N = size(robotPos,1);
    M = numel(tasks);

    types    = string({tasks.type});
    isSingle = (types == "single");
    isCoop   = (types == "coop");

    freeRobot = true(1,N);
    freeTask  = true(1,M);

    singleAssignments = [];
    coopAssignments   = [];

    % single tasks
    while any(freeRobot) && any(freeTask & isSingle)
        bestCost = inf; bestR = NaN; bestT = NaN;
        for r = 1:N
            if ~freeRobot(r), continue; end
            for t = 1:M
                if ~(freeTask(t) && isSingle(t)), continue; end
                if C(r,t) < bestCost
                    bestCost = C(r,t);
                    bestR = r; bestT = t;
                end
            end
        end
        if isfinite(bestCost)
            singleAssignments(end+1,:) = [bestR, bestT]; %#ok<AGROW>
            freeRobot(bestR) = false;
            freeTask(bestT)  = false;
        else
            break;
        end
    end

    % cooperative tasks
    while sum(freeRobot) >= 2 && any(freeTask & isCoop)
        bestCost = inf; bestPair = [NaN,NaN]; bestTask = NaN;
        freeRobotsIdx = find(freeRobot);
        freeCoopTasks = find(freeTask & isCoop);

        for ti = 1:numel(freeCoopTasks)
            t = freeCoopTasks(ti);
            for i = 1:numel(freeRobotsIdx)
                for j = i+1:numel(freeRobotsIdx)
                    ri = freeRobotsIdx(i);
                    rj = freeRobotsIdx(j);
                    pairCost = C(ri,t) + C(rj,t);
                    if pairCost < bestCost
                        bestCost = pairCost;
                        bestPair = [ri, rj];
                        bestTask = t;
                    end
                end
            end
        end
        if isfinite(bestCost)
            coopAssignments(end+1,:) = [bestPair, bestTask]; %#ok<AGROW>
            freeRobot(bestPair(1)) = false;
            freeRobot(bestPair(2)) = false;
            freeTask(bestTask)     = false;
        else
            break;
        end
    end

    allocation.single = singleAssignments;
    allocation.coop   = coopAssignments;
end