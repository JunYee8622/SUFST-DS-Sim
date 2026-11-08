% RUN_FS_SIM  FS Driverless 2D sim - ONE file, press Run (or type run_fs_sim).
% Edit the settings below. Everything after the line of ===== is the sim itself.
%
% Note: this file must NOT be called fs_driverless_2d_sim.m (MATLAB rule:
% a script cannot contain a function with the same name as the file).

clc; clear; close all;

% ------------------------- SETTINGS ---------------------------------------
trackFolder = 'C:\UoS\Year 4\Formula Student';      % use single quotes
trackFiles  = { fullfile(trackFolder, 'acceleration_track.csv'), ...
                fullfile(trackFolder, 'skidpad.csv') };
animate = true;                                        % false = fast, no animation
% --------------------------------------------------------------------------

results = fs_driverless_2d_sim(trackFiles, animate);

% ==========================================================================
%  SIMULATOR - no need to edit below this line
% ==========================================================================
function results = fs_driverless_2d_sim(trackFiles, animate)
% FS_DRIVERLESS_2D_SIM  2D Formula Student Driverless simulator driven by track files.
%
%   fs_driverless_2d_sim                        % runs acceleration.csv AND skidpad.csv
%   fs_driverless_2d_sim('skidpad.csv')         % one track file
%   fs_driverless_2d_sim({'a.csv','b.csv'})     % several track files
%   fs_driverless_2d_sim('pick')                % choose file(s) with a dialog
%   fs_driverless_2d_sim([], false)             % no animation (fast)
%
% Track file format (CSV, header row, EUFS-compatible - extra columns ignored):
%   tag,x,y        tags: blue, yellow, orange, big_orange, car_start
%   Frame: ISO 8855 (FS Driverless Spec 2026, Fig. 5) - x forward, y left, metres.
%
% The event is detected from the file name ('skid'/'accel') or the cone layout.
% The driving line is built ONLY from the imported cones:
%   acceleration : straight line through blue/yellow midpoints into the stop area
%   skidpad      : circles fitted to the cone rings, 2 laps right + 2 laps left
%   other        : blue/yellow midpoint centreline (e.g. trackdrive)
% Then a kinematic bicycle model drives it with pure pursuit + speed profile,
% and the sim reports event time and cones hit.
%
% Plain MATLAB only - no toolboxes.

    if nargin < 1 || isempty(trackFiles)
        trackFiles = {'acceleration.csv', 'skidpad.csv'};
    end
    if nargin < 2, animate = true; end
    if ischar(trackFiles) && strcmpi(trackFiles, 'pick')
        [f, p] = uigetfile('*.csv', 'Select track file(s)', 'MultiSelect', 'on');
        if isequal(f, 0), results = []; return; end
        trackFiles = fullfile(p, cellstr(f));
    end
    if ischar(trackFiles), trackFiles = {trackFiles}; end
    if isstring(trackFiles), trackFiles = cellstr(trackFiles); end   % "double-quoted" paths
    trackFiles = cellfun(@char, trackFiles, 'UniformOutput', false);

    P = vehicleParams();
    nT = numel(trackFiles);
    fig = [];
    if animate
        fig = figure('Name', 'FS Driverless 2D Sim', 'Position', [100 80 1000 800]);
        % Force a light look. Newer MATLAB (R2025a+) follows the desktop dark
        % theme, which gives dark axes with grey text on a white figure.
        try, set(fig, 'Theme', 'light'); catch, end    % ignored on older versions
        set(fig, 'Color', 'w', ...
            'DefaultAxesColor', 'w', 'DefaultAxesXColor', 'k', ...
            'DefaultAxesYColor', 'k', 'DefaultAxesGridColor', [0.15 0.15 0.15], ...
            'DefaultTextColor', 'k');
    end

    results = struct([]);
    for i = 1:nT
        file  = trackFiles{i};
        cones = importTrack(file);
        event = detectEvent(file, cones);
        trk   = buildPath(event, cones);
        ax = [];
        if animate
            ax = subplot(nT, 1, i, 'Parent', fig);
            drawTrack(ax, cones, trk.path, file);
        end
        r = simulate(trk, cones, P, ax);
        r.file = file; r.event = event;
        printResult(r);
        if isempty(results), results = r; else, results(i) = r; end
    end
end

% =====================================================================
%  Parameters
% =====================================================================
function P = vehicleParams()
    P.L        = 1.53;          % wheelbase [m]
    P.width    = 1.40;          % car width [m]
    P.length   = 2.90;          % car length [m]
    P.maxSteer = deg2rad(25);   % max steering angle [rad]
    P.vmax     = 25;            % top speed [m/s]
    P.ayMax    = 12;            % lateral accel limit [m/s^2]
    P.axMax    = 8;             % max acceleration [m/s^2]
    P.brakeMax = 10;            % max braking [m/s^2]
    P.dt       = 0.01;          % time step [s]
    P.tMax     = 120;           % safety stop [s]
    P.coneR    = 0.114;         % small cone base half-width [m] (Spec Table 1: 228 mm)
end

% =====================================================================
%  Import
% =====================================================================
function c = importTrack(file)
    fid = fopen(file, 'r');
    if fid < 0
        error(['Cannot open track file "%s".\nPut it in the current folder ' ...
               '(%s) or give the full path.'], file, pwd);
    end
    fgetl(fid);                                    % skip header row
    tag = {}; xy = zeros(0, 2);
    line = fgetl(fid);
    while ischar(line)
        parts = strsplit(strtrim(line), ',');
        if numel(parts) >= 3
            tag{end+1, 1} = lower(strtrim(parts{1}));          %#ok<AGROW>
            xy(end+1, :)  = [str2double(parts{2}), str2double(parts{3})]; %#ok<AGROW>
        end
        line = fgetl(fid);
    end
    fclose(fid);
    c.blue      = xy(strcmp(tag, 'blue'), :);
    c.yellow    = xy(strcmp(tag, 'yellow'), :);
    c.orange    = xy(strcmp(tag, 'orange'), :);
    c.bigOrange = xy(strcmp(tag, 'big_orange'), :);
    s = xy(strcmp(tag, 'car_start'), :);
    if isempty(s), s = [0 0]; end
    c.start = s(1, :);
    if isempty(c.blue) || isempty(c.yellow)
        error('Track "%s" has no blue/yellow cones - check the tag column.', file);
    end
    fprintf('Imported %s: %d blue, %d yellow, %d orange, %d big orange\n', file, ...
        size(c.blue,1), size(c.yellow,1), size(c.orange,1), size(c.bigOrange,1));
end

function event = detectEvent(file, c)
    [~, name] = fileparts(file); name = lower(name);
    if ~isempty(strfind(name, 'skid'))
        event = 'skidpad';
    elseif ~isempty(strfind(name, 'accel'))
        event = 'acceleration';
    else
        % geometry fallback: blue cones on both sides of the start line -> skidpad
        allC = [c.blue; c.yellow];
        u = trackDirection(c);  nrm = [-u(2) u(1)];
        lat = (allC - c.start) * nrm';
        if any(lat > 5) && any(lat < -5) && rangeOf(c.blue * nrm') > 15
            event = 'skidpad';
        elseif rangeOf([c.blue; c.yellow] * nrm') < 6
            event = 'acceleration';
        else
            event = 'trackdrive';
        end
    end
end

function u = trackDirection(c)
    % driving direction: from car start towards the timing line / nearest cones
    if ~isempty(c.bigOrange)
        tgt = mean(c.bigOrange(nearestCluster(c.bigOrange, c.start), :), 1);
    else
        tgt = mean([c.blue; c.yellow], 1);
    end
    d = tgt - c.start;
    if norm(d) < 0.5                     % start is on the line: use cone midpoints
        d = mean([c.blue; c.yellow], 1) - c.start;
    end
    u = d / norm(d);
end

function idx = nearestCluster(p, from)
    % big orange cones come in groups per line; return the group nearest 'from'
    dd = hypot(p(:,1) - from(1), p(:,2) - from(2));
    idx = dd < min(dd) + 2.5;
end

% =====================================================================
%  Driving line from cones
% =====================================================================
function trk = buildPath(event, c)
    ds = 0.1;
    u = trackDirection(c);  nrm = [-u(2) u(1)];
    trk.u = u; trk.n = nrm; trk.event = event;

    switch event
        case 'acceleration'
            % line through blue/yellow midpoints, from start into the stop area
            mids = pairMidpoints(c.blue, c.yellow);
            off  = mean((mids - c.start) * nrm');           % lateral offset of centreline
            p0   = c.start + off * nrm;
            allC = [c.blue; c.yellow; c.orange; c.bigOrange];
            sEnd = max((allC - c.start) * u') - 5;          % stop 5 m before end wall
            s = (0:ds:sEnd)';
            trk.path = [c.start; p0 + s * u];
            % timing lines from big orange groups along the track
            sb = (c.bigOrange - c.start) * u';
            trk.sStart = mean(sb(sb < median(sb)));
            trk.sFinish = mean(sb(sb >= median(sb)));
            trk.origin = c.start;

        case 'skidpad'
            T = mean(c.bigOrange(nearestCluster(c.bigOrange, c.start + 20*u), :), 1);
            if isempty(c.bigOrange), T = mean([c.blue; c.yellow], 1); end
            sideB = (c.blue   - T) * nrm';
            sideY = (c.yellow - T) * nrm';
            % left circle: inner = blue (left side), outer = yellow (left side)
            [cL1, rL1] = fitCircle(c.blue(sideB > 0, :));
            [cL2, rL2] = fitCircle(c.yellow(sideY > 0, :));
            % right circle: inner = yellow (right side), outer = blue (right side)
            [cR1, rR1] = fitCircle(c.yellow(sideY < 0, :));
            [cR2, rR2] = fitCircle(c.blue(sideB < 0, :));
            CL = (cL1 + cL2) / 2;  CR = (cR1 + cR2) / 2;
            rL = norm(T - CL);     rR = norm(T - CR);       % pass through timing point
            fprintf('  fitted circles: left r_in %.2f / r_out %.2f, right r_in %.2f / r_out %.2f m\n', ...
                rL1, rL2, rR1, rR2);

            entryLen = norm(T - c.start);
            entry = c.start + (0:ds:entryLen)' * u;
            a0 = atan2(T(2) - CR(2), T(1) - CR(1));
            th = (a0 : -ds/rR : a0 - 4*pi)';                % 2 laps clockwise (right)
            right = [CR(1) + rR*cos(th), CR(2) + rR*sin(th)];
            a0 = atan2(T(2) - CL(2), T(1) - CL(1));
            th = (a0 : ds/rL : a0 + 4*pi)';                 % 2 laps anticlockwise (left)
            left = [CL(1) + rL*cos(th), CL(2) + rL*sin(th)];
            sEnd = max(([c.orange; c.bigOrange] - T) * u') - 4;   % stop before end wall
            exitP = T + (ds:ds:sEnd)' * u;
            trk.path = [entry; right(2:end,:); left(2:end,:); exitP];
            trk.origin = T; trk.sStart = 0; trk.sFinish = 0;
            trk.CL = CL; trk.CR = CR;

        otherwise   % trackdrive / autocross: midpoint centreline
            mids = pairMidpoints(c.blue, c.yellow);
            order = zeros(size(mids,1), 1); used = false(size(order)); p = c.start;
            for i = 1:numel(order)
                dd = sum((mids - p).^2, 2); dd(used) = inf;
                [~, j] = min(dd); order(i) = j; used(j) = true; p = mids(j,:);
            end
            pts = [c.start; mids(order,:)];
            s = [0; cumsum(hypot(diff(pts(:,1)), diff(pts(:,2))))];
            [s, keep] = unique(s); pts = pts(keep,:);
            sq = (0:ds:s(end))';
            trk.path = [interp1(s, pts(:,1), sq), interp1(s, pts(:,2), sq)];
            trk.origin = c.start; trk.sStart = 0; trk.sFinish = 0;
    end
    trk.heading0 = atan2(trk.path(2,2) - trk.path(1,2), trk.path(2,1) - trk.path(1,1));
end

function mids = pairMidpoints(blue, yellow)
    mids = zeros(size(blue));
    for i = 1:size(blue, 1)
        [~, j] = min(sum((yellow - blue(i,:)).^2, 2));
        mids(i,:) = (blue(i,:) + yellow(j,:)) / 2;
    end
end

function [ctr, r] = fitCircle(p)
    % algebraic least-squares circle fit (Kasa)
    if size(p, 1) < 3, error('Need at least 3 cones to fit a skidpad circle.'); end
    A = [2*p(:,1), 2*p(:,2), ones(size(p,1), 1)];
    b = p(:,1).^2 + p(:,2).^2;
    sol = A \ b;
    ctr = sol(1:2)';
    r = sqrt(sol(3) + sum(ctr.^2));
end

% =====================================================================
%  Simulation
% =====================================================================
function r = simulate(trk, c, P, ax)
    path = trk.path;  nPath = size(path, 1);
    vRef = speedProfile(path, P);

    x = path(1,1); y = path(1,2); psi = trk.heading0; v = 0; idx = 1;
    nSteps = round(P.tMax / P.dt);
    log = zeros(nSteps, 6);                          % t x y psi v delta
    allCones = [c.blue; c.yellow; c.orange; c.bigOrange];
    hit = false(size(allCones, 1), 1);

    if ~isempty(ax)
        hTrail = plot(ax, x, y, 'r-', 'LineWidth', 1.2);
        hCar   = patch(ax, 'XData', 0, 'YData', 0, 'FaceColor', [0.2 0.2 0.2]);
        baseTitle = get(get(ax, 'Title'), 'String');
        % MATLAB can return the title as a cell array or string object - force plain text
        if iscell(baseTitle), baseTitle = strjoin(baseTitle, ' '); end
        baseTitle = char(baseTitle);
    end

    k = 0;
    for k = 1:nSteps
        t = (k - 1) * P.dt;

        win = idx:min(idx + 200, nPath);             % progress: search forward only
        [~, j] = min((path(win,1) - x).^2 + (path(win,2) - y).^2);
        idx = win(j);

        Ld = max(2, 0.25 * v);                       % pure pursuit lookahead
        look = idx;
        while look < nPath && hypot(path(look,1) - x, path(look,2) - y) < Ld
            look = look + 1;
        end
        alpha = atan2(path(look,2) - y, path(look,1) - x) - psi;
        delta = atan2(2 * P.L * sin(alpha), Ld);
        delta = max(-P.maxSteer, min(P.maxSteer, delta));

        iv = min(nPath, idx + round(max(v, 1) * 0.3 / 0.1));   % speed target 0.3 s ahead
        a = max(-P.brakeMax, min(P.axMax, 3 * (vRef(iv) - v)));

        v   = max(0, v + a * P.dt);                  % kinematic bicycle model
        x   = x + v * cos(psi) * P.dt;
        y   = y + v * sin(psi) * P.dt;
        psi = psi + v / P.L * tan(delta) * P.dt;
        log(k,:) = [t, x, y, psi, v, delta];

        rel = allCones - [x y];                      % cone inside car footprint?
        lon =  rel(:,1) * cos(psi) + rel(:,2) * sin(psi);
        lat = -rel(:,1) * sin(psi) + rel(:,2) * cos(psi);
        hit = hit | (abs(lon) < P.length/2 + P.coneR & abs(lat) < P.width/2 + P.coneR);

        if ~isempty(ax) && mod(k, 10) == 0 && ishandle(ax)
            set(hTrail, 'XData', log(1:k,2), 'YData', log(1:k,3));
            [cx, cy] = carOutline(x, y, psi, P);
            set(hCar, 'XData', cx, 'YData', cy);
            title(ax, sprintf('%s  |  t = %.2f s  v = %.0f km/h  cones hit %d', ...
                baseTitle, t, v*3.6, nnz(hit)), 'Interpreter', 'none', 'Color', 'k');
            drawnow;
        end

        if idx >= nPath - 2 && v < 0.1, break; end
    end
    log = log(1:k, :);

    r.log       = log;
    r.conesHit  = nnz(hit);
    r.hitXY     = allCones(hit, :);
    r.totalTime = log(end, 1);
    r.eventTime = eventTime(trk, log);
    r.topSpeed  = max(log(:,5)) * 3.6;

    if ~isempty(ax) && ishandle(ax)
        set(hTrail, 'XData', log(:,2), 'YData', log(:,3));
        title(ax, sprintf('%s  |  %s time %.3f s  |  cones hit %d', ...
            baseTitle, trk.event, r.eventTime, r.conesHit), 'Interpreter', 'none', 'Color', 'k');
        if r.conesHit > 0
            plot(ax, r.hitXY(:,1), r.hitXY(:,2), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
        end
    end
end

function vRef = speedProfile(path, P)
    n = size(path, 1);
    ds = [hypot(diff(path(:,1)), diff(path(:,2))); 0.1];
    kappa = zeros(n, 1);                             % curvature, 3-point circle
    for i = 6:n-5
        a = path(i-5,:); b = path(i,:); cc = path(i+5,:);
        A = norm(b - a); B = norm(cc - b); C = norm(cc - a);
        area2 = abs((b(1)-a(1))*(cc(2)-a(2)) - (b(2)-a(2))*(cc(1)-a(1)));
        if A*B*C > 1e-9, kappa(i) = 2 * area2 / (A*B*C); end
    end
    vRef = min(P.vmax, sqrt(P.ayMax ./ max(kappa, 1e-6)));
    vRef(end) = 0;
    for i = n-1:-1:1                                 % braking limit (80 % margin)
        vRef(i) = min(vRef(i), sqrt(vRef(i+1)^2 + 2 * 0.8 * P.brakeMax * ds(i)));
    end
    vRef = max(vRef, 0.5);
    vRef(end-5:end) = 0;
end

function T = eventTime(trk, log)
    t = log(:,1); p = log(:,2:3);
    s   = (p - trk.origin) * trk.u';                 % distance along track direction
    lat = (p - trk.origin) * trk.n';
    switch trk.event
        case 'acceleration'
            k0 = find(s >= trk.sStart, 1); k1 = find(s >= trk.sFinish, 1);
            if isempty(k0) || isempty(k1), T = NaN; else, T = t(k1) - t(k0); end
        case 'skidpad'
            cross = find(s(1:end-1) < 0 & s(2:end) >= 0 & abs(lat(2:end)) < 3);
            if numel(cross) >= 5                     % entry + 4 laps
                tc = t(cross);
                T = mean([tc(3) - tc(2), tc(5) - tc(4)]);   % mean of timed laps 2 and 4
            else
                T = NaN;
            end
        otherwise
            T = t(end);
    end
end

% =====================================================================
%  Output
% =====================================================================
function printResult(r)
    fprintf('=== %s (%s) ===\n', upper(r.event), r.file);
    fprintf('  Event time : %.3f s\n', r.eventTime);
    fprintf('  Total time : %.2f s (incl. stopping)\n', r.totalTime);
    fprintf('  Top speed  : %.1f km/h\n', r.topSpeed);
    fprintf('  Cones hit  : %d\n\n', r.conesHit);
end

function drawTrack(ax, c, path, name)
    hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
    set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.15 0.15 0.15]);
    [~, fn, ext] = fileparts(name); name = [fn ext];    % show file name, not full path
    plot(ax, path(:,1), path(:,2), ':', 'Color', [0.6 0.6 0.6]);
    sc = @(p, col, sz) scatter(ax, p(:,1), p(:,2), sz, col, 'filled', '^');
    if ~isempty(c.blue),      sc(c.blue,      [0.1 0.3 0.9], 30); end
    if ~isempty(c.yellow),    sc(c.yellow,    [0.95 0.8 0],  30); end
    if ~isempty(c.orange),    sc(c.orange,    [1 0.55 0],    30); end
    if ~isempty(c.bigOrange), sc(c.bigOrange, [1 0.3 0],     60); end
    xlabel(ax, 'x forward [m]'); ylabel(ax, 'y left [m]');
    title(ax, name, 'Interpreter', 'none', 'Color', 'k');
end

function [cx, cy] = carOutline(x, y, psi, P)
    l = P.length/2; w = P.width/2;
    body = [l 0; l*0.6 w; -l w; -l -w; l*0.6 -w];
    R = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    pts = (R * body')';
    cx = pts(:,1) + x; cy = pts(:,2) + y;
end

function r = rangeOf(v)
    r = max(v) - min(v);           % (built-in range() needs the Statistics Toolbox)
end