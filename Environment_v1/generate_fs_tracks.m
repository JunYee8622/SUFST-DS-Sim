%% generate_fs_tracks.m - builds the FS driverless track library
% Follows the FS Driverless Specification 2026, DS 1 (track marking):
%   - blue = left border, yellow = right border
%   - small orange = entry/exit lanes and stop areas
%   - big orange = before AND after start, finish and timekeeping lines
%   - max 5 m between cones in driving direction, tighter in corners
%   - min track width 3 m
%   - DS 1.4: spare cone stacks and timekeeping equipment near the track
% Layout dimensions marked CHECK come from FS Rules chapter D
% (D 5.1 acceleration, D 4.2.5 skidpad, D 8.1 trackdrive). Verify them
% against the current rules before relying on them.
%
% For each track, writes into the project's tracks/ folder:
%   <name>.csv       columns x, y, type
%   <name>_meta.mat  centre (driving path, in order), startPose [x y yaw],
%                    event, nLaps, width
% Types: blue | yellow | orange_small | orange_big |
%        distractor_stack | timekeeping
clear; clc;
rng(1);                          % reproducible distractor placement
trackDir = getTrackDir();
if ~exist(trackDir, 'dir'), mkdir(trackDir); end

%% Settings
W           = 3.0;    % track width [m]                       (DS 1.1: min 3 m)
SP_MAX      = 4.5;    % cone spacing on straights [m]         (DS 1.1: max 5 m)
SP_MIN      = 2.0;    % tightest cone spacing in corners [m]
ds          = 0.25;   % centre path resolution [m]
BIG_ALONG   = 0.6;    % big orange this far before/after a line [m]
TK_OFFSET   = 2.5;    % timekeeping equipment distance outside the border [m]
DISTRACTORS = true;   % add spare cone stacks + timekeeping equipment (DS 1.4)

%% 1. Acceleration (D 5.1)
L_ACC  = 75;          % start line to finish line [m]          (CHECK D 5.1)
L_STOP = 75;          % stop area after the finish line [m]    (CHECK D 5.1)
STAGE  = 0.3;         % staging line behind the start line [m] (CHECK D 5.1)

x      = (-STAGE:ds:L_ACC + L_STOP)';
centre = [x, zeros(size(x))];
t = [1 0]; n = [0 1];

xs       = (5:5:L_ACC - 5)';
C.blue   = [xs,  W/2*ones(size(xs))];
C.yellow = [xs, -W/2*ones(size(xs))];

C.orange_big = [bigAtLine([0 0],     t, n, BIG_ALONG, W/2);
                bigAtLine([L_ACC 0], t, n, BIG_ALONG, W/2)];

% Stop area: small orange along both sides ~5 m apart, end wall <= 1 m gaps
xsS   = (L_ACC + 5:5:L_ACC + L_STOP - 5)';
wallY = linspace(-W/2, W/2, ceil(W/1.0) + 1)';
C.orange_small = [xsS,  W/2*ones(size(xsS));
                  xsS, -W/2*ones(size(xsS));
                  (L_ACC + L_STOP)*ones(size(wallY)), wallY];

if DISTRACTORS
    C.timekeeping      = [tkAtLine([0 0], n, W/2 + TK_OFFSET);
                          tkAtLine([L_ACC 0], n, W/2 + TK_OFFSET)];
    C.distractor_stack = [30, W/2 + 6; 110, -(W/2 + 6)];
end

saveTrack(trackDir, 'acceleration', C, centre, [-STAGE 0 0], 'acceleration', 1, W);
clear C

%% 2. Skidpad (D 4.2.5)
R_PATH = 9.125;       % driving-path radius, centres 18.25 m apart (CHECK D 4.2.5)
R_IN   = 7.625;       % inner cone circle radius, 15.25 m diameter  (CHECK D 4.2.5)
R_OUT  = 10.625;      % outer cone circle radius, 21.25 m diameter  (CHECK D 4.2.5)
N_IN   = 16;          % inner cones per circle                      (CHECK D 4.2.5)
L_IN   = 20;          % entry lane: start position to crossing [m]
L_OUT  = 20;          % exit lane: crossing to stop area [m]
L_STOPS = 20;         % stop area length [m]

cR = [0 -R_PATH];     % right circle centre, driven clockwise first
cL = [0  R_PATH];     % left circle centre, driven anticlockwise second

% Driving path: entry, right circle x2, left circle x2, exit + stop area
xe    = (-L_IN:ds:-ds)';
nC    = round(2*pi*R_PATH/ds);
th    = linspace(pi/2, pi/2 - 4*pi, 2*nC + 1)';    th(end)  = [];
th2   = linspace(-pi/2, -pi/2 + 4*pi, 2*nC + 1)';  th2(end) = [];
xx    = (0:ds:L_OUT + L_STOPS)';
centre = [xe, zeros(size(xe));
          cR + R_PATH*[cos(th)  sin(th)];
          cL + R_PATH*[cos(th2) sin(th2)];
          xx, zeros(size(xx))];

% Circle cones
aIn    = linspace(0, 2*pi, N_IN + 1)'; aIn(end) = [];
innerR = cR + R_IN*[cos(aIn) sin(aIn)];          % right circle: inner = yellow
innerL = cL + R_IN*[cos(aIn) sin(aIn)];          % left circle:  inner = blue
nOut   = ceil(2*pi*R_OUT/SP_MAX);
aOut   = linspace(0, 2*pi, nOut + 1)'; aOut(end) = [];
outerR = cR + R_OUT*[cos(aOut) sin(aOut)];       % right circle: outer = blue
outerL = cL + R_OUT*[cos(aOut) sin(aOut)];       % left circle:  outer = yellow
% Remove outer cones lying in the other circle's lane or in the straight lane
outerR = outerR(vecnorm(outerR - cL, 2, 2) > R_OUT + 0.5 & abs(outerR(:,2)) > W/2 + 0.3, :);
outerL = outerL(vecnorm(outerL - cR, 2, 2) > R_OUT + 0.5 & abs(outerL(:,2)) > W/2 + 0.3, :);

C.blue   = [innerL; outerR];
C.yellow = [innerR; outerL];

% Big orange before and after the timekeeping line at the crossing
C.orange_big = bigAtLine([0 0], [1 0], [0 1], 1.2, W/2 + 0.1);

% Entry/exit lanes (small orange), kept clear of both circles' lanes
xl   = [(-L_IN:3:-3)'; (3:3:L_OUT)'];
lane = [xl,  W/2*ones(size(xl)); xl, -W/2*ones(size(xl))];
ok   = vecnorm(lane - cL, 2, 2) > R_OUT + 0.3 & vecnorm(lane - cR, 2, 2) > R_OUT + 0.3;
% Stop area: sides ~5 m apart, end wall <= 1 m gaps
xsS   = (L_OUT + 5:5:L_OUT + L_STOPS - 5)';
wallY = linspace(-W/2, W/2, ceil(W/1.0) + 1)';
C.orange_small = [lane(ok,:);
                  xsS,  W/2*ones(size(xsS));
                  xsS, -W/2*ones(size(xsS));
                  (L_OUT + L_STOPS)*ones(size(wallY)), wallY];

if DISTRACTORS
    C.timekeeping      = tkAtLine([0 0], [0 1], W/2 + 2.0);   % inside the circle islands
    C.distractor_stack = [-L_IN + 2, W/2 + 5; L_OUT + 10, -(W/2 + 5)];
end

saveTrack(trackDir, 'skidpad', C, centre, [-L_IN 0 0], 'skidpad', 4, W);
clear C

%% 3. Autocross (D 8.1 layout style) - three closed layouts
shapes = { @(t) 25 + 5*sin(3*t), ...
           @(t) 22 + 6*sin(4*t + 0.5), ...
           @(t) 28 + 4*sin(2*t) + 3*cos(5*t) };
for k = 1:numel(shapes)
    tt = linspace(0, 2*pi, 4001)'; tt(end) = [];
    r  = shapes{k}(tt);
    cl = [1.2*r.*cos(tt), r.*sin(tt)];
    [centre, nrm, tang] = resampleClosed(cl, ds);
    N  = size(centre, 1);

    % Curvature-adaptive cone spacing: tighter in corners, and the outer
    % border gap never exceeds 4.9 m
    Rloc = localRadius(centre, round(2/ds));
    idx  = 1; i = 1;
    while true
        R  = Rloc(i);
        sp = min([SP_MAX, max(SP_MIN, 0.35*R), 4.9*R/(R + W/2)]);
        i  = i + max(1, round(sp/ds));
        if i > N - round(0.5*sp/ds), break; end
        idx(end+1) = i; %#ok<SAGROW>
    end
    C.blue   = centre(idx,:) + W/2*nrm(idx,:);
    C.yellow = centre(idx,:) - W/2*nrm(idx,:);

    % Start/finish line at centre(1): big orange before and after, both sides
    C.orange_big = bigAtLine(centre(1,:), tang(1,:), nrm(1,:), BIG_ALONG, W/2);
    C.blue   = removeNear(C.blue,   C.orange_big, 1.0);
    C.yellow = removeNear(C.yellow, C.orange_big, 1.0);

    if DISTRACTORS
        C.timekeeping = tkAtLine(centre(1,:), nrm(1,:), W/2 + TK_OFFSET);
        % A few spare cone stacks well outside the track
        cand = [];
        for j = randperm(N, 12)
            side = sign(randn);
            p = centre(j,:) + side*(W/2 + 6)*nrm(j,:);
            if min(vecnorm(centre - p, 2, 2)) > W/2 + 4
                cand(end+1,:) = p; %#ok<AGROW>
            end
            if size(cand,1) >= 3, break; end
        end
        C.distractor_stack = cand;
    end

    iS = N - round(6/ds);                        % start position 6 m before the line
    startPose = [centre(iS,:), atan2(tang(iS,2), tang(iS,1))];

    name = sprintf('autocross_%02d', k);
    saveTrack(trackDir, name, C, centre, startPose, 'autocross', 1, W);

    %% 4. Trackdrive (D 8.1): same layout as autocross_01, 10 laps
    if k == 1
        saveTrack(trackDir, 'trackdrive_01', C, centre, startPose, 'trackdrive', 10, W);
    end
    clear C
end

fprintf('Track library written to %s\n', trackDir);

%% ===== Local functions =====

function trackDir = getTrackDir()
% tracks/ folder in the root of the open Simulink Project
try
    proj = currentProject;
    trackDir = fullfile(proj.RootFolder, 'tracks');
catch
    trackDir = fullfile(fileparts(mfilename('fullpath')), 'tracks');
    warning('No project open - writing tracks to %s', trackDir);
end
end

function B = bigAtLine(p, t, n, along, lateral)
% Four big orange cones: before and after a line, on both sides.
% p = point on the line, t = driving direction, n = left normal.
B = [p + along*t + lateral*n;
     p - along*t + lateral*n;
     p + along*t - lateral*n;
     p - along*t - lateral*n];
end

function T = tkAtLine(p, n, lateral)
% Timekeeping equipment on both sides of a line
T = [p + lateral*n; p - lateral*n];
end

function A = removeNear(A, B, d)
% Remove points of A closer than d to any point of B
if isempty(A) || isempty(B), return; end
D = sqrt((A(:,1) - B(:,1)').^2 + (A(:,2) - B(:,2)').^2);
A = A(min(D, [], 2) >= d, :);
end

function [centre, nrm, tang] = resampleClosed(cl, ds)
% Resample a closed curve at equal spacing; unit tangents, left normals
clClosed = [cl; cl(1,:)];
s      = [0; cumsum(vecnorm(diff(clClosed), 2, 2))];
sq     = (0:ds:s(end)-ds)';
centre = interp1(s, clClosed, sq);
tang   = [centre(2:end,:); centre(1,:)] - [centre(end,:); centre(1:end-1,:)];
tang   = tang ./ vecnorm(tang, 2, 2);
nrm    = [-tang(:,2), tang(:,1)];
end

function R = localRadius(centre, m)
% Radius of curvature at each point of a closed path (3-point circle,
% neighbours m samples away). Straight sections give Inf.
N = size(centre, 1);
a = centre(mod((0:N-1) - m, N) + 1, :);
c = centre(mod((0:N-1) + m, N) + 1, :);
b = centre;
cr = abs((b(:,1)-a(:,1)).*(c(:,2)-b(:,2)) - (b(:,2)-a(:,2)).*(c(:,1)-b(:,1)));
k  = 2*cr ./ (vecnorm(b-a,2,2) .* vecnorm(c-b,2,2) .* vecnorm(c-a,2,2));
R  = 1 ./ k;
end

function saveTrack(trackDir, name, C, centre, startPose, event, nLaps, W)
types = ["blue", "yellow", "orange_small", "orange_big", "distractor_stack", "timekeeping"];
x = []; y = []; type = strings(0, 1);
for tp = types
    f = char(tp);
    if isfield(C, f) && ~isempty(C.(f))
        x    = [x;    C.(f)(:,1)];                     %#ok<AGROW>
        y    = [y;    C.(f)(:,2)];                     %#ok<AGROW>
        type = [type; repmat(tp, size(C.(f),1), 1)];   %#ok<AGROW>
    end
end
writetable(table(x, y, type), fullfile(trackDir, [name '.csv']));

meta.centre    = centre;
meta.startPose = startPose;
meta.event     = event;
meta.nLaps     = nLaps;
meta.width     = W;
save(fullfile(trackDir, [name '_meta.mat']), '-struct', 'meta');
fprintf('  %-15s %4d objects\n', name, numel(x));
end