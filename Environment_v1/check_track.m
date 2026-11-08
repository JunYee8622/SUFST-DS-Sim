function report = check_track(name)
% check_track('autocross_01') - plots a track and checks it against the
% FS Driverless Specification 2026 (DS 1) track-marking rules.
% Run from the Command Window with a track name in quotes.
%
% Checks (track cones only - distractors and timekeeping excluded):
%   min gap between any two cones    > 0.5 m  (no overlapping cones)
%   max spacing, same-colour cones   <= 5.0 m (DS 1.1)
%   min track width                  >= 3.0 m (DS 1.1)
%   big orange cones present         (DS 1.1: before and after lines)
%   min radius of the driving path   reported

if nargin < 1
    error('Give a track name, e.g. check_track(''skidpad'')');
end
try
    proj = currentProject;
    d = fullfile(proj.RootFolder, 'tracks');
catch
    d = fullfile(fileparts(mfilename('fullpath')), 'tracks');
end
T = readtable(fullfile(d, [name '.csv']), 'TextType', 'string');
M = load(fullfile(d, [name '_meta.mat']));

isCone = ismember(T.type, ["blue", "yellow", "orange_small", "orange_big"]);
P = [T.x(isCone), T.y(isCone)];
n = size(P, 1);

%% Plot
fig = figure('Name', ['Track: ' name]);
try theme(fig, 'light'); catch, set(fig, 'Color', 'w'); end
hold on; axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
plot(M.centre(:,1), M.centre(:,2), '-', 'Color', [0.75 0.75 0.75]);
styles = {"blue",             'o', [0 0.45 0.9],  6;
          "yellow",           'o', [1 0.8 0],     6;
          "orange_small",     'o', [1 0.55 0],    6;
          "orange_big",       'o', [1 0.35 0],    10;
          "distractor_stack", 's', [0.5 0.5 0.5], 9;
          "timekeeping",      '^', [0.85 0 0],    10};
h = gobjects(0); lab = strings(0);
for i = 1:size(styles, 1)
    m = T.type == styles{i,1};
    if any(m)
        h(end+1) = plot(T.x(m), T.y(m), styles{i,2}, 'MarkerFaceColor', styles{i,3}, ...
            'MarkerEdgeColor', 'k', 'MarkerSize', styles{i,4}); %#ok<AGROW>
        lab(end+1) = strrep(styles{i,1}, "_", " "); %#ok<AGROW>
    end
end
sp = M.startPose;
quiver(sp(1), sp(2), 3*cos(sp(3)), 3*sin(sp(3)), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
legend(h, lab, 'Location', 'bestoutside');
title(sprintf('%s  (%s, %d lap(s))', name, M.event, M.nLaps), 'Interpreter', 'none');

%% Checks
% 1. Minimum gap between any two track cones
D = sqrt((P(:,1) - P(:,1)').^2 + (P(:,2) - P(:,2)').^2);
D(1:n+1:end) = inf;
report.minConeGap = min(D(:));

% 2. Max nearest-neighbour spacing within blue and within yellow
report.maxSpacing = 0;
for c = ["blue", "yellow"]
    Q = [T.x(T.type == c), T.y(T.type == c)];
    if size(Q, 1) > 1
        Dq = sqrt((Q(:,1) - Q(:,1)').^2 + (Q(:,2) - Q(:,2)').^2);
        Dq(1:size(Q,1)+1:end) = inf;
        report.maxSpacing = max(report.maxSpacing, max(min(Dq, [], 2)));
    end
end

% 3. Track width along the path: nearest blue + nearest yellow
B = [T.x(T.type == "blue"),   T.y(T.type == "blue")];
Y = [T.x(T.type == "yellow"), T.y(T.type == "yellow")];
C = M.centre(1:4:end, :);
if ~isempty(B) && ~isempty(Y)
    dB = min(sqrt((C(:,1) - B(:,1)').^2 + (C(:,2) - B(:,2)').^2), [], 2);
    dY = min(sqrt((C(:,1) - Y(:,1)').^2 + (C(:,2) - Y(:,2)').^2), [], 2);
    report.minWidth = min(dB + dY);
else
    report.minWidth = NaN;
end

% 4. Big orange cones
report.nBigOrange = nnz(T.type == "orange_big");

% 5. Minimum radius of the driving path (3-point circle, 1 m apart)
a = M.centre(1:end-8, :); b = M.centre(5:end-4, :); c3 = M.centre(9:end, :);
cr = abs((b(:,1)-a(:,1)).*(c3(:,2)-b(:,2)) - (b(:,2)-a(:,2)).*(c3(:,1)-b(:,1)));
k  = 2*cr ./ (vecnorm(b-a,2,2) .* vecnorm(c3-b,2,2) .* vecnorm(c3-a,2,2));
report.minRadius = 1 / max(k);

%% Print
labels = ["FAIL", "PASS"];
pf = @(ok) labels(double(ok) + 1);
fprintf('\n=== %s ===\n', name);
fprintf('Objects: %d blue, %d yellow, %d small orange, %d big orange, %d distractors, %d timekeeping\n', ...
    nnz(T.type=="blue"), nnz(T.type=="yellow"), nnz(T.type=="orange_small"), ...
    nnz(T.type=="orange_big"), nnz(T.type=="distractor_stack"), nnz(T.type=="timekeeping"));
fprintf('Min cone gap:         %.2f m   %s\n', report.minConeGap, pf(report.minConeGap > 0.5));
fprintf('Max same-colour gap:  %.2f m   %s\n', report.maxSpacing, pf(report.maxSpacing <= 5.0));
fprintf('Min track width:      %.2f m   %s\n', report.minWidth,   pf(report.minWidth >= 2.95));
fprintf('Big orange cones:     %d       %s\n', report.nBigOrange, pf(report.nBigOrange >= 4));
fprintf('Min path radius:      %.2f m\n',     report.minRadius);
end