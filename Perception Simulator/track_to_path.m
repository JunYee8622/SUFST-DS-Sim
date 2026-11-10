function path = track_to_path(trackName, v, opts)
% TRACK_TO_PATH  Constant-speed driving path along the track centre,
% built from the <name>_meta.mat files written by generate_fs_tracks.m.
%
%   path = track_to_path('autocross_01');          % 10 m/s (default)
%   path = track_to_path('skidpad', 8);
%   path = track_to_path('trackdrive_01', 12, struct('runOut', 15));
%
% Uses meta.centre (the generator's ordered driving path), so it follows the
% exact middle of the blue/yellow borders on every event, including skidpad.
%   acceleration, skidpad  : centre path as generated (start -> stop area)
%   autocross, trackdrive  : start at meta.startPose, drive meta.nLaps laps,
%                            then opts.runOut metres past the finish line
%
% Simulink: three From Workspace blocks -> path.X, path.Y, path.Yaw
%           model Stop time = path.tEnd
%
% opts fields (all optional):
%   yawInDegrees  true   - check what your vehicle block's Yaw input expects
%   runOut        10     - metres driven past the finish line (closed tracks)
%   plot          true

if nargin < 2 || isempty(v), v = 10; end
if nargin < 3, opts = struct(); end
o = struct('yawInDegrees', true, 'runOut', 10, 'plot', true);
f = fieldnames(opts);
for k = 1:numel(f), o.(f{k}) = opts.(f{k}); end
assert(v > 0, 'Speed must be positive.');

% ---------- load the track ----------
trackDir = getTrackDir();
meta = load(fullfile(trackDir, [trackName '_meta.mat']));
c = meta.centre;
closed = any(strcmp(meta.event, {'autocross', 'trackdrive'}));

% ---------- build the full path ----------
if closed
    % startPose sits before the line at centre(1); drive to the line,
    % then nLaps full laps, then run out past the line
    [~, iS] = min(vecnorm(c - meta.startPose(1:2), 2, 2));
    segLen  = [0; cumsum(vecnorm(diff(c), 2, 2))];
    nRun    = find(segLen >= o.runOut, 1);
    if isempty(nRun), nRun = size(c, 1); end
    pts = [c(iS:end, :); repmat(c, meta.nLaps, 1); c(1:nRun, :)];
else
    pts = c;
end
pts = pts([true; vecnorm(diff(pts), 2, 2) > 1e-6], :);   % drop duplicate points

% ---------- arc length, time, heading ----------
s   = [0; cumsum(vecnorm(diff(pts), 2, 2))];
t   = s / v;
yaw = unwrap(atan2(gradient(pts(:,2)), gradient(pts(:,1))));

% ---------- sanity checks ----------
dYaw = rad2deg(angdiff(yaw(1), meta.startPose(3)));
if abs(dYaw) > 5
    warning('Path heading at start differs from meta.startPose by %.1f deg.', dYaw);
end
kappa = movmean(gradient(yaw) ./ max(gradient(s), eps), 9);
aLat  = v^2 * abs(kappa);
fprintf('%s (%s): %.1f m, %.1f s at %.1f m/s, peak lateral accel %.1f m/s^2 (%.2f g)\n', ...
        trackName, meta.event, s(end), t(end), v, max(aLat), max(aLat)/9.81);

% ---------- outputs ----------
if o.yawInDegrees, yawOut = rad2deg(yaw); else, yawOut = yaw; end
path.X     = timeseries(pts(:,1), t);
path.Y     = timeseries(pts(:,2), t);
path.Yaw   = timeseries(yawOut, t);
path.tEnd  = t(end);
path.xy    = pts;
path.s     = s;
path.t     = t;
path.v     = v;
path.aLat  = aLat;
path.track = trackName;
path.meta  = meta;

% ---------- plot ----------
if o.plot
    cones = readtable(fullfile(trackDir, [trackName '.csv']), 'TextType', 'string');
    style = struct('blue', {{'o', [0 0.3 1]}}, 'yellow', {{'o', [0.9 0.75 0]}}, ...
                   'orange_small', {{'o', [1 0.5 0]}}, 'orange_big', {{'s', [1 0.4 0]}}, ...
                   'distractor_stack', {{'x', [0.4 0.4 0.4]}}, 'timekeeping', {{'^', [0 0 0]}});
    figure('Name', ['track_to_path: ' trackName]);
    subplot(1, 2, 1); hold on; axis equal; grid on
    for tp = fieldnames(style)'
        m = cones.type == tp{1};
        if any(m)
            st = style.(tp{1});
            plot(cones.x(m), cones.y(m), st{1}, 'Color', st{2}, 'MarkerFaceColor', st{2}, ...
                 'DisplayName', strrep(tp{1}, '_', ' '));
        end
    end
    plot(pts(:,1), pts(:,2), 'k-', 'DisplayName', 'path');
    plot(pts(1,1), pts(1,2), 'g*', 'MarkerSize', 12, 'DisplayName', 'start');
    legend('Location', 'bestoutside'); xlabel('X [m]'); ylabel('Y [m]');
    title(sprintf('%s, %.0f m/s', strrep(trackName, '_', '\_'), v));
    subplot(1, 2, 2);
    plot(t, aLat / 9.81); grid on
    xlabel('t [s]'); ylabel('lateral accel [g]'); title('Lateral acceleration (check only)');
end
end

function trackDir = getTrackDir()
% Same folder logic as generate_fs_tracks.m
try
    proj = currentProject;
    trackDir = fullfile(proj.RootFolder, 'tracks');
catch
    trackDir = fullfile(fileparts(mfilename('fullpath')), 'tracks');
end
end
