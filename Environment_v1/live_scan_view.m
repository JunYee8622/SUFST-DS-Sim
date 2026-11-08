function live_scan_view(pc, pose)
% LIVE_SCAN_VIEW  Live top view of the simulated LiDAR during simulation.
% Called from the ScanViewer MATLAB Function block (as an extrinsic call).
%   pc    [nCh x nAz x 3] point cloud in the sensor frame
%   pose  [x y z yaw] car reference pose, world frame
% Reads lidarCfg, objects, objType and P from the base workspace.

persistent fig hGround hObj hCar lastTrack warned

cfg = evalin('base', 'lidarCfg');
trackName = evalin('base', 'P.trackName');

% (Re)create the figure if it was closed or the track changed
if isempty(fig) || ~isvalid(fig) || ~strcmp(lastTrack, trackName)
    if ~isempty(fig) && isvalid(fig), close(fig); end
    objects = evalin('base', 'objects');
    objType = evalin('base', 'objType');

    fig = figure('Name', 'Live LiDAR - top view');
    try theme(fig, 'light'); catch, set(fig, 'Color', 'w'); end
    hold on; axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
    isB = objType == "blue";  isY = objType == "yellow";
    isO = startsWith(objType, "orange"); isD = ~(isB | isY | isO);
    plot(objects(isB,1), objects(isB,2), 'o', 'MarkerFaceColor', [0 0.45 0.9],  'MarkerEdgeColor', 'k');
    plot(objects(isY,1), objects(isY,2), 'o', 'MarkerFaceColor', [1 0.8 0],     'MarkerEdgeColor', 'k');
    plot(objects(isO,1), objects(isO,2), 'o', 'MarkerFaceColor', [1 0.45 0],    'MarkerEdgeColor', 'k');
    plot(objects(isD,1), objects(isD,2), 's', 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor', 'k');
    hGround = plot(nan, nan, '.', 'Color', [0.75 0.75 0.75], 'MarkerSize', 3);
    hObj    = plot(nan, nan, 'r.', 'MarkerSize', 8);
    hCar    = plot(nan, nan, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'g');
    title(sprintf('%s - grey: ground returns, red: cone/object returns', trackName), ...
        'Interpreter', 'none');
    lastTrack = trackName;
end

% Accept either an [N x 3] list or an [nCh x nAz x 3] organised cloud
if size(pc, 2) == 3 && ismatrix(pc)
    pts = pc;
elseif ndims(pc) == 3 && size(pc, 3) == 3
    pts = reshape(pc, [], 3);
else
    if isempty(warned)
        fprintf(2, 'live_scan_view: unexpected point cloud size [%s] - skipping draw\n', ...
            num2str(size(pc)));
        warned = true;
    end
    return;
end
pts = pts(~any(isnan(pts), 2), :);
onObj = pts(:,3) > -cfg.mount(3) + 0.03;

yaw  = pose(4);
R    = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
sPos = pose(1:2) + (R * cfg.mount(1:2)')';
W    = (R * pts(:,1:2)')' + sPos;

g = W(~onObj, :);
g = g(1:5:end, :);                       % thin out ground points for speed
set(hGround, 'XData', g(:,1),        'YData', g(:,2));
set(hObj,    'XData', W(onObj,1),    'YData', W(onObj,2));
set(hCar,    'XData', sPos(1),       'YData', sPos(2));
drawnow limitrate;
end