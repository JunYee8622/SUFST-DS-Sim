%% view_lidar_scan.m - one simulated LiDAR scan at the track's start pose
% Runs lidar_raycast directly (no Simulink) and shows:
%   Figure 1: the 3D point cloud as the LiDAR sees it (sensor frame)
%   Figure 2: top view of the track with the scan overlaid (world frame)
% and prints how many LiDAR points land on each track cone within 15 m.
% Run sim_params first (it runs automatically when the project opens).
%
% Tip: point markers have a fixed size on screen, so zooming in spreads
% them out rather than making them bigger. Use Rotate (not Zoom) to look
% around Figure 1.

if ~exist('lidarCfg', 'var') || ~exist('objects', 'var')
    sim_params;
end

pose = [P.pose0.x, P.pose0.y, P.pose0.z, P.pose0.yaw];

tic;
pc = lidar_raycast(pose, lidarCfg, objects, objGeom);
fprintf('Scan computed in %.2f s\n', toc);

pts = reshape(pc, [], 3);
pts = pts(~any(isnan(pts), 2), :);

% Points above the ground = hits on cones and other objects
onObj = pts(:,3) > -lidarCfg.mount(3) + 0.03;
fprintf('%d points: %d on the ground, %d on cones/objects\n', ...
    size(pts,1), nnz(~onObj), nnz(onObj));

% World-frame coordinates of every point
yaw  = pose(4);
R    = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
sPos = pose(1:2) + (R * lidarCfg.mount(1:2)')';
ptsW = [(R * pts(:,1:2)')' + sPos, pts(:,3) + lidarCfg.mount(3) + pose(3)];  % z = height above ground

%% Figure 1: 3D point cloud, sensor frame
f1 = figure('Name', 'LiDAR scan - sensor frame');
try theme(f1, 'light'); catch, set(f1, 'Color', 'w'); end
plot3(pts(~onObj,1), pts(~onObj,2), pts(~onObj,3), '.', 'Color', [0.5 0.5 0.5], 'MarkerSize', 3); hold on;
plot3(pts(onObj,1),  pts(onObj,2),  pts(onObj,3),  '.', 'Color', [0.85 0 0],   'MarkerSize', 10);
axis equal; grid on;
xlabel('x forward [m]'); ylabel('y left [m]'); zlabel('z up [m]');
title(sprintf('%s: grey = ground, red = cones/objects', P.trackName), 'Interpreter', 'none');
xlim([-25 25]); ylim([-25 25]);
view(-60, 30);

%% Figure 2: top view, world frame
f2 = figure('Name', 'LiDAR scan - top view');
try theme(f2, 'light'); catch, set(f2, 'Color', 'w'); end
plot(ptsW(~onObj,1), ptsW(~onObj,2), '.', 'Color', [0.75 0.75 0.75], 'MarkerSize', 3); hold on;
isB = objType == "blue";  isY = objType == "yellow";
isO = startsWith(objType, "orange"); isD = ~(isB | isY | isO);
plot(objects(isB,1), objects(isB,2), 'o', 'MarkerFaceColor', [0 0.45 0.9],  'MarkerEdgeColor', 'k');
plot(objects(isY,1), objects(isY,2), 'o', 'MarkerFaceColor', [1 0.8 0],     'MarkerEdgeColor', 'k');
plot(objects(isO,1), objects(isO,2), 'o', 'MarkerFaceColor', [1 0.45 0],    'MarkerEdgeColor', 'k');
plot(objects(isD,1), objects(isD,2), 's', 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor', 'k');
plot(ptsW(onObj,1),  ptsW(onObj,2),  'r.', 'MarkerSize', 8);
plot(sPos(1), sPos(2), 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'g');
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
title('Top view: grey = ground returns, red = object returns, star = sensor');

%% Points per track cone (printed only - quick sanity check)
isTrack = isB | isY;
rel  = objects - sPos;
dObj = vecnorm(rel, 2, 2);
for k = find(isTrack & dObj < 15)'
    near = vecnorm(ptsW(:,1:2) - objects(k,:), 2, 2) < objGeom(k,1) + 0.15 & ptsW(:,3) > 0.01;
    fprintf('  %-6s cone at %5.1f m: %3d points\n', objType(k), dObj(k), nnz(near));
end