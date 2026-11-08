%% sim_params.m - master parameter file (runs at project startup)
% Loads all simulation parameters, the chosen track and bus definitions
% into the base workspace for the Simulink model.

%% ===== 0. Simulation =====
P.dt   = 0.05;          % simulation step [s]
P.tEnd = 40;            % stop time [s]

%% ===== 1. Environment =====
% Track (change the name to switch event):
% 'acceleration' | 'skidpad' | 'autocross_01' | 'autocross_02' |
% 'autocross_03' | 'trackdrive_01'
P.trackName = 'autocross_01';
P.trackDir  = fullfile(currentProject().RootFolder, 'tracks');

T = readtable(fullfile(P.trackDir, [P.trackName '.csv']), 'TextType', 'string');
trackMeta = load(fullfile(P.trackDir, [P.trackName '_meta.mat']));

% Everything physically on track (what the LiDAR will see)
objects = [T.x, T.y];
objType = T.type;

% Track cones only (ground truth for perception - excludes distractors)
isCone   = ismember(objType, ["blue", "yellow", "orange_small", "orange_big"]);
cones    = objects(isCone, :);
coneType = objType(isCone);

% Object shapes for the LiDAR model: [rBottom rTop height] in metres.
% Cone bodies modelled as truncated cones (DS Table 1: small cone base
% 228 x 228 mm, 325 mm tall; big cone 285 x 285 mm, 505 mm tall).
% Body radii are approximate - the square base plate is ignored.
geom.blue             = [0.105 0.020 0.325];
geom.yellow           = [0.105 0.020 0.325];
geom.orange_small     = [0.105 0.020 0.325];
geom.orange_big       = [0.130 0.025 0.505];
geom.distractor_stack = [0.105 0.030 0.600];   % stack of spare cones (approximate)
geom.timekeeping      = [0.150 0.150 1.200];   % timekeeping equipment (approximate)
objGeom = zeros(size(objects, 1), 3);
for k = 1:size(objects, 1)
    objGeom(k, :) = geom.(char(objType(k)));
end

% LiDAR: Hesai Pandar40P (design elevations, datasheet Appendix I)
lidarCfg.elev = [15 11 8 5 3 2 5/3 4/3 1 2/3 1/3 0 -1/3 -2/3 -1 -4/3 -5/3 -2 -7/3 -8/3 ...
                 -3 -10/3 -11/3 -4 -13/3 -14/3 -5 -16/3 -17/3 -6 -7 -8 -9 -10 -11 -12 -13 -14 -19 -25];
lidarCfg.az       = -180 : 0.2 : 180 - 0.2;  % 0.2 deg at 10 Hz -> 1800 steps
lidarCfg.Ts       = 0.1;                     % scan period [s] (10 Hz)
lidarCfg.range    = 80;                      % sim range limit [m] (sensor: 200 m)
lidarCfg.minRange = 0.3;                     % [m] (datasheet)
lidarCfg.noiseStd = 0.02;                    % range noise [m] (datasheet: +/-2 cm)
lidarCfg.mount    = [1.0 0 0.9];             % PLACEHOLDER [x fwd, y left, z up] in car frame [m]
P.lidar = lidarCfg;                          % same values, for reference

% Start pose, taken from the track
P.pose0 = struct('x',    trackMeta.startPose(1), ...
                 'y',    trackMeta.startPose(2), ...
                 'z',    0, ...
                 'roll', 0, ...
                 'pitch',0, ...
                 'yaw',  trackMeta.startPose(3));

% Bus: VehiclePose (the Environment's input)
names = {'x', 'y', 'z', 'roll', 'pitch', 'yaw'};
elems = Simulink.BusElement.empty;
for i = 1:numel(names)
    elems(i) = Simulink.BusElement;
    elems(i).Name = names{i};
end
VehiclePose = Simulink.Bus;
VehiclePose.Elements = elems;

% Test driver: TEMPORARY stand-in for Vehicle dynamics. Moves the car
% along the true centreline at constant speed, for testing the Environment.
testDrive.centre = trackMeta.centre;
testDrive.ds     = 0.25;      % centre path spacing [m] (from generate_fs_tracks)
testDrive.speed  = 5;         % [m/s]
testDrive.closed = double(any(strcmp(trackMeta.event, {'autocross', 'trackdrive'})));

clear T names elems i k isCone geom   % tidy up temporary variables

fprintf('sim_params: track "%s" loaded (%d cones, %d other objects)\n', ...
    P.trackName, size(cones, 1), size(objects, 1) - size(cones, 1));

%% ===== 2. Perception =====          (later)

%% ===== 3. State estimation / SLAM =====  (later)

%% ===== 4. Planning & Control =====  (later)

%% ===== 5. Vehicle dynamics =====    (later - from VD group)

%% ===== 6. Evaluation =====          (later)