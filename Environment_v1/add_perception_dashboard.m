function add_perception_dashboard(sys)
% ADD_PERCEPTION_DASHBOARD  Adds the camera-view point cloud viewer and the
% perception score block to the open Simulink model.
%
%   add_perception_dashboard              % the model that is open / current
%   add_perception_dashboard('my_model')
%
% It adds three blocks next to the existing Perception (Python) block and wires
% them to the same signals it already uses (LiDAR cloud, detected cones, pose):
%
%   Perception Score     compares the detections with the true cones
%   Camera View 3D       chase-camera 3-D view of the cloud with cone models + score
%   Pose Select / Pose Vector   picks x, y, yaw out of the pose bus
%
% Safe to run again: it replaces its own blocks and leaves everything else alone.
% It does not save the model - check the diagram, then save as usual.

if nargin < 1, sys = bdroot; end
load_system(sys);
sys = getfullname(sys);

perc = [sys '/Perception (Python)'];
env  = [sys '/Environment'];
assert(~isempty(find_system(sys, 'SearchDepth', 1, 'Name', 'Perception (Python)')), ...
    'No "Perception (Python)" block in %s', sys);
assert(~isempty(find_system(sys, 'SearchDepth', 1, 'Name', 'Environment')), ...
    'No "Environment" subsystem in %s', sys);

% ---- the signals to tap
phP = get_param(perc, 'PortHandles');
cloudSrc = srcPort(phP.Inport(1));      % LiDAR cloud into the perception block
conesSrc = phP.Outport(5);              % detected cones
poseSrc  = srcPort(get_param(env, 'PortHandles').Inport(1));   % pose bus into the Environment

% ---- remove an earlier copy
names = {'Perception Score', 'Camera View 3D', 'Pose Select', 'Pose Vector'};
for k = 1:numel(names)
    b = [sys '/' names{k}];
    if ~isempty(find_system(sys, 'SearchDepth', 1, 'Name', names{k}))
        delete_line_of(b);
        delete_block(b);
    end
end

% ---- the 3-D camera, from the Scene Configuration block (defaults if not found)
camPos = [-6 0 2];  pitch = 15;  hfov = 60;
try
    scn = find_system(sys, 'SearchDepth', 2, 'Name', 'Simulation 3D Scene Configuration');
    camPos = eval(get_param(scn{1}, 'FreeCameraDefaultInitialTranslation'));
    crot   = eval(get_param(scn{1}, 'FreeCameraDefaultInitialRotation'));
    pitch  = -crot(2);
    hfov   = str2double(get_param(scn{1}, 'HorizontalFOV'));
catch err
    warning('add_perception_dashboard:defaults', ...
        'Could not read the camera settings (%s) - using defaults.', err.message);
end

% ---- blocks
posP = get_param(perc, 'Position');
x0 = posP(1);  y0 = posP(4) + 330;

add_block('simulink/Signal Routing/Bus Selector', [sys '/Pose Select'], ...
    'OutputSignals', 'x,y,yaw', 'Position', [x0 - 330 y0 x0 - 325 y0 + 60]);
add_block('simulink/Signal Routing/Mux', [sys '/Pose Vector'], ...
    'Inputs', '3', 'Position', [x0 - 250 y0 x0 - 245 y0 + 60]);

add_block('simulink/User-Defined Functions/MATLAB System', [sys '/Perception Score'], ...
    'System', 'PerceptionScore', 'SimulateUsing', 'Interpreted execution', ...
    'Position', [x0 y0 x0 + 210 y0 + 100]);
add_block('simulink/User-Defined Functions/MATLAB System', [sys '/Camera View 3D'], ...
    'System', 'CameraViewer3D', 'SimulateUsing', 'Interpreted execution', ...
    'CameraPos', mat2str(camPos), 'CameraPitch', num2str(pitch), ...
    'HorizontalFOV', num2str(hfov), 'Position', [x0 + 330 y0 x0 + 540 y0 + 125]);

% ---- wires
sc = get_param([sys '/Perception Score'],  'PortHandles');
vw = get_param([sys '/Camera View 3D'],    'PortHandles');
ps = get_param([sys '/Pose Select'],       'PortHandles');
pv = get_param([sys '/Pose Vector'],       'PortHandles');

add_line(sys, poseSrc, ps.Inport(1));
for k = 1:3
    add_line(sys, ps.Outport(k), pv.Inport(k));
end
add_line(sys, cloudSrc, sc.Inport(1));
add_line(sys, conesSrc, sc.Inport(2));
add_line(sys, pv.Outport(1), sc.Inport(3));

add_line(sys, cloudSrc, vw.Inport(1));
add_line(sys, conesSrc, vw.Inport(2));
add_line(sys, sc.Outport(2), vw.Inport(3));   % truth
add_line(sys, sc.Outport(3), vw.Inport(4));   % false positives
add_line(sys, sc.Outport(1), vw.Inport(5));   % stats

fprintf(['add_perception_dashboard: added Perception Score and Camera View 3D to %s\n' ...
    '  camera in the car frame: [%g %g %g] m, pitch %g deg, FOV %g deg\n' ...
    '  (the LiDAR position is found automatically from the first scan)\n'], ...
    sys, camPos, pitch, hfov);
end

function p = srcPort(inport)
% output port that drives the given input port
ln = get_param(inport, 'Line');
assert(ln > 0, 'An input port that should be connected is not.');
p = get_param(ln, 'SrcPortHandle');
end

function delete_line_of(blk)
% remove every wire attached to a block (so the block can be deleted cleanly)
ph = get_param(blk, 'PortHandles');
for p = [ph.Inport ph.Outport]
    ln = get_param(p, 'Line');
    if ln > 0, delete_line(ln); end
end
end
