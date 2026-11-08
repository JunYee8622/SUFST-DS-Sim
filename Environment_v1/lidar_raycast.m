function pc = lidar_raycast(pose, cfg, objects, objGeom)
%#codegen
% LIDAR_RAYCAST  Simulated spinning LiDAR scan of a flat track with cones.
% Usable from MATLAB and from a Simulink MATLAB Function block.
%
%   pose     [x y z yaw]   car reference point, world frame (yaw in rad)
%   cfg      struct:
%              elev      [1 x nCh] channel elevations [deg]
%              az        [1 x nAz] azimuth angles [deg], evenly spaced from -180
%              range     max range [m]
%              minRange  min range [m]
%              mount     [x y z] sensor position in the car frame [m]
%              noiseStd  range noise, 1 sigma [m]
%   objects  [n x 2]       object positions x, y (world frame)
%   objGeom  [n x 3]       [rBottom rTop height] of each object [m]
%                          (cones as truncated cones, rBottom = rTop for a cylinder)
%
%   pc       [nCh x nAz x 3] organised point cloud in the SENSOR frame
%            (x forward, y left, z up). NaN = no return.
%
% Model: ground is the plane z = 0; objects are vertical truncated cones
% standing on it. Ignores roll/pitch, the car's own body, beam divergence,
% intensity and per-channel azimuth offsets.

nCh = numel(cfg.elev);
nAz = numel(cfg.az);
pc  = nan(nCh, nAz, 3);
rngT = inf(nCh, nAz);                    % closest hit distance per beam

yaw = pose(4);
cyw = cos(yaw); syw = sin(yaw);

% Sensor position in the world
sx = pose(1) + cyw*cfg.mount(1) - syw*cfg.mount(2);
sy = pose(2) + syw*cfg.mount(1) + cyw*cfg.mount(2);
sz = pose(3) + cfg.mount(3);

ce = cosd(cfg.elev); se = sind(cfg.elev);
azRes = 360 / nAz;

%% 1. Ground plane (z = 0): each downward beam hits at the same range all round
for i = 1:nCh
    if se(i) < 0
        t = -sz / se(i);
        if t <= cfg.range
            rngT(i, :) = t;
        end
    end
end

%% 2. Objects: only test the beams whose azimuth can reach each object
for k = 1:size(objects, 1)
    ox = objects(k,1) - sx;
    oy = objects(k,2) - sy;
    dist = sqrt(ox^2 + oy^2);
    rb = objGeom(k,1); rt = objGeom(k,2); h = objGeom(k,3);
    if dist > cfg.range + rb || dist < 0.05
        continue;
    end
    kk  = (rb - rt) / h;                 % radius shrinks by kk per metre of height
    phi = atan2d(oy, ox) - yaw*180/pi;   % object azimuth in the sensor frame
    w   = asind(min(1, (rb + 0.05)/dist)) + azRes;
    j0  = floor((phi - w - cfg.az(1)) / azRes);
    j1  = ceil((phi + w - cfg.az(1)) / azRes);

    % Ray origin relative to the object's base centre
    px = -ox; py = -oy; pz = sz;
    A0 = rb - kk*pz;

    for jj = j0:j1
        j  = mod(jj, nAz) + 1;
        cA = cosd(cfg.az(j)) * cyw - sind(cfg.az(j)) * syw;   % cos(az + yaw)
        sA = sind(cfg.az(j)) * cyw + cosd(cfg.az(j)) * syw;   % sin(az + yaw)
        for i = 1:nCh
            dx = ce(i)*cA; dy = ce(i)*sA; dz = se(i);
            B0 = -kk*dz;
            qa = dx^2 + dy^2 - B0^2;
            qb = 2*(px*dx + py*dy - A0*B0);
            qc = px^2 + py^2 - A0^2;
            tHit = inf;
            if abs(qa) < 1e-12
                if abs(qb) > 1e-12
                    t = -qc/qb;
                    z = pz + t*dz;
                    if t > 0 && z >= 0 && z <= h
                        tHit = t;
                    end
                end
            else
                disc = qb^2 - 4*qa*qc;
                if disc >= 0
                    sq = sqrt(disc);
                    tA = (-qb - sq) / (2*qa);
                    tB = (-qb + sq) / (2*qa);
                    t1 = min(tA, tB); t2 = max(tA, tB);
                    z1 = pz + t1*dz; z2 = pz + t2*dz;
                    if t1 > 0 && z1 >= 0 && z1 <= h
                        tHit = t1;
                    elseif t2 > 0 && z2 >= 0 && z2 <= h
                        tHit = t2;
                    end
                end
            end
            if tHit < rngT(i, j) && tHit <= cfg.range
                rngT(i, j) = tHit;
            end
        end
    end
end

%% 3. Convert ranges to points in the sensor frame, with range noise
for i = 1:nCh
    for j = 1:nAz
        t = rngT(i, j);
        if isfinite(t) && t >= cfg.minRange
            t = t + cfg.noiseStd * randn;
            pc(i, j, 1) = t * ce(i) * cosd(cfg.az(j));
            pc(i, j, 2) = t * ce(i) * sind(cfg.az(j));
            pc(i, j, 3) = t * se(i);
        end
    end
end
end