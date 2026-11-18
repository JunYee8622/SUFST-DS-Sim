classdef LidarPostProcess < matlab.System
    % LIDARPOSTPROCESS  Makes the simulated point cloud behave like the real Pandar40P's.
    %
    % The Simulation 3D Lidar block takes an instantaneous snapshot: every beam
    % is fired at the same moment from the same place. A real spinning LiDAR
    % sweeps once round in Ts = 0.1 s, so each azimuth is measured at a
    % different time, from a different place, while the car keeps moving.
    % The driver publishes the points as measured, without correcting for that,
    % so a moving car sees a smeared cloud. This block adds:
    %
    %   1. MOTION DISTORTION   each column is re-expressed in the sensor frame
    %      at the moment its beam fires (tau = fraction of the sweep x Ts),
    %      using the car's speed and yaw rate. At 15 m/s the sensor travels
    %      1.5 m during one sweep, so cones swept late appear 1.5 m closer
    %      than cones swept early, and the sweep's start/end seam tears.
    %   2. MIN / MAX RANGE     points outside become NaN (no return); this also
    %      removes the returns from the car's own nose a few cm from the sensor.
    %   3. RANGE NOISE         Gaussian, along each beam.
    %
    % Inputs   ptCloud [M x N x 3], reflectivity [M x N], pose [x y yaw] (world, rad)
    % Outputs  ptCloud       distorted, noisy cloud - what perception gets
    %          reflectivity  matching the cloud
    %          pointTime     [M x N] firing time of each point in the sweep [s]
    %          ptRef         the instantaneous (undistorted) cloud, used ONLY by
    %                        the scoring block to decide which cones were visible
    %          ego           [vx; vy; yaw rate] of the car during the sweep, body frame
    %                        [m/s, m/s, rad/s] - what wheel-speed / IMU odometry gives
    %                        perception so it can de-skew the cloud
    %
    % Model limits: static world (true for cones); roll, pitch and vibration of
    % the car are ignored; the car's twist is taken from the pose change over the
    % sweep (constant during it). Points keep the column of the snapshot beam
    % they came from, so the beam pattern is not re-gridded.

    properties (Nontunable)
        MinRange      = 0.3     % [m]  datasheet minimum range
        MaxRange      = 80      % [m]  simulation range limit
        NoiseStd      = 0       % [m]  range noise, 1 sigma (0 = off)
        Ts            = 0.1     % [s]  scan period
        Az            = -180 : 0.2 : 179.8   % azimuth of each column [deg] (ISO: 0 front, +90 left)
        ScanStartAz   = 180     % [deg] azimuth where each sweep starts (180 = behind the car)
        SpinDir       = -1      % -1 = clockwise seen from above (Hesai), +1 = anticlockwise
        SensorOffset  = [1.2 0] % [x y] sensor position in the car frame, from the pose reference [m]
        Distortion    = 1       % 1 = motion distortion on, 0 = off (instantaneous snapshot)
        PoseLag       = 1       % scans: Unreal delivers each scan this many scan periods after the pose (measured)
    end

    properties (Access = private)
        Hist           % [3 x (PoseLag+1)] recent poses, oldest first
        HasHist = false
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Hist = zeros(3, max(obj.PoseLag, 1) + 1);
            obj.HasHist = false;
        end

        function resetImpl(obj)
            obj.Hist = zeros(3, max(obj.PoseLag, 1) + 1);
            obj.HasHist = false;
        end

        function [ptOut, reflOut, tPoint, ptRef, ego] = stepImpl(obj, pt, refl, pose)
            pose = double(pose(:));
            assert(numel(pose) == 3, ['LidarPostProcess: input 3 "pose" must be a 3-element ' ...
                'vector [x y yaw], but it has %d element(s). Wire the Pose Vector (Mux of ' ...
                'x, y, yaw) output to this port.'], numel(pose));
            [nRow, nCol, ~] = size(pt);

            %% firing time of every column within the sweep
            az  = reshape(double(obj.Az), 1, []);
            tau = mod(obj.SpinDir * (az - obj.ScanStartAz), 360) / 360 * obj.Ts;   % 1 x N
            tPoint = single(repmat(tau, nRow, 1));

            %% the car's twist in its own frame while the sweep happens.
            % The scan in hand was taken at the OLDEST pose in the history (Unreal
            % delivers it PoseLag scans late); the car then moves to the next pose.
            if ~obj.HasHist
                obj.Hist = repmat(pose, 1, max(obj.PoseLag, 1) + 1);
                obj.HasHist = true;
            end
            obj.Hist = [obj.Hist(:, 2:end), pose];
            p0 = obj.Hist(:, 1);  p1 = obj.Hist(:, 2);
            dpos = p1(1:2) - p0(1:2);
            dyaw = atan2(sin(p1(3) - p0(3)), cos(p1(3) - p0(3)));
            ym   = p0(3) + dyaw / 2;
            vb   = [cos(ym) sin(ym); -sin(ym) cos(ym)] * (dpos / obj.Ts);
            vx = vb(1); vy = vb(2); om = dyaw / obj.Ts;
            ego = [vx; vy; om];                              % what odometry / IMU would report

            x = pt(:, :, 1);  y = pt(:, :, 2);  z = pt(:, :, 3);

            %% reference (instantaneous) cloud for the scorer: min/max range only
            r0   = sqrt(x.^2 + y.^2 + z.^2);
            bad0 = ~isfinite(r0) | r0 < obj.MinRange | r0 > obj.MaxRange;
            xr = x; yr = y; zr = z;
            xr(bad0) = NaN; yr(bad0) = NaN; zr(bad0) = NaN;
            ptRef = cat(3, xr, yr, zr);

            %% 1. motion distortion
            if obj.Distortion ~= 0 && (vx ~= 0 || vy ~= 0 || om ~= 0)
                wt = om * tau;
                c = cos(wt); s = sin(wt);
                if abs(om) > 1e-6
                    dRx = (vx * s - vy * (1 - c)) / om;
                    dRy = (vx * (1 - c) + vy * s) / om;
                else
                    dRx = vx * tau;
                    dRy = vy * tau;
                end
                % sensor displacement since the sweep start (frame at tau = 0);
                % the lever arm makes the sensor swing when the car yaws
                sx = obj.SensorOffset(1); sy = obj.SensorOffset(2);
                dx = dRx + (c - 1) * sx - s * sy;
                dy = dRy + s * sx + (c - 1) * sy;

                ax = double(x) - dx;                         % implicit expansion over rows
                ay = double(y) - dy;
                x = single( c .* ax + s .* ay);              % rotate into the frame at time tau
                y = single(-s .* ax + c .* ay);
            end

            %% 3. range noise (along each beam), then 2. min / max range
            r = sqrt(x.^2 + y.^2 + z.^2);
            if obj.NoiseStd > 0
                k = 1 + single(obj.NoiseStd) * randn(size(r), 'like', r) ./ max(r, eps('single'));
                x = x .* k;  y = y .* k;  z = z .* k;
                r = r .* k;
            end
            bad = ~isfinite(r) | r < obj.MinRange | r > obj.MaxRange | isnan(pt(:, :, 1));
            x(bad) = NaN;  y(bad) = NaN;  z(bad) = NaN;
            ptOut = cat(3, x, y, z);

            reflOut = refl;
            reflOut(bad) = 0;
        end

        %% sample time: one scan per Ts regardless of the faster pose input
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end

        function num = getNumInputsImpl(~),  num = 3; end
        function num = getNumOutputsImpl(~), num = 5; end
        function [n1, n2, n3] = getInputNamesImpl(~)
            n1 = 'ptCloud'; n2 = 'reflectivity'; n3 = 'pose';
        end
        function [n1, n2, n3, n4, n5] = getOutputNamesImpl(~)
            n1 = 'ptCloud'; n2 = 'reflectivity'; n3 = 'pointTime'; n4 = 'ptRef'; n5 = 'ego';
        end
        function [s1, s2, s3, s4, s5] = getOutputSizeImpl(obj)
            s1 = propagatedInputSize(obj, 1);
            s2 = propagatedInputSize(obj, 2);
            s3 = propagatedInputSize(obj, 2);
            s4 = propagatedInputSize(obj, 1);
            s5 = [3 1];
        end
        function [t1, t2, t3, t4, t5] = getOutputDataTypeImpl(~)
            t1 = 'single'; t2 = 'single'; t3 = 'single'; t4 = 'single'; t5 = 'double';
        end
        function [c1, c2, c3, c4, c5] = isOutputComplexImpl(~)
            c1 = false; c2 = false; c3 = false; c4 = false; c5 = false;
        end
        function [f1, f2, f3, f4, f5] = isOutputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true; f4 = true; f5 = true;
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header('LidarPostProcess', ...
                'Title', 'LiDAR Post-Processing', ...
                'Text', ['Sweep motion distortion, min/max range and range noise, as on the ' ...
                         'real spinning Pandar40P.']);
        end
    end
end
