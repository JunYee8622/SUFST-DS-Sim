classdef PerceptionScore < matlab.System
    % PERCEPTIONSCORE  Grades the perception block against the true cone positions.
    %
    % Every scan it compares the detected cones with the track's real cones
    % (cones / coneType from sim_params.m) and keeps running totals. When the
    % simulation stops it prints a report and stores it in the base workspace
    % as  perceptionScoreReport.
    %
    % Inputs
    %   ptCloud  [M x N x 3]    LiDAR scan, sensor frame (x fwd, y left, z up)
    %   cones    [K x 4]        detections [x y z class], sensor frame, NaN = unused
    %   pose     [3 x 1]        car reference pose [x; y; yaw], world frame, yaw in rad
    %
    % Outputs
    %   stats    [1 x 6]        [cones in range, cones visible, TP, FP, FN, LiDAR x offset]
    %   truth    [T x 4]        true cones in range, sensor frame: [x y class state]
    %                           state 1 = detected, 0 = visible but MISSED,
    %                                 2 = too few LiDAR points to be fair
    %   fp       [K x 2]        [x y] of detections that match no cone (false positives)
    %
    % Scoring rules (the same as perception/evaluate.py):
    %   * a cone counts if it is within ScoreRange and hit by >= MinHits points;
    %   * detections are matched to those cones one-to-one, gate MatchGate;
    %     matched = TP, unmatched detection = FP, unmatched cone = FN.
    %
    % Where is the LiDAR? The truth is in world coordinates, the detections in
    % the sensor frame, so the block needs the LiDAR position relative to the
    % pose point. The Unreal "Front bumper" mount does not report it, so by
    % default it is found automatically on the first usable scan: the block tries
    % x offsets from -2 m to +5 m and keeps the one where the most true cones
    % line up with LiDAR returns (this uses the LiDAR points only, not the
    % detector, so it cannot flatter the detector). If almost no cone lines up
    % at any offset it warns: the pose and the scene then disagree (mirrored Y,
    % wrong yaw units, wrong track).

    properties (Nontunable)
        ScoreRange   = 30          % only cones / detections closer than this count [m]
        MinHits      = 3           % LiDAR points a cone needs to count as visible
        HitRadius    = 0.25        % point-to-cone distance that counts as a hit [m]
        GroundTol    = 0.04        % points above ground + this are objects [m]
        MatchGate    = 0.5         % detection and cone further apart never match [m]
        AutoCalibrate = true       % find the LiDAR x offset from the first scan
        SensorOffset = [1.3 0]     % LiDAR [x fwd, y left] from the pose point [m]; fixed value if AutoCalibrate is off
        MaxTruth     = 100         % size of the truth output
        MaxCones     = 150         % size of the cones input
    end

    properties (Access = private)
        Off            % LiDAR offset in use [x y]
        Calibrated = false
        TruthXY        % [n x 2] true cones, world frame
        TruthCls       % [n x 1] 0 blue, 1 yellow, 2 orange
        DistXY         % [m x 2] distractors (spare cones, timekeeping), world frame
        Track
        T              % running totals
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.TruthXY  = double(evalin('base', 'cones'));
            ct           = string(evalin('base', 'coneType'));
            obj.TruthCls = double(ct == "yellow") + 2 * double(startsWith(ct, "orange"));
            allXY        = double(evalin('base', 'objects'));
            allType      = string(evalin('base', 'objType'));
            obj.DistXY   = allXY(~ismember(allType, ["blue" "yellow" "orange_small" "orange_big"]), :);
            try obj.Track = char(evalin('base', 'P.trackName')); catch, obj.Track = '?'; end
            obj.T = struct('scans', 0, 'visible', 0, 'tp', 0, 'fp', 0, 'fn', 0, ...
                'distFP', 0, 'colourOK', 0, 'errSum', 0, 'bias', [0 0], ...
                'bandTP', [0 0 0], 'bandN', [0 0 0]);
            obj.Off = obj.SensorOffset;
            obj.Calibrated = ~obj.AutoCalibrate;
        end

        function [stats, truth, fp] = stepImpl(obj, ptCloud, cones, pose)
            xyz = reshape(ptCloud, [], 3);
            xyz = xyz(all(isfinite(xyz), 2) & any(xyz ~= 0, 2), :);
            if ~obj.Calibrated
                obj.calibrate(pose, xyz);
            end

            c = cos(pose(3));  s = sin(pose(3));
            sx = pose(1) + c * obj.Off(1) - s * obj.Off(2);
            sy = pose(2) + s * obj.Off(1) + c * obj.Off(2);

            % true cones and distractors in the sensor frame
            [tx, ty] = obj.toSensor(obj.TruthXY, sx, sy, c, s);
            dist = hypot(tx, ty);
            inR  = find(dist <= obj.ScoreRange);
            [~, o] = sort(dist(inR));  inR = inR(o);

            % which of them did the LiDAR actually hit?
            hits = zeros(numel(inR), 1);
            if ~isempty(xyz)
                obj3 = xyz(xyz(:, 3) > PerceptionScore.groundZ(xyz(:, 3)) + obj.GroundTol, :);
                for k = 1:numel(inR)
                    hits(k) = nnz(hypot(obj3(:, 1) - tx(inR(k)), obj3(:, 2) - ty(inR(k))) < obj.HitRadius);
                end
            end
            visible = hits >= obj.MinHits;
            visIdx  = inR(visible);

            % detections inside the scoring range
            det = cones(all(isfinite(cones(:, 1:3)), 2), :);
            det = det(hypot(det(:, 1), det(:, 2)) <= obj.ScoreRange, :);

            % one-to-one matching
            pairs = PerceptionScore.match(det(:, 1:2), [tx(visIdx) ty(visIdx)], obj.MatchGate);
            nTP = size(pairs, 1);
            nFP = size(det, 1) - nTP;
            nFN = numel(visIdx) - nTP;

            % running totals
            t = obj.T;
            t.scans = t.scans + 1;
            t.visible = t.visible + numel(visIdx);
            t.tp = t.tp + nTP;  t.fp = t.fp + nFP;  t.fn = t.fn + nFN;
            for p = 1:nTP
                dv = det(pairs(p, 1), 1:2) - [tx(visIdx(pairs(p, 2))) ty(visIdx(pairs(p, 2)))];
                t.errSum = t.errSum + hypot(dv(1), dv(2));
                t.bias   = t.bias + dv;
                t.colourOK = t.colourOK + (round(det(pairs(p, 1), 4)) == obj.TruthCls(visIdx(pairs(p, 2))));
            end
            edges = [0 10 20 30];
            for b = 1:3
                inBand = dist(visIdx) >= edges(b) & dist(visIdx) < edges(b + 1) + (b == 3) * 1e-9;
                t.bandN(b)  = t.bandN(b)  + nnz(inBand);
                t.bandTP(b) = t.bandTP(b) + nnz(inBand(pairs(:, 2)));
            end
            unmatched = setdiff(1:size(det, 1), pairs(:, 1));
            if ~isempty(unmatched) && ~isempty(obj.DistXY)
                [dxs, dys] = obj.toSensor(obj.DistXY, sx, sy, c, s);
                for u = unmatched
                    t.distFP = t.distFP + (min(hypot(dxs - det(u, 1), dys - det(u, 2))) < 0.4);
                end
            end
            obj.T = t;

            % outputs
            stats = [numel(inR), numel(visIdx), nTP, nFP, nFN, obj.Off(1)];

            truth = nan(obj.MaxTruth, 4);
            m = min(numel(inR), obj.MaxTruth);
            state = 2 * ones(numel(inR), 1);
            state(visible) = 0;
            state(find(visible & ismember(inR, visIdx(pairs(:, 2))))) = 1; %#ok<FNDSB>
            truth(1:m, :) = [tx(inR(1:m)) ty(inR(1:m)) obj.TruthCls(inR(1:m)) state(1:m)];

            fp = nan(obj.MaxCones, 2);
            f = det(unmatched, 1:2);
            fp(1:min(size(f, 1), obj.MaxCones), :) = f(1:min(size(f, 1), obj.MaxCones), :);
        end

        function releaseImpl(obj)
            t = obj.T;
            if isempty(t) || t.scans == 0, return; end
            prec = t.tp / max(t.tp + t.fp, 1);
            rec  = t.tp / max(t.tp + t.fn, 1);
            rep = struct('track', obj.Track, 'scans', t.scans, 'visibleCones', t.visible, ...
                'TP', t.tp, 'FP', t.fp, 'FN', t.fn, 'precision', prec, 'recall', rec, ...
                'F1', 2 * prec * rec / max(prec + rec, eps), ...
                'lidarOffsetX_m', obj.Off(1), ...
                'meanPosErr_m', t.errSum / max(t.tp, 1), 'biasXY_m', t.bias / max(t.tp, 1), ...
                'colourAccuracy', t.colourOK / max(t.tp, 1), ...
                'recallByDistance', t.bandTP ./ max(t.bandN, 1), 'falsePosOnDistractors', t.distFP);
            assignin('base', 'perceptionScoreReport', rep);
            fprintf('\n===== Perception score: %s (%d scans) =====\n', rep.track, rep.scans);
            fprintf('  visible cones (summed over scans) %d\n', rep.visibleCones);
            fprintf('  TP %d   FP %d   FN %d\n', rep.TP, rep.FP, rep.FN);
            fprintf('  precision %.1f %%   recall %.1f %%   F1 %.3f\n', 100 * rep.precision, 100 * rep.recall, rep.F1);
            fprintf('  position error %.3f m (mean)   bias x %+.3f m, y %+.3f m\n', ...
                rep.meanPosErr_m, rep.biasXY_m(1), rep.biasXY_m(2));
            fprintf('  colour accuracy %.1f %%\n', 100 * rep.colourAccuracy);
            fprintf('  recall by distance  0-10 m %.0f %%   10-20 m %.0f %%   20-30 m %.0f %%\n', 100 * rep.recallByDistance);
            fprintf('  false positives on distractors: %d of %d\n', rep.falsePosOnDistractors, rep.FP);
            if abs(rep.biasXY_m(1)) > 0.3 || abs(rep.biasXY_m(2)) > 0.3
                fprintf('  NOTE: a bias this large usually means the LiDAR offset is wrong, not the detector.\n');
            end
            fprintf('  (stored in the workspace variable perceptionScoreReport)\n\n');
        end

        %% ------------------------------------------------- ports and sizes
        function num = getNumInputsImpl(~),  num = 3; end
        function num = getNumOutputsImpl(~), num = 3; end
        function [n1, n2, n3] = getInputNamesImpl(~)
            n1 = 'ptCloud'; n2 = 'cones'; n3 = 'pose';
        end
        function [n1, n2, n3] = getOutputNamesImpl(~)
            n1 = 'stats'; n2 = 'truth'; n3 = 'fp';
        end
        function [s1, s2, s3] = getOutputSizeImpl(obj)
            s1 = [1 6]; s2 = [obj.MaxTruth 4]; s3 = [obj.MaxCones 2];
        end
        function [t1, t2, t3] = getOutputDataTypeImpl(~)
            t1 = 'double'; t2 = 'double'; t3 = 'double';
        end
        function [c1, c2, c3] = isOutputComplexImpl(~)
            c1 = false; c2 = false; c3 = false;
        end
        function [f1, f2, f3] = isOutputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true;
        end
        function flag = supportsMultipleInstanceImpl(~)
            flag = false;
        end
    end

    methods (Access = private)
        function calibrate(obj, pose, xyz)
            % Find the LiDAR x offset at which the most true cones line up with returns
            if size(xyz, 1) < 50, return; end
            o = xyz(xyz(:, 3) > PerceptionScore.groundZ(xyz(:, 3)) + obj.GroundTol, 1:2);
            if size(o, 1) < 4 * obj.MinHits, return; end
            c = cos(pose(3));  s = sin(pose(3));
            cand  = -2:0.05:5;
            score = -inf(size(cand));
            nIn   = zeros(size(cand));
            for i = 1:numel(cand)
                sx = pose(1) + c * cand(i) - s * obj.SensorOffset(2);
                sy = pose(2) + s * cand(i) + c * obj.SensorOffset(2);
                [tx, ty] = PerceptionScore.toSensor(obj.TruthXY, sx, sy, c, s);
                near = hypot(tx, ty) <= 20;
                if nnz(near) < 4, continue; end
                D   = hypot(tx(near) - o(:, 1)', ty(near) - o(:, 2)');
                hit = sum(D < obj.HitRadius, 2) >= obj.MinHits;
                score(i) = nnz(hit) - 0.01 * sum(min(D(hit, :), [], 2));   % tie-break: closer is better
                nIn(i)   = nnz(near);
            end
            [best, k] = max(score);
            if ~isfinite(best), return; end             % nothing to go on yet - try the next scan
            obj.Off = [cand(k) obj.SensorOffset(2)];
            obj.Calibrated = true;
            fprintf('PerceptionScore: LiDAR is %.2f m ahead of the pose point (found from the first scan)\n', cand(k));
            if round(best) < 0.4 * nIn(k)
                warning('PerceptionScore:noAlignment', ...
                    ['Only %d of %d true cones within 20 m line up with LiDAR returns at the best offset. ' ...
                     'The pose and the 3-D scene probably disagree (mirrored Y, yaw units, wrong track?). ' ...
                     'Scores will be meaningless until that is fixed.'], round(best), nIn(k));
            end
        end
    end

    methods (Static, Access = private)
        function [xs, ys] = toSensor(xy, sx, sy, c, s)
            d  = xy - [sx sy];
            xs = c * d(:, 1) + s * d(:, 2);
            ys = -s * d(:, 1) + c * d(:, 2);
        end

        function pairs = match(detXY, truXY, gate)
            % one-to-one matches within the gate -> [det index, truth index]
            pairs = zeros(0, 2);
            if isempty(detXY) || isempty(truXY), return; end
            D = hypot(detXY(:, 1) - truXY(:, 1)', detXY(:, 2) - truXY(:, 2)');
            C = D;  C(D > gate) = 1e6;
            M = matchpairs(C, 1e5);
            if ~isempty(M)
                M = M(D(sub2ind(size(D), M(:, 1), M(:, 2))) <= gate, :);
            end
            pairs = M;
        end

        function zg = groundZ(z)
            % Ground = the most common height (the ground is most of the scan)
            bin = 0.02;
            edges = floor(min(z) / bin) * bin : bin : ceil(max(z) / bin) * bin + bin;
            if numel(edges) < 2, zg = median(z); return; end
            n = histcounts(z, edges);
            [~, k] = max(n);
            zg = (edges(k) + edges(k + 1)) / 2;
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header('PerceptionScore', ...
                'Title', 'Perception Score', ...
                'Text', 'Compares detected cones with the true cones: TP, FP, FN, precision, recall, position error.');
        end
    end
end
