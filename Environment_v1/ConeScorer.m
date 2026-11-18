classdef ConeScorer < matlab.System
    % CONESCORER  Scores the perception output against the true cones, scan by scan.
    %
    % Inputs
    %   cones    [K x 4]    detections from the Perception block (sensor frame, NaN rows unused)
    %   ptRef    [M x N x 3] the instantaneous point cloud (from LiDAR Post-Processing)
    %   pose     [x y yaw]  car pose at the scan start, world frame
    % Outputs (this scan)
    %   nTruth   cones that should have been found (in range AND hit by the LiDAR)
    %   nTP      found           nFP  false detections        nFN  missed
    %
    % Rules (settings in sim_params.m, P.score):
    %  - A true cone counts only if it is within MaxRange of the sensor and was
    %    hit by at least MinHits LiDAR points (so cones hidden behind the car or
    %    other cones are not held against perception).
    %  - Detections and true cones are matched one-to-one (optimal assignment)
    %    within Gate metres. A match to a counted cone is a TP. A detection with no
    %    match within range is a FP - unless it sits on a distractor object
    %    (cone stack, timekeeping), which is reported separately.
    %  - Left/right accuracy compares the detected class with the cone colour
    %    (blue = left, yellow = right, orange = other).
    %  - Positions are compared in the sensor frame at the scan start.
    %
    % When the simulation ends the block builds the report (variable scoreReport
    % in the base workspace), prints it, shows the scoreboard and saves it to
    % results/.

    properties (Nontunable)
        MaxRange     = 30         % [m]  scoring range
        Gate         = 0.5        % [m]  match distance
        MinHits      = 3          % LiDAR points on a cone for it to count as visible
        SensorOffset = [1.2 0]    % [x y] sensor in the car frame, from the pose reference [m]
        Ts           = 0.1        % [s]  scan period
        PoseLag      = 1          % scans: Unreal delivers each scan this many scan periods after the pose (measured)
    end

    properties (Access = private)
        ConeW        % [n x 2] true cone positions, world
        ConeClass    % [n x 1] 0 blue/left, 1 yellow/right, 2 orange
        DistW        % [m x 2] distractor objects, world
        Edges        % range bin edges
        nScans = 0
        Hist       % recent poses, oldest first
        HasHist = false
        Log          % per-scan log  [t nTruth nTP nFP nFN]
        Err = zeros(0, 1)          % position errors of all TPs
        ErrRange = zeros(0, 1)     % their range
        Confusion = zeros(3, 4)    % truth class (rows) x detected class 0..3 (cols), TPs only
        BinTruth     % visible cones per range bin
        BinTP
        nDistHits = 0
        nIgnored = 0               % detections matched to real cones that were not visible
    end

    methods (Access = protected)
        function setupImpl(obj)
            cones    = evalin('base', 'cones');
            coneType = string(evalin('base', 'coneType'));
            objects  = evalin('base', 'objects');
            objType  = string(evalin('base', 'objType'));

            obj.ConeW = cones(:, 1:2);
            cls = 2 * ones(numel(coneType), 1);
            cls(coneType == "blue")   = 0;
            cls(coneType == "yellow") = 1;
            obj.ConeClass = cls;
            isCone = ismember(objType, ["blue", "yellow", "orange_small", "orange_big"]);
            obj.DistW = objects(~isCone, 1:2);

            obj.Edges = 0 : 5 : ceil(obj.MaxRange / 5) * 5;
            nb = numel(obj.Edges) - 1;
            obj.BinTruth = zeros(1, nb);
            obj.BinTP    = zeros(1, nb);
            obj.Log = zeros(0, 5);
            obj.Err = zeros(0, 1);  obj.ErrRange = zeros(0, 1);
            obj.Confusion = zeros(3, 4);
            obj.nScans = 0;  obj.nDistHits = 0;  obj.nIgnored = 0;
            obj.Hist = zeros(3, obj.PoseLag + 1);  obj.HasHist = false;
        end

        function [nTruth, nTP, nFP, nFN] = stepImpl(obj, dets, ptRef, pose)
            pose = double(pose(:));
            % the scan in hand was taken at the pose PoseLag scans ago
            if ~obj.HasHist
                obj.Hist = repmat(pose, 1, obj.PoseLag + 1);
                obj.HasHist = true;
            end
            obj.Hist = [obj.Hist(:, 2:end), pose];
            pose = obj.Hist(:, 1);
            obj.nScans = obj.nScans + 1;
            t = (obj.nScans - 1) * obj.Ts;

            % world -> sensor frame at the scan start
            cy = cos(pose(3)); sy = sin(pose(3));
            toSensor = @(W) [ cy*(W(:,1)-pose(1)) + sy*(W(:,2)-pose(2)) - obj.SensorOffset(1), ...
                             -sy*(W(:,1)-pose(1)) + cy*(W(:,2)-pose(2)) - obj.SensorOffset(2)];
            T = toSensor(obj.ConeW);
            D = toSensor(obj.DistW);
            inR  = hypot(T(:,1), T(:,2)) <= obj.MaxRange;

            %% which true cones were actually hit by the LiDAR?
            xyz = reshape(double(ptRef), [], 3);
            xyz = xyz(all(isfinite(xyz), 2), :);
            visible = false(size(T, 1), 1);
            if size(xyz, 1) > 10
                zg  = ConeScorer.groundZ(xyz(:, 3));
                obj3 = xyz(xyz(:, 3) > zg + 0.03, :);
                idx = find(inR);
                for q = 1:numel(idx)
                    k = idx(q);
                    n = nnz((obj3(:,1) - T(k,1)).^2 + (obj3(:,2) - T(k,2)).^2 < 0.3^2);
                    visible(k) = n >= obj.MinHits;
                end
            end

            %% detections in range
            dets = double(dets);
            dets = dets(all(isfinite(dets(:, 1:2)), 2), :);
            dets = dets(hypot(dets(:,1), dets(:,2)) <= obj.MaxRange, :);

            %% optimal one-to-one match against every real cone in range
            tIdx = find(inR);
            matchedDet = false(size(dets, 1), 1);
            matchedTru = false(size(T, 1), 1);
            if ~isempty(dets) && ~isempty(tIdx)
                C = hypot(dets(:,1) - T(tIdx,1)', dets(:,2) - T(tIdx,2)');   % dets x truths
                C(C > obj.Gate) = 4 * obj.Gate;                                % never matched
                pairs = matchpairs(C, obj.Gate / 2 + 1e-9);                    % cost < Gate -> matched
                for p = 1:size(pairs, 1)
                    di = pairs(p, 1); ti = tIdx(pairs(p, 2));
                    if C(pairs(p,1), pairs(p,2)) > obj.Gate, continue; end
                    matchedDet(di) = true;  matchedTru(ti) = true;
                    if visible(ti)
                        obj.Err(end+1, 1)      = C(pairs(p,1), pairs(p,2));            %#ok<AGROW>
                        obj.ErrRange(end+1, 1) = hypot(T(ti,1), T(ti,2));              %#ok<AGROW>
                        dc = min(max(round(dets(di, 4)), 0), 3);
                        obj.Confusion(obj.ConeClass(ti) + 1, dc + 1) = ...
                            obj.Confusion(obj.ConeClass(ti) + 1, dc + 1) + 1;
                        b = discretize(hypot(T(ti,1), T(ti,2)), obj.Edges);
                        if ~isnan(b), obj.BinTP(b) = obj.BinTP(b) + 1; end
                    else
                        obj.nIgnored = obj.nIgnored + 1;
                    end
                end
            end

            nTP    = nnz(matchedTru & visible);
            nTruth = nnz(visible);
            nFN    = nTruth - nTP;

            % unmatched detections: distractor hit, or false positive
            nFP = 0;
            for d = find(~matchedDet)'
                onDist = ~isempty(D) && any(hypot(D(:,1) - dets(d,1), D(:,2) - dets(d,2)) <= obj.Gate);
                if onDist
                    obj.nDistHits = obj.nDistHits + 1;
                else
                    nFP = nFP + 1;
                end
            end

            for k = find(visible)'
                b = discretize(hypot(T(k,1), T(k,2)), obj.Edges);
                if ~isnan(b), obj.BinTruth(b) = obj.BinTruth(b) + 1; end
            end

            obj.Log(end+1, :) = [t nTruth nTP nFP nFN];                        %#ok<AGROW>
            nTruth = double(nTruth); nTP = double(nTP); nFP = double(nFP); nFN = double(nFN);
        end

        function releaseImpl(obj)
            if obj.nScans < 2, return; end
            L = obj.Log;
            r = struct();
            r.track      = evalin('base', 'P.trackName');
            r.speed      = evalin('base', 'testDrive.speed');
            r.distortion = evalin('base', 'lidarCfg.motionDistortion');
            r.duration   = L(end, 1) + obj.Ts;
            r.nScans     = size(L, 1);
            r.maxRange   = obj.MaxRange;
            r.gate       = obj.Gate;
            r.minHits    = obj.MinHits;
            r.TP = sum(L(:,3));  r.FP = sum(L(:,4));  r.FN = sum(L(:,5));
            r.truth = sum(L(:,2));
            r.precision = r.TP / max(r.TP + r.FP, 1);
            r.recall    = r.TP / max(r.truth, 1);
            r.f1        = 2 * r.precision * r.recall / max(r.precision + r.recall, eps);
            r.meanErr   = mean(obj.Err);
            r.p95Err    = ConeScorer.prc(obj.Err, 95);
            r.distractorHits = obj.nDistHits;
            r.ignoredMatches = obj.nIgnored;
            r.confusion = obj.Confusion;
            nL = sum(obj.Confusion(1, :)); nR = sum(obj.Confusion(2, :));
            r.sideAccuracy = (obj.Confusion(1,1) + obj.Confusion(2,2)) / max(nL + nR, 1);
            r.edges      = obj.Edges;
            r.binTruth   = obj.BinTruth;
            r.binTP      = obj.BinTP;
            r.binRecall  = obj.BinTP ./ max(obj.BinTruth, 1);
            r.log        = L;
            r.logHeader  = {'t', 'nTruth', 'nTP', 'nFP', 'nFN'};
            r.created    = datetime('now');
            assignin('base', 'scoreReport', r);
            try
                showScoreboard(r);
            catch err
                fprintf(2, 'ConeScorer: could not show the scoreboard: %s\n', err.message);
            end
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end

        function num = getNumInputsImpl(~),  num = 3; end
        function num = getNumOutputsImpl(~), num = 4; end
        function [n1, n2, n3] = getInputNamesImpl(~)
            n1 = 'cones'; n2 = 'ptRef'; n3 = 'pose';
        end
        function [n1, n2, n3, n4] = getOutputNamesImpl(~)
            n1 = 'nTruth'; n2 = 'nTP'; n3 = 'nFP'; n4 = 'nFN';
        end
        function [s1, s2, s3, s4] = getOutputSizeImpl(~)
            s1 = [1 1]; s2 = [1 1]; s3 = [1 1]; s4 = [1 1];
        end
        function [t1, t2, t3, t4] = getOutputDataTypeImpl(~)
            t1 = 'double'; t2 = 'double'; t3 = 'double'; t4 = 'double';
        end
        function [c1, c2, c3, c4] = isOutputComplexImpl(~)
            c1 = false; c2 = false; c3 = false; c4 = false;
        end
        function [f1, f2, f3, f4] = isOutputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true; f4 = true;
        end
        function flag = supportsMultipleInstanceImpl(~)
            flag = false;
        end
    end

    methods (Static, Access = private)
        function zg = groundZ(z)
            bin = 0.02;
            edges = floor(min(z)/bin)*bin : bin : ceil(max(z)/bin)*bin + bin;
            if numel(edges) < 2, zg = median(z); return; end
            [~, k] = max(histcounts(z, edges));
            zg = (edges(k) + edges(k+1)) / 2;
        end

        function v = prc(x, p)
            if isempty(x), v = NaN; return; end
            x = sort(x(:));
            v = x(max(1, ceil(p / 100 * numel(x))));
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header('ConeScorer', ...
                'Title', 'Cone Scorer', ...
                'Text', ['Scores detections against the true cones every scan and shows ' ...
                         'the scoreboard when the simulation ends.']);
        end
    end
end
