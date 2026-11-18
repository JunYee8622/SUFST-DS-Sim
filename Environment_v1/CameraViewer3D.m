classdef CameraViewer3D < matlab.System
    % CAMERAVIEWER3D  The LiDAR point cloud seen from the 3-D camera, with the
    % detected cones drawn as coloured 3-D cone models and a live score.
    %
    % The view is a chase camera behind and above the car (same idea as the
    % Unreal window), so the cloud lines up with what you see in the 3-D scene.
    %
    % Inputs (from PerceptionPython and PerceptionScore)
    %   ptCloud  [M x N x 3]  LiDAR scan, sensor frame (x fwd, y left, z up)
    %   cones    [K x 4]      detections [x y z class]; class 0 blue, 1 yellow, 2 orange
    %   truth    [T x 4]      true cones [x y class state] (state 1 found, 0 missed, 2 unseen)
    %   fp       [K x 2]      detections that match no cone
    %   stats    [1 x 6]      [in range, visible, TP, FP, FN, LiDAR x offset from the car point]
    %
    % What you see
    %   grey dots            ground returns
    %   coloured dots        returns that belong to a detected cone (its colour)
    %   red dots             returns standing above the ground that nothing claimed
    %   cone models          detected cones: blue / yellow / orange (white = no colour)
    %   green ring           true cone that was found
    %   red ring + pin       true cone the LiDAR saw but the detector MISSED
    %   magenta pin          detection where there is no cone (false positive)
    %   dotted grey ring     true cone with too few LiDAR points to be fair
    %
    % Plain MATLAB graphics only - no toolbox needed.

    properties (Nontunable)
        CameraPos    = [-6 0 2]       % camera in the car frame [x fwd, y left, z above ground] m
                                      % (the Scene Configuration block's free camera default)
        CameraPitch  = 15             % looks down by this many degrees
        HorizontalFOV = 60            % degrees (the Unreal camera uses 60)
        GroundTol    = 0.04           % points higher than this above the ground are objects [m]
        ThinGround   = 4              % draw every n-th ground point (speed)
        ClaimRadius  = 0.35           % return within this of a detection belongs to it [m]
    end

    properties (Access = private)
        Fig
        Ax
        hGround
        hObjCls = gobjects(1, 4)   % blue, yellow, orange, unknown-colour returns
        hObjFree        % unclaimed returns
        hCones          % patch with all cone models
        hRingOK
        hRingMiss
        hRingUnseen
        hFalse
        hFalseTop
        hGrid
        hRings
        hHud
        ConeV           % template vertices
        ConeF           % template faces
        ConeCode        % per face: 0 class colour, 1 white band, 2 base plate
        LastGround = -0.4
        Count = 0
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Fig = figure('Name', 'Camera view - 3D point cloud', 'NumberTitle', 'off', ...
                'Color', [0.09 0.10 0.12], 'Position', [80 80 960 540]);
            ax = axes('Parent', obj.Fig, 'Position', [0 0 1 1], 'Visible', 'off');
            obj.Ax = ax;
            hold(ax, 'on');

            % ground grid and distance rings (z is set every scan)
            gx = []; gy = [];
            for y = -30:5:30, gx = [gx 0 60 nan]; gy = [gy y y nan]; end %#ok<AGROW>
            for x = 0:5:60,   gx = [gx x x nan];  gy = [gy -30 30 nan]; end %#ok<AGROW>
            obj.hGrid = plot3(ax, gx, gy, zeros(size(gx)), '-', 'Color', [0.20 0.22 0.26], 'LineWidth', 0.5);
            a = linspace(-pi, pi, 90); rx = []; ry = [];
            for r = [10 20 30], rx = [rx r * cos(a) nan]; ry = [ry r * sin(a) nan]; end %#ok<AGROW>
            obj.hRings = plot3(ax, rx, ry, zeros(size(rx)), '-', 'Color', [0.35 0.38 0.45], 'LineWidth', 1);

            obj.hGround = plot3(ax, nan, nan, nan, '.', 'Color', [0.55 0.58 0.62], 'MarkerSize', 3);
            cols = CameraViewer3D.classColours();
            for k = 1:4
                obj.hObjCls(k) = plot3(ax, nan, nan, nan, '.', 'Color', cols(k, :), 'MarkerSize', 11);
            end
            obj.hObjFree = plot3(ax, nan, nan, nan, '.', 'Color', [1 0.15 0.15], 'MarkerSize', 9);

            obj.hRingUnseen = plot3(ax, nan, nan, nan, ':', 'Color', [0.5 0.5 0.55], 'LineWidth', 1);
            obj.hRingOK     = plot3(ax, nan, nan, nan, '-', 'Color', [0.25 0.95 0.35], 'LineWidth', 1.5);
            obj.hRingMiss   = plot3(ax, nan, nan, nan, '-', 'Color', [1 0.15 0.15], 'LineWidth', 3);
            obj.hFalse      = plot3(ax, nan, nan, nan, '-', 'Color', [1 0.2 0.85], 'LineWidth', 3);
            obj.hFalseTop   = plot3(ax, nan, nan, nan, 'd', 'Color', [1 0.2 0.85], ...
                'MarkerFaceColor', [1 0.2 0.85], 'MarkerSize', 9);

            [obj.ConeV, obj.ConeF, obj.ConeCode] = CameraViewer3D.coneTemplate();
            obj.hCones = patch(ax, 'Faces', zeros(0, 4), 'Vertices', zeros(0, 3), ...
                'FaceVertexCData', zeros(0, 3), 'FaceColor', 'flat', 'EdgeColor', 'none', ...
                'FaceLighting', 'none');

            plot3(ax, 0, 0, 0, 'p', 'MarkerSize', 13, 'MarkerFaceColor', [0.2 0.9 0.3], 'MarkerEdgeColor', 'w');
            for r = [10 20 30]
                text(ax, r, 0, 0, sprintf(' %d m', r), 'Color', [0.55 0.58 0.65], 'FontSize', 8);
            end

            % camera
            ax.XLim = [-15 70]; ax.YLim = [-40 40]; ax.ZLim = [-3 5];
            ax.DataAspectRatio = [1 1 1];
            ax.Projection = 'perspective';
            ax.CameraUpVector = [0 0 1];
            obj.placeCamera(1.3);                       % refined every scan once the LiDAR position is known
            ax.CameraViewAngle = 2 * atand(tand(obj.HorizontalFOV / 2) * 9 / 16);   % vertical FOV, 16:9

            obj.hHud = annotation(obj.Fig, 'textbox', [0.015 0.77 0.52 0.21], 'String', 'waiting for first scan', ...
                'Color', 'w', 'FontName', 'Consolas', 'FontSize', 11, 'EdgeColor', 'none', ...
                'BackgroundColor', [0 0 0], 'FaceAlpha', 0.55, 'Interpreter', 'none', 'FitBoxToText', 'off');
            annotation(obj.Fig, 'textbox', [0.015 0.015 0.97 0.05], 'EdgeColor', 'none', 'Interpreter', 'none', ...
                'Color', [0.75 0.78 0.85], 'FontSize', 9, 'VerticalAlignment', 'middle', ...
                'String', ['cone model = detected   |   green ring = found   |   red ring + pin = MISSED   |   ' ...
                           'magenta pin = false detection   |   red dots = unclaimed object returns']);
            obj.Count = 0;
        end

        function stepImpl(obj, ptCloud, cones, truth, fp, stats)
            if isempty(obj.Fig) || ~isvalid(obj.Fig)      % window was closed
                return;
            end
            obj.Count = obj.Count + 1;

            xyz = reshape(ptCloud, [], 3);
            xyz = xyz(all(isfinite(xyz), 2) & any(xyz ~= 0, 2), :);
            if ~isempty(xyz)
                obj.LastGround = CameraViewer3D.groundZ(xyz(:, 3));
            end
            zg = obj.LastGround;
            obj.placeCamera(stats(6));
            det = cones(all(isfinite(cones(:, 1:3)), 2), :);
            nDet = size(det, 1);
            cls = det(:, 4);  cls(~ismember(cls, [0 1 2])) = 3;

            % ---- LiDAR returns: ground / object, objects tinted by the cone that claims them
            isObj = xyz(:, 3) > zg + obj.GroundTol;
            g = xyz(~isObj, :);   g = g(1:obj.ThinGround:end, :);
            o = xyz(isObj, :);
            owner = 4 * ones(size(o, 1), 1);                % 4 = unclaimed
            if nDet > 0 && ~isempty(o)
                dd = (o(:, 1) - det(:, 1)').^2 + (o(:, 2) - det(:, 2)').^2;
                [mn, j] = min(dd, [], 2);
                near = mn < obj.ClaimRadius^2;
                owner(near) = cls(j(near));
            end
            CameraViewer3D.setPts(obj.hGround, g);
            for k = 0:3
                CameraViewer3D.setPts(obj.hObjCls(k + 1), o(owner == k, :));
            end
            CameraViewer3D.setPts(obj.hObjFree, o(owner == 4, :));

            % ---- ground references follow the estimated ground height
            set(obj.hGrid,  'ZData', zg * ones(size(obj.hGrid.XData)));
            set(obj.hRings, 'ZData', zg * ones(size(obj.hRings.XData)));

            % ---- detected cones as 3-D models
            if nDet > 0
                nv = size(obj.ConeV, 1);  nf = size(obj.ConeF, 1);
                V = repmat(obj.ConeV, nDet, 1) + repelem([det(:, 1) det(:, 2) zg * ones(nDet, 1)], nv, 1);
                F = repmat(obj.ConeF, nDet, 1) + repelem((0:nDet - 1)' * nv, nf, 1);
                cc = CameraViewer3D.classColours();
                C = cc(repelem(cls, nf) + 1, :);
                code = repmat(obj.ConeCode, nDet, 1);
                C(code == 1, :) = 1;
                C(code == 2, :) = repmat([0.12 0.12 0.12], nnz(code == 2), 1);
                set(obj.hCones, 'Faces', F, 'Vertices', V, 'FaceVertexCData', C);
            else
                set(obj.hCones, 'Faces', zeros(0, 4), 'Vertices', zeros(0, 3), 'FaceVertexCData', zeros(0, 3));
            end

            % ---- truth rings (found / missed / unseen) and false positives
            tr = truth(all(isfinite(truth(:, 1:2)), 2), :);
            CameraViewer3D.setLine(obj.hRingOK,     CameraViewer3D.rings(tr(tr(:, 4) == 1, 1:2), zg, 0.30, 0));
            CameraViewer3D.setLine(obj.hRingUnseen, CameraViewer3D.rings(tr(tr(:, 4) == 2, 1:2), zg, 0.30, 0));
            CameraViewer3D.setLine(obj.hRingMiss,   CameraViewer3D.rings(tr(tr(:, 4) == 0, 1:2), zg, 0.35, 0.7));
            f = fp(all(isfinite(fp), 2), :);
            CameraViewer3D.setLine(obj.hFalse, CameraViewer3D.pins(f, zg, 0.7));
            if isempty(f)
                set(obj.hFalseTop, 'XData', nan, 'YData', nan, 'ZData', nan);
            else
                set(obj.hFalseTop, 'XData', f(:, 1), 'YData', f(:, 2), 'ZData', zg + 0.7 + 0 * f(:, 1));
            end

            % ---- heads-up display
            nL = nnz(det(:, 4) == 0);  nR = nnz(det(:, 4) == 1);  nO = nDet - nL - nR;
            inRange = stats(1);  vis = stats(2);  tp = stats(3);  nfp = stats(4);  fn = stats(5);
            if vis > 0, rec = sprintf('%3.0f %%', 100 * tp / vis); else, rec = '  - '; end
            set(obj.hHud, 'String', { ...
                sprintf('scan %d', obj.Count), ...
                sprintf('DETECTED %3d   blue %d | yellow %d | other %d', nDet, nL, nR, nO), ...
                sprintf('ON TRACK %3d   within 30 m, %d hit by the LiDAR', inRange, vis), ...
                sprintf('FOUND    %3d of %d   recall %s', tp, vis, rec), ...
                sprintf('MISSED   %3d   FALSE DETECTIONS %d', fn, nfp)});
            drawnow limitrate;
        end

        function releaseImpl(~)
            % leave the figure open so the last scan can be inspected
        end

        function num = getNumInputsImpl(~),  num = 5; end
        function num = getNumOutputsImpl(~), num = 0; end
        function [n1, n2, n3, n4, n5] = getInputNamesImpl(~)
            n1 = 'ptCloud'; n2 = 'cones'; n3 = 'truth'; n4 = 'fp'; n5 = 'stats';
        end
        function flag = supportsMultipleInstanceImpl(~)
            flag = false;
        end
    end

    methods (Access = private)
        function placeCamera(obj, lidarX)
            % The view is defined in the car frame; the cloud is in the LiDAR frame,
            % which sits lidarX ahead of the car's reference point, at sensor height
            % (= -LastGround) above the ground.
            cp = [obj.CameraPos(1) - lidarX, obj.CameraPos(2), obj.CameraPos(3) + obj.LastGround];
            obj.Ax.CameraPosition = cp;
            obj.Ax.CameraTarget = cp + 10 * [cosd(obj.CameraPitch) 0 -sind(obj.CameraPitch)];
        end
    end

    methods (Static, Access = private)
        function c = classColours()
            % rows = class 0 blue, 1 yellow, 2 orange, 3 unknown (light grey)
            c = [0.10 0.35 1.00; 1.00 0.85 0.05; 1.00 0.45 0.00; 0.80 0.80 0.80];
        end

        function setPts(h, p)
            if isempty(p)
                set(h, 'XData', nan, 'YData', nan, 'ZData', nan);
            else
                set(h, 'XData', p(:, 1), 'YData', p(:, 2), 'ZData', p(:, 3));
            end
        end

        function setLine(h, p)
            if isempty(p)
                set(h, 'XData', nan, 'YData', nan, 'ZData', nan);
            else
                set(h, 'XData', p(:, 1), 'YData', p(:, 2), 'ZData', p(:, 3));
            end
        end

        function p = rings(xy, z, r, pinH)
            % circles of radius r at the ground (plus a vertical pin of height pinH if > 0)
            if isempty(xy), p = []; return; end
            n = size(xy, 1);
            a = linspace(0, 2 * pi, 25)';
            X = [xy(:, 1)' + r * cos(a); nan(1, n)];
            Y = [xy(:, 2)' + r * sin(a); nan(1, n)];
            Z = [z * ones(25, n); nan(1, n)];
            p = [X(:) Y(:) Z(:)];
            if pinH > 0
                p = [p; CameraViewer3D.pins(xy, z, pinH)];
            end
        end

        function p = pins(xy, z, h)
            if isempty(xy), p = []; return; end
            n = size(xy, 1);
            X = [xy(:, 1)'; xy(:, 1)'; nan(1, n)];
            Y = [xy(:, 2)'; xy(:, 2)'; nan(1, n)];
            Z = [z * ones(1, n); (z + h) * ones(1, n); nan(1, n)];
            p = [X(:) Y(:) Z(:)];
        end

        function [V, F, code] = coneTemplate()
            % small FS cone: 0.228 m plate, 0.325 m tall frustum, 12 sides, 3 bands
            rb = 0.105; rt = 0.02; h = 0.325; ns = 12;
            hz = [0 0.40 0.68 1] * h;
            th = linspace(0, 2 * pi, ns + 1)'; th(end) = [];
            V = zeros(0, 3);
            for i = 1:numel(hz)
                r = rb + (rt - rb) * hz(i) / h;
                V = [V; r * cos(th) r * sin(th) hz(i) * ones(ns, 1)]; %#ok<AGROW>
            end
            F = zeros(0, 4); code = zeros(0, 1);
            s = (1:ns)';  nxt = [2:ns 1]';
            for i = 1:numel(hz) - 1
                a0 = (i - 1) * ns;  b0 = i * ns;
                F = [F; a0 + s, a0 + nxt, b0 + nxt, b0 + s]; %#ok<AGROW>
                code = [code; (i == 2) * ones(ns, 1)]; %#ok<AGROW>
            end
            nb = size(V, 1);
            V = [V; -0.114 -0.114 0; 0.114 -0.114 0; 0.114 0.114 0; -0.114 0.114 0];
            F = [F; nb + 1 nb + 2 nb + 3 nb + 4];
            code = [code; 2];
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
            header = matlab.system.display.Header('CameraViewer3D', ...
                'Title', 'Camera View 3D', ...
                'Text', 'Chase-camera 3-D view of the LiDAR cloud with coloured cone models and a live score.');
        end
    end
end
