classdef PointCloudViewer < matlab.System
    % POINTCLOUDVIEWER  Live 3-D view of the LiDAR point cloud during simulation.
    %
    % Inputs
    %   ptCloud  [M x N x 3]  point cloud from the Simulation 3D Lidar block
    %                         (sensor frame: x forward, y left, z up)
    %   cones    [K x 4]      detections from the Perception block, rows
    %                         [x y z class]; NaN rows are ignored
    %
    % The figure shows ground returns in grey, returns standing above the
    % ground (cones and other objects) in red, and each detected cone as a
    % ring: blue = left, yellow = right, orange = other. The sensor is at
    % the origin. Drag with the mouse to rotate.
    %
    % Plain MATLAB graphics only - no toolbox needed.

    properties (Nontunable)
        XLim        = [-5 40]    % view limits in the sensor frame [m]
        YLim        = [-20 20]
        ZLim        = [-1.5 1.5]
        GroundTol   = 0.04       % points higher than this above the ground are "objects" [m]
        ThinGround  = 4          % draw every n-th ground point (speed)
    end

    properties (Access = private)
        Fig
        hGround
        hObj
        hLeft
        hRight
        hOther
        hTitle
        Count = 0
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Fig = figure('Name', 'LiDAR point cloud (sensor frame)', ...
                'NumberTitle', 'off');
            try theme(obj.Fig, 'light'); catch, set(obj.Fig, 'Color', 'w'); end
            ax = axes('Parent', obj.Fig);
            hold(ax, 'on'); grid(ax, 'on');
            obj.hGround = plot3(ax, nan, nan, nan, '.', 'Color', [0.65 0.65 0.65], 'MarkerSize', 3);
            obj.hObj    = plot3(ax, nan, nan, nan, '.', 'Color', [0.85 0 0],       'MarkerSize', 8);
            obj.hLeft   = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 12, 'LineWidth', 2, 'Color', [0 0.45 0.9]);
            obj.hRight  = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 12, 'LineWidth', 2, 'Color', [0.9 0.7 0]);
            obj.hOther  = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 12, 'LineWidth', 2, 'Color', [1 0.45 0]);
            plot3(ax, 0, 0, 0, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'g');   % the sensor
            xlim(ax, obj.XLim); ylim(ax, obj.YLim); zlim(ax, obj.ZLim);
            daspect(ax, [1 1 1]);
            xlabel(ax, 'x forward [m]'); ylabel(ax, 'y left [m]'); zlabel(ax, 'z up [m]');
            view(ax, -75, 28);
            obj.hTitle = title(ax, 'waiting for first scan', 'Interpreter', 'none');
            legend(ax, [obj.hGround obj.hObj obj.hLeft obj.hRight obj.hOther], ...
                {'ground', 'objects (cones)', 'detected: left', 'detected: right', 'detected: other'}, ...
                'Location', 'northeastoutside');
            obj.Count = 0;
        end

        function stepImpl(obj, ptCloud, cones)
            if isempty(obj.Fig) || ~isvalid(obj.Fig)      % window was closed
                return;
            end
            obj.Count = obj.Count + 1;

            xyz = reshape(ptCloud, [], 3);
            xyz = xyz(all(isfinite(xyz), 2) & any(xyz ~= 0, 2), :);
            nObj = 0;
            if ~isempty(xyz)
                zg  = PointCloudViewer.groundZ(xyz(:, 3));
                isObj = xyz(:, 3) > zg + obj.GroundTol;
                g = xyz(~isObj, :);
                g = g(1:obj.ThinGround:end, :);
                o = xyz(isObj, :);
                nObj = size(o, 1);
                set(obj.hGround, 'XData', g(:,1), 'YData', g(:,2), 'ZData', g(:,3));
                set(obj.hObj,    'XData', o(:,1), 'YData', o(:,2), 'ZData', o(:,3));
            end

            ok = all(isfinite(cones(:, 1:3)), 2);
            c  = cones(ok, :);
            PointCloudViewer.setPts(obj.hLeft,  c(c(:,4) == 0, :));
            PointCloudViewer.setPts(obj.hRight, c(c(:,4) == 1, :));
            PointCloudViewer.setPts(obj.hOther, c(c(:,4) ~= 0 & c(:,4) ~= 1, :));

            set(obj.hTitle, 'String', sprintf( ...
                'scan %d   |   %d points, %d above ground   |   %d cones detected', ...
                obj.Count, size(xyz, 1), nObj, size(c, 1)));
            drawnow limitrate;
        end

        function releaseImpl(~)
            % leave the figure open so the last scan can be inspected
        end

        function num = getNumInputsImpl(~),  num = 2; end
        function num = getNumOutputsImpl(~), num = 0; end
        function [n1, n2] = getInputNamesImpl(~)
            n1 = 'ptCloud'; n2 = 'cones';
        end
        function flag = supportsMultipleInstanceImpl(~)
            flag = false;
        end
    end

    methods (Static, Access = private)
        function setPts(h, c)
            if isempty(c)
                set(h, 'XData', nan, 'YData', nan, 'ZData', nan);
            else
                set(h, 'XData', c(:,1), 'YData', c(:,2), 'ZData', c(:,3));
            end
        end

        function zg = groundZ(z)
            % Ground = the most common height (the ground is most of the scan)
            bin = 0.02;
            edges = floor(min(z)/bin)*bin : bin : ceil(max(z)/bin)*bin + bin;
            if numel(edges) < 2, zg = median(z); return; end
            n = histcounts(z, edges);
            [~, k] = max(n);
            zg = (edges(k) + edges(k+1)) / 2;
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header('PointCloudViewer', ...
                'Title', 'Point Cloud Viewer', ...
                'Text', 'Live 3-D view of the LiDAR point cloud and the detected cones.');
        end
    end
end
