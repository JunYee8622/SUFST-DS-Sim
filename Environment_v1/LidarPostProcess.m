classdef LidarPostProcess < matlab.System
    % LIDARPOSTPROCESS  Makes the simulated point cloud look like the real Pandar40P's.
    %
    % The Simulation 3D Lidar block returns every ray that hits anything,
    % including the car's own nose a few centimetres from the sensor. The real
    % sensor ignores everything closer than its minimum range, and its ranges
    % are noisy. This block applies, to each point:
    %   - minimum / maximum range  -> points outside become NaN (no return)
    %   - Gaussian range noise     -> the point moves along its own ray
    %
    % Inputs  ptCloud [M x N x 3], reflectivity [M x N]
    % Outputs the same, cleaned. Settings come from lidarCfg (sim_params.m).

    properties (Nontunable)
        MinRange = 0.3     % [m]  datasheet minimum range
        MaxRange = 80      % [m]  simulation range limit
        NoiseStd = 0       % [m]  range noise, 1 sigma (0 = off)
    end

    methods (Access = protected)
        function [ptOut, reflOut] = stepImpl(obj, pt, refl)
            x = pt(:, :, 1);  y = pt(:, :, 2);  z = pt(:, :, 3);
            r = sqrt(x.^2 + y.^2 + z.^2);

            bad = ~isfinite(r) | r < obj.MinRange | r > obj.MaxRange;

            if obj.NoiseStd > 0
                s = 1 + single(obj.NoiseStd) * randn(size(r), 'like', r) ./ max(r, eps('single'));
                x = x .* s;  y = y .* s;  z = z .* s;
            end

            x(bad) = NaN;  y(bad) = NaN;  z(bad) = NaN;
            ptOut = cat(3, x, y, z);

            reflOut = refl;
            reflOut(bad) = 0;
        end

        function num = getNumInputsImpl(~),  num = 2; end
        function num = getNumOutputsImpl(~), num = 2; end
        function [n1, n2] = getInputNamesImpl(~)
            n1 = 'ptCloud'; n2 = 'reflectivity';
        end
        function [n1, n2] = getOutputNamesImpl(~)
            n1 = 'ptCloud'; n2 = 'reflectivity';
        end
        function [s1, s2] = getOutputSizeImpl(obj)
            s1 = propagatedInputSize(obj, 1);
            s2 = propagatedInputSize(obj, 2);
        end
        function [t1, t2] = getOutputDataTypeImpl(obj)
            t1 = propagatedInputDataType(obj, 1);
            t2 = propagatedInputDataType(obj, 2);
        end
        function [c1, c2] = isOutputComplexImpl(~)
            c1 = false; c2 = false;
        end
        function [f1, f2] = isOutputFixedSizeImpl(~)
            f1 = true; f2 = true;
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header('LidarPostProcess', ...
                'Title', 'LiDAR Post-Processing', ...
                'Text', 'Minimum / maximum range and range noise, as on the real Pandar40P.');
        end
    end
end
