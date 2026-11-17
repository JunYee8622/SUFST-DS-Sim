classdef PerceptionPython < matlab.System
    % PERCEPTIONPYTHON  Simulink block that runs the perception team's Python file.
    %
    % Once per LiDAR scan the block calls   detect(points, intensity, ring)
    % in the chosen .py file and turns the result into outputs:
    %
    %   nCones   number of cones detected
    %   nLeft    ... of which on the left  (class 0, blue)
    %   nRight   ... of which on the right (class 1, yellow)
    %   nOther   ... anything else (orange, unknown)
    %   cones    [MaxCones x 4] rows [x y z class] in the SENSOR frame
    %            (x forward, y left, z up); unused rows are NaN
    %
    % See perception/perception.py for the exact input/return format. The
    % .py file is re-imported at every simulation start, so edit, save, press
    % Run - no need to restart MATLAB.
    %
    % Needs a Python install that MATLAB can see (pyenv) with numpy.
    % Runs in Interpreted execution mode (no code generation).

    properties (Nontunable)
        PythonFile   = 'perception/perception.py'  % path to the .py file
        FunctionName = 'detect'                    % function the block calls
        MaxCones     = 150                         % size of the cones output
    end

    properties (Access = private)
        Fcn       % python function handle
        NArgs     % how many arguments the python function takes (1..3)
    end

    methods (Access = protected)
        %% ---------------------------------------------------------- setup
        function setupImpl(obj)
            file = PerceptionPython.resolveFile(obj.PythonFile);
            [folder, modName] = fileparts(file);

            pe = pyenv;
            if pe.Status == "NotLoaded"
                fprintf('PerceptionPython: starting Python %s (%s)\n', ...
                    pe.Version, pe.Executable);
            end
            if count(py.sys.path, folder) == 0
                insert(py.sys.path, int32(0), folder);
            end
            mod = py.importlib.import_module(modName);
            mod = py.importlib.reload(mod);          % pick up edits to the file
            if ~logical(py.hasattr(mod, obj.FunctionName))
                error('PerceptionPython:noFunction', ...
                    'Function "%s" not found in %s', obj.FunctionName, file);
            end
            obj.Fcn = py.getattr(mod, obj.FunctionName);

            nParams = double(py.len(py.inspect.signature(obj.Fcn).parameters));
            obj.NArgs = min(max(nParams, 1), 3);
            fprintf('PerceptionPython: %s.%s loaded (%d argument(s))\n', ...
                modName, obj.FunctionName, obj.NArgs);
        end

        %% ----------------------------------------------------------- step
        function [nCones, nLeft, nRight, nOther, cones] = stepImpl(obj, ptCloud, refl)
            [nRows, nCols, ~] = size(ptCloud);
            xyz = reshape(ptCloud, nRows * nCols, 3);
            ok  = all(isfinite(xyz), 2) & any(xyz ~= 0, 2);   % drop "no return"

            pts = single(xyz(ok, :)');                       % 3 x n  (= n x 3 row-major)
            args = {PerceptionPython.toNumpy(pts, 'float32', [nnz(ok) 3])};
            if obj.NArgs >= 2
                inten = single(reshape(refl, nRows * nCols, 1));
                args{2} = PerceptionPython.toNumpy(inten(ok), 'float32', nnz(ok));
            end
            if obj.NArgs >= 3
                ringAll = repmat(int32(0:nRows-1)', 1, nCols);   % row index = channel
                ringAll = reshape(ringAll, nRows * nCols, 1);
                args{3} = PerceptionPython.toNumpy(ringAll(ok), 'int32', nnz(ok));
            end

            res = obj.Fcn(args{:});

            if isa(res, 'py.NoneType')                        % detect() returned None
                det = zeros(0, 4);
            else
                det = double(py.numpy.asarray(res, pyargs('dtype', 'float64')));
            end
            if isempty(det)
                det = zeros(0, 4);
            elseif isvector(det)
                det = reshape(det, 1, []);
            end
            if size(det, 2) == 3                              % no class given: use the side
                det(:, 4) = double(det(:, 2) < 0);            % y > 0 -> 0 (left), y < 0 -> 1 (right)
            elseif size(det, 2) < 3
                error('PerceptionPython:badOutput', ...
                    'detect() must return rows of [x y z] or [x y z class]');
            end
            det = det(:, 1:4);

            n = size(det, 1);
            nCones = double(n);
            nLeft  = double(nnz(det(:, 4) == 0));
            nRight = double(nnz(det(:, 4) == 1));
            nOther = nCones - nLeft - nRight;

            cones = nan(obj.MaxCones, 4);
            m = min(n, obj.MaxCones);
            cones(1:m, :) = det(1:m, :);
        end

        function releaseImpl(obj)
            obj.Fcn = [];
        end

        %% ------------------------------------------------- port / sizes
        function num = getNumInputsImpl(~),  num = 2; end
        function num = getNumOutputsImpl(~), num = 5; end

        function [n1, n2, n3, n4, n5] = getOutputNamesImpl(~)
            n1 = 'nCones'; n2 = 'nLeft'; n3 = 'nRight'; n4 = 'nOther'; n5 = 'cones';
        end
        function [n1, n2] = getInputNamesImpl(~)
            n1 = 'ptCloud'; n2 = 'reflectivity';
        end

        function [s1, s2, s3, s4, s5] = getOutputSizeImpl(obj)
            s1 = [1 1]; s2 = [1 1]; s3 = [1 1]; s4 = [1 1];
            s5 = [obj.MaxCones 4];
        end
        function [t1, t2, t3, t4, t5] = getOutputDataTypeImpl(~)
            t1 = 'double'; t2 = 'double'; t3 = 'double'; t4 = 'double'; t5 = 'double';
        end
        function [c1, c2, c3, c4, c5] = isOutputComplexImpl(~)
            c1 = false; c2 = false; c3 = false; c4 = false; c5 = false;
        end
        function [f1, f2, f3, f4, f5] = isOutputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true; f4 = true; f5 = true;
        end

        function flag = supportsMultipleInstanceImpl(~)
            flag = false;                   % one Python session, one detector
        end
    end

    methods (Static, Access = private)
        function file = resolveFile(name)
            % Absolute path, else relative to the current folder, else to the project root
            name = char(name);
            cands = {name, fullfile(pwd, name)};
            try, cands{end+1} = fullfile(currentProject().RootFolder, name); catch, end
            file = '';
            for k = 1:numel(cands)
                if isfile(cands{k})
                    [~, info] = fileattrib(cands{k});     % full path
                    file = info.Name;
                    return;
                end
            end
            error('PerceptionPython:noFile', 'Python file not found: %s', name);
        end

        function arr = toNumpy(x, dtype, shape)
            % MATLAB array -> numpy array (shape given in row-major order)
            buf = py.bytes(typecast(x(:), 'uint8'));
            arr = py.numpy.frombuffer(buf, pyargs('dtype', dtype));
            arr = arr.reshape(py.tuple(num2cell(int32(shape))));
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header('PerceptionPython', ...
                'Title', 'Perception (Python)', ...
                'Text', ['Runs detect(points, intensity, ring) from your .py file on every ' ...
                         'LiDAR scan. Path is relative to the project folder, or absolute.']);
        end
    end
end
