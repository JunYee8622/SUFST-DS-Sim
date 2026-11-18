function showScoreboard(r)
% SHOWSCOREBOARD  Print, display and save the perception scoreboard.
%   Called automatically by the Cone Scorer block when a simulation ends.
%   Call it yourself on a saved report:  load results/xxx.mat; showScoreboard(scoreReport)
%
% Everything is written to the results/ folder next to this file:
%   score_<track>_<speed>mps_<date>.mat   the full report
%   score_<track>_<speed>mps_<date>.txt   the scoreboard as text
%   score_<track>_<speed>mps_<date>.png   the scoreboard figure

if r.distortion ~= 0, dist = 'ON'; else, dist = 'off'; end
sideTxt = 'n/a';
if ~isnan(r.sideAccuracy), sideTxt = sprintf('%.1f %%', 100 * r.sideAccuracy); end

%% ---------------------------------------------------------------- text
lines = {
    '================ PERCEPTION SCOREBOARD ================'
    sprintf('Track %s   |   speed %.1f m/s   |   %.1f s, %d scans', r.track, r.speed, r.duration, r.nScans)
    sprintf('LiDAR motion distortion: %s   |   scored within %.0f m, match gate %.2f m, >= %d hits', ...
            dist, r.maxRange, r.gate, r.minHits)
    '-------------------------------------------------------'
    sprintf('Cones to find (summed over scans) : %d', r.truth)
    sprintf('Found    (TP)                     : %d', r.TP)
    sprintf('Missed   (FN)                     : %d', r.FN)
    sprintf('False    (FP)                     : %d', r.FP)
    sprintf('Hits on distractor objects        : %d   (not counted as FP)', r.distractorHits)
    '-------------------------------------------------------'
    sprintf('Recall     : %.1f %%', 100 * r.recall)
    sprintf('Precision  : %.1f %%', 100 * r.precision)
    sprintf('F1         : %.3f', r.f1)
    sprintf('Position error  mean %.2f m   95th pct %.2f m', r.meanErr, r.p95Err)
    sprintf('Left/right accuracy (blue/yellow) : %s', sideTxt)
    '-------------------------------------------------------'
    'Recall by distance from the sensor:'};
for b = 1:numel(r.binTruth)
    lines{end+1} = sprintf('   %2d-%2d m : %5.1f %%   (%d of %d)', r.edges(b), r.edges(b+1), ...
        100 * r.binRecall(b), r.binTP(b), r.binTruth(b)); %#ok<AGROW>
end
lines{end+1} = '=======================================================';
txt = strjoin(lines, newline);
disp(' '); disp(txt); disp(' ');

%% ---------------------------------------------------------- figure
f = figure('Name', 'Perception scoreboard', 'NumberTitle', 'off', ...
           'Position', [80 80 1100 620]);
try theme(f, 'light'); catch, set(f, 'Color', 'w'); end

rows = {
    'Cones to find',  sprintf('%d', r.truth)
    'Found (TP)',     sprintf('%d', r.TP)
    'Missed (FN)',    sprintf('%d', r.FN)
    'False (FP)',     sprintf('%d', r.FP)
    'Distractor hits',sprintf('%d', r.distractorHits)
    'Recall',         sprintf('%.1f %%', 100 * r.recall)
    'Precision',      sprintf('%.1f %%', 100 * r.precision)
    'F1',             sprintf('%.3f', r.f1)
    'Mean pos. error',sprintf('%.2f m', r.meanErr)
    '95th pct error', sprintf('%.2f m', r.p95Err)
    'Left/right acc.',sideTxt};
uicontrol(f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.03 0.90 0.40 0.08], ...
    'String', sprintf('%s  |  %.1f m/s  |  distortion %s', r.track, r.speed, dist), ...
    'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'BackgroundColor', get(f, 'Color'));
uitable(f, 'Units', 'normalized', 'Position', [0.03 0.22 0.44 0.68], ...
    'Data', rows, 'ColumnName', {'Metric', 'Value'}, 'RowName', {}, ...
    'ColumnWidth', {190, 150}, 'FontSize', 12);
uicontrol(f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.03 0.02 0.44 0.17], ...
    'String', sprintf(['Scored within %.0f m. A cone counts if the LiDAR hit it with >= %d points.\n' ...
        'Match gate %.2f m. Positions in the sensor frame at the scan start.\n%d scans over %.1f s.'], ...
        r.maxRange, r.minHits, r.gate, r.nScans, r.duration), ...
    'FontSize', 10, 'HorizontalAlignment', 'left', 'BackgroundColor', get(f, 'Color'));

ax1 = axes(f, 'Position', [0.52 0.60 0.45 0.32]);
ctr = (r.edges(1:end-1) + r.edges(2:end)) / 2;
bar(ax1, ctr, 100 * r.binRecall, 0.8, 'FaceColor', [0.15 0.45 0.85]);
ylim(ax1, [0 115]); grid(ax1, 'on');
xlabel(ax1, 'distance from sensor [m]'); ylabel(ax1, 'recall [%]');
title(ax1, 'Recall by distance');
for b = 1:numel(ctr)
    text(ax1, ctr(b), 100 * r.binRecall(b) + 2, sprintf('%d/%d', r.binTP(b), r.binTruth(b)), ...
        'HorizontalAlignment', 'center', 'FontSize', 8);
end

ax2 = axes(f, 'Position', [0.52 0.10 0.45 0.38]);
L = r.log;
plot(ax2, L(:,1), L(:,2), '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5); hold(ax2, 'on');
plot(ax2, L(:,1), L(:,3), '-', 'Color', [0.1 0.65 0.2], 'LineWidth', 1.5);
plot(ax2, L(:,1), L(:,4), '-', 'Color', [0.85 0.15 0.15], 'LineWidth', 1.5);
grid(ax2, 'on'); xlabel(ax2, 'time [s]'); ylabel(ax2, 'cones per scan');
legend(ax2, {'to find', 'found (TP)', 'false (FP)'}, 'Location', 'northeast');
title(ax2, 'Every scan');
drawnow;

%% ---------------------------------------------------------------- save
folder = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(folder, 'dir'), mkdir(folder); end
stamp = char(datetime(r.created, 'Format', 'yyyyMMdd_HHmmss'));
base  = fullfile(folder, sprintf('score_%s_%gmps_%s', r.track, r.speed, stamp));
scoreReport = r; %#ok<NASGU>
save([base '.mat'], 'scoreReport');
fid = fopen([base '.txt'], 'w'); fprintf(fid, '%s\n', txt); fclose(fid);
try
    exportapp(f, [base '.png']);
catch
    try, saveas(f, [base '.png']); catch, end
end
fprintf('Scoreboard saved: %s.(mat/txt/png)\n', base);
end
