function evids_test_schedule_results(modelName)
% Delay until To Workspace blocks have published their final values.
if ~evalin('base', 'exist(''EVIDS_TEST_RUN_FOLDER'', ''var'')'); return; end
runFolder = evalin('base', 'EVIDS_TEST_RUN_FOLDER');
job = timer('ExecutionMode', 'singleShot', 'StartDelay', 0.5, ...
    'TimerFcn', @(~, ~) saveResults(modelName, runFolder), ...
    'StopFcn', @(t, ~) delete(t));
start(job);
end
function saveResults(modelName, runFolder)
try
    evids_test_save_results(modelName, runFolder);
catch ME
    fid = fopen(fullfile(runFolder, 'result_export_error.txt'), 'w', 'n', 'UTF-8');
    if fid >= 0
        fprintf(fid, '%s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));
        fclose(fid);
    end
    warning('EVIDS:ResultExport', 'Result export failed: %s. See %s', ME.message, runFolder);
end
end
