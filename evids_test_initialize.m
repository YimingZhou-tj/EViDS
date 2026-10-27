function evids_test_initialize(modelName, phase)
% 模型自动调用，无需手动执行。使用者只需修改下面的本机安装路径。
% ==================== 按自己的安装目录填写 ====================
env.PythonExe = '';     % 64位Python 3.7的python.exe完整路径
env.CarlaEgg = '';      % CARLA 0.9.11/Python 3.7客户端egg；已安装客户端可留空
env.CarlaExe = '';      % CarlaUE4.exe完整路径；AutoStartCarla=false时可留空
env.CarSimSolvers = ''; % 包含Solver_SF.slx或Solver_SF.mdl的CarSim求解器目录
env.CarlaHost = 'localhost';
env.CarlaPort = 2000;
env.AutoStartCarla = true;
env.StopTime = 5;        % 仿真时长，单位秒
% ============================================================
if nargin < 2; phase = 'run'; end
phase = validatestring(phase, {'load','run'});
root = fileparts(mfilename('fullpath'));
addpath(root);
if isempty(env.CarSimSolvers) || ...
        ~(isfile(fullfile(env.CarSimSolvers,'Solver_SF.slx')) || ...
          isfile(fullfile(env.CarSimSolvers,'Solver_SF.mdl')))
    error('EVIDS:CarSimPath', ...
        'Fill in env.CarSimSolvers at the top of evids_test_initialize.m. It must contain Solver_SF.slx or Solver_SF.mdl.');
end
addpath(env.CarSimSolvers);
if strcmp(phase, 'load')
    if isempty(env.PythonExe) || ~isfile(env.PythonExe)
        error('EVIDS:PythonPath', 'Fill in env.PythonExe at the top of evids_test_initialize.m.');
    end
    % 在System类解析之前选择正确解释器及客户端；此处不连接/启动CARLA世界。
    evids_test_initialize_python(env.PythonExe, env.CarlaEgg);
    return;
end
% If initialization fails, StopFcn must not overwrite an earlier run's folder.
evalin('base', 'clear EVIDS_TEST_RUN_FOLDER');
if isempty(env.PythonExe) || ~isfile(env.PythonExe)
    error('EVIDS:PythonPath', 'Fill in env.PythonExe at the top of evids_test_initialize.m.');
end
validateattributes(env.StopTime, {'numeric'}, {'scalar','real','finite','positive'});
validateattributes(env.CarlaPort, {'numeric'}, {'scalar','integer','>=',1,'<=',65535});
evids_test_initialize_python(env.PythonExe, env.CarlaEgg);
if env.AutoStartCarla
    if isempty(env.CarlaExe) || ~isfile(env.CarlaExe)
        error('EVIDS:CarlaExePath', 'Fill in env.CarlaExe or set env.AutoStartCarla=false.');
    end
    command = sprintf(['powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%s" ', ...
        '-CarlaExe "%s" -CarlaHost "%s" -CarlaPort %d'], ...
        fullfile(root, 'ensure_carla_for_test.ps1'), env.CarlaExe, env.CarlaHost, env.CarlaPort);
    [status, message] = system(command);
    if status ~= 0; error('EVIDS:CarlaStartup', '%s', message); end
    fprintf('%s', message);
end
% CarSim Send may change StopTime to inf; use the configured duration.
set_param(modelName, 'StopTime', num2str(env.StopTime, 17));
% Preserve the SIMFILE selected by CarSim Send to Simulink; never replace it here.
systemBlock = [modelName '/MATLAB System2'];
if ~ismember(get_param(systemBlock, 'System'), {'EVIDSClosedLoopSensors', 'EVIDSClosedLoopSensorsV2'})
    error('EVIDS:TestClass', 'The independent test class is not selected.');
end
set_param(systemBlock, 'CarlaHost', char(env.CarlaHost), 'CarlaPort', num2str(env.CarlaPort));
stamp = char(datetime('now', 'TimeZone', 'Asia/Shanghai', 'Format', 'yyyyMMdd_HHmmss_SSS'));
runFolder = fullfile(root, 'results', stamp);
mkdir(runFolder);
assignin('base', 'EVIDS_TEST_RUN_FOLDER', runFolder);
blocks = find_system(modelName, 'BlockType', 'ToWorkspace');
for k = 1:numel(blocks)
    name = get_param(blocks{k}, 'VariableName');
    if startsWith(name, 'evids_')
        evalin('base', ['clear ' name]);
    end
end
metadata = struct('Model', modelName, ...
    'ModelFile', get_param(modelName, 'FileName'), ...
    'SimFile', get_param([modelName '/CarSim S-Function'], 'SIMFILE'), ...
    'ConfiguredStopTime', get_param(modelName, 'StopTime'), ...
    'StartTimeChina', char(datetime('now', 'TimeZone', 'Asia/Shanghai')), ...
    'CarlaPort', env.CarlaPort);
save(fullfile(runFolder, 'run_configuration.mat'), 'metadata');
fprintf('EVIDS test output folder: %s\n', runFolder);
end
