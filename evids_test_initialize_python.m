function evids_test_initialize_python(pythonExecutable, carlaEgg)
% Configure only this MATLAB session; no hard-coded drive letters.
root = fileparts(mfilename('fullpath'));
env = pyenv;
if nargin > 0 && strlength(string(pythonExecutable)) > 0
    if string(env.Status) == "Loaded"
        if ~strcmpi(char(env.Executable), char(pythonExecutable))
            error('V22:PythonAlreadyLoaded', ...
                'Python is already loaded from %s. Restart MATLAB before switching to %s.', ...
                char(env.Executable), char(pythonExecutable));
        end
    else
        pyenv('Version', char(pythonExecutable));
    end
end
env = pyenv;
if string(env.Version) ~= "3.7"
    error('V22:PythonVersion', ['This release requires CARLA 0.9.11 and 64-bit Python 3.7. ', ...
        'Before Python is loaded, call evids_test_initialize_python(fullPathToPythonExe).']);
end
pythonDir = fullfile(root, 'python');
% 使用用户自己的CARLA客户端，不随项目分发egg/wheel。
if nargin < 2 || strlength(string(carlaEgg)) == 0
    carlaEgg = getenv('CARLA_EGG');
end
if strlength(string(carlaEgg)) > 0 && ~isfile(carlaEgg)
    error('V22:CarlaEgg', 'CARLA_EGG does not exist: %s', char(carlaEgg));
end
if int64(py.sys.maxsize) <= int64(2147483647)
    error('V22:PythonArchitecture', '64-bit Python is required.');
end
paths = {pythonDir};
if strlength(string(carlaEgg)) > 0; paths{end + 1} = char(carlaEgg); end
for k = 1:numel(paths)
    if count(py.sys.path, paths{k}) == 0
        insert(py.sys.path, int32(0), paths{k});
    end
end
try
    py.importlib.import_module('carla');
catch ME
    error('V22:CarlaImport', ...
        'Install the matching CARLA 0.9.11 client or supply its egg path via start_v22 / CARLA_EGG. Details: %s', ME.message);
end
% R2021b对Boost.Python类使用完整py命名空间调用。
probeClient = py.carla.Client('localhost', int32(2000));
if string(probeClient.get_client_version()) ~= "0.9.11"
    error('V22:CarlaVersion', 'This release requires the CARLA 0.9.11 Python client.');
end
module = py.importlib.import_module('evids_test_sensor_manager');
% Refresh this test-only module after a stopped run so spawn fixes take effect.
module = py.importlib.reload(module);
actual = char(py.getattr(module, '__file__'));
assert(strcmpi(actual, fullfile(pythonDir, 'evids_test_sensor_manager.py')), ...
    'Another bundle is already loaded in Python. Restart MATLAB before switching bundles.');
end
