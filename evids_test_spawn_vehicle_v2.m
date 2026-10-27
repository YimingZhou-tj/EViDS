function actor = evids_test_spawn_vehicle_v2(world, blueprint, target, diagnosticsFile)
% Resolve at runtime instead of using MATLAB's cached py.module.function name.
pythonDir = fullfile(fileparts(mfilename('fullpath')), 'python');
expectedFile = fullfile(pythonDir, 'evids_ghost_vehicle.py');
if ~isfile(expectedFile)
    error('EVIDS:SpawnModuleMissing', 'Missing ghost spawn module: %s', expectedFile);
end
if count(py.sys.path, pythonDir) == 0
    insert(py.sys.path, int32(0), pythonDir);
end
% Load the exact source file; do not depend on cached namespace discovery.
spec = py.importlib.util.spec_from_file_location('evids_ghost_vehicle', expectedFile);
module = py.importlib.util.module_from_spec(spec);
py.operator.setitem(py.sys.modules, 'evids_ghost_vehicle', module);
loader = py.getattr(spec, 'loader');
loader.exec_module(module);
actualFile = char(py.getattr(module, '__file__'));
if ~strcmpi(actualFile, expectedFile)
    error('EVIDS:SpawnModuleConflict', 'Wrong ghost spawn module loaded: %s', actualFile);
end
if ~logical(py.hasattr(module, 'spawn_ghost_vehicle'))
    error('EVIDS:SpawnFunctionMissing', 'spawn_ghost_vehicle is missing from %s', actualFile);
end
spawnFunction = py.getattr(module, 'spawn_ghost_vehicle');
actor = spawnFunction(world, blueprint, target, char(diagnosticsFile));
end
