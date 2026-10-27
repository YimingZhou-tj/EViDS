function evids_test_validate_prop_definitions(definitions)
% 在连接/修改CARLA场景之前检查所有用户道具配置。
required = {'Blueprint', 'Transforms', 'Mu', 'UseSurfaceHeight', 'ZScale', 'Tolerance'};
if ~isstruct(definitions) || ~all(isfield(definitions, required))
    error('V22:PropConfiguration', 'configure_v22_props must return the documented struct array.');
end
for k = 1:numel(definitions)
    d = definitions(k);
    if ~(ischar(d.Blueprint) && isrow(d.Blueprint)) && ...
            ~(isstring(d.Blueprint) && isscalar(d.Blueprint))
        error('V22:PropBlueprint', 'Prop %d: Blueprint must be a scalar name.', k);
    end
    if ismissing(string(d.Blueprint)) || strlength(strtrim(string(d.Blueprint))) == 0
        error('V22:PropBlueprint', 'Prop %d: fill in Blueprint.', k);
    end
    validateattributes(d.Transforms, {'numeric'}, ...
        {'2d', 'nonempty', 'ncols', 4, 'real', 'finite'}, mfilename, 'Transforms');
    validateattributes(d.Mu, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
    validateattributes(d.UseSurfaceHeight, {'logical'}, {'scalar'});
    validateattributes(d.ZScale, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
    validateattributes(d.Tolerance, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
end
end
