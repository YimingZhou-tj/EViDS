function evids_test_save_results(modelName, runFolder)
% Save only this copy's logged outputs; do not save the user's entire workspace.
items = evalin('base', 'whos(''evids_*'')');
payload = struct();
for k = 1:numel(items)
    payload.(items(k).name) = evalin('base', items(k).name);
end
save(fullfile(runFolder, 'simulation_outputs.mat'), '-struct', 'payload', '-v7.3');
required = {'evids_z','evids_mu','evids_detect','evids_imports'};
if ~all(isfield(payload, required))
    error('EVIDS:MissingResults', ...
        'Required outputs were not produced. Check the Simulink diagnostic; partial data was saved.');
end
[time, z] = asRows(payload.evids_z);
[muTime, mu] = asRows(payload.evids_mu);
[detectTime, detected] = asRows(payload.evids_detect);
[importTime, imports] = asRows(payload.evids_imports);
if ~isequal(time, muTime, detectTime, importTime)
    error('EVIDS:ResultTime', 'Core result timestamps differ. Raw outputs have been saved.');
end
assert(size(z,2)==4 && size(mu,2)==4 && size(detected,2)==4 && size(imports,2)==12);
names = {'Time_s','Z_L1_m','Z_R1_m','Z_L2_m','Z_R2_m', ...
    'Mu_L1','Mu_R1','Mu_L2','Mu_R2','Detect_L1','Detect_R1','Detect_L2','Detect_R2'};
writetable(array2table([time,z,mu,detected], 'VariableNames', names), fullfile(runFolder,'wheel_feedback.csv'));
summary = struct('Model',modelName, 'Samples',numel(time),'LastSimulationTime_s',time(end), ...
    'WheelOrder',{{'L1','R1','L2','R2'}}, 'MinZ_m',min(z,[],1),'MaxZ_m',max(z,[],1), ...
    'MinMu',min(mu,[],1),'MaxMu',max(mu,[],1),'DetectedSamples',sum(detected>0,1), ...
    'AllCoreValuesFinite',all(isfinite([z(:);mu(:);detected(:);imports(:)])), ...
    'ImportPackingMatches',max(abs(imports-[z,mu,mu]),[],'all')<1e-9);
fid=fopen(fullfile(runFolder,'signal_summary.json'),'w','n','UTF-8');
if fid<0;error('EVIDS:SummaryFile','Cannot write signal summary.');end
fprintf(fid,'%s\n',jsonencode(summary));fclose(fid);
fig=figure('Visible','off','Color','w');
cleanup=onCleanup(@() close(fig)); %#ok<NASGU>
subplot(3,1,1);plot(time,z);ylabel('Z / m');grid on;legend('L1','R1','L2','R2');
subplot(3,1,2);plot(time,mu);ylabel('Mu');grid on;
subplot(3,1,3);stairs(time,detected);ylabel('Detect');xlabel('Simulation time / s');grid on;ylim([-0.1,1.1]);
exportgraphics(fig,fullfile(runFolder,'wheel_feedback.png'),'Resolution',150);
fprintf('EVIDS results saved: %s\n',runFolder);
if ~any(detected(:)>0)
    fprintf('No registered prop was hit. Check whether the CarSim trajectory crosses the configured prop locations.\n');
end
end
function [time,data]=asRows(value)
if ~isa(value,'timeseries');error('EVIDS:ResultType','Expected Timeseries logging.');end
time=double(value.Time(:));data=double(value.Data);
if size(data,1)==numel(time)
    data=reshape(data,numel(time),[]);
elseif size(data,ndims(data))==numel(time)
    data=reshape(permute(data,[ndims(data),1:ndims(data)-1]),numel(time),[]);
else
    error('EVIDS:ResultShape','Result dimensions do not match its timestamps.');
end
end
