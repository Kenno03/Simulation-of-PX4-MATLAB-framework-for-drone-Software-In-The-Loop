function offset=controller_clock_offset(dataset)
%CONTROLLER_CLOCK_OFFSET Time offset between the PX4_Receive log and the plant log [s].
%   offset = controller_clock_offset(controllerData) uses the PX4 microsecond timestamps recorded in
%   the PX4_Receive SDI dataset (any logged field named timestamp) and returns the median offset that
%   maps controller time onto plant (simulation) time: t_plant = t_controller + offset.
%   Recorded timestamps are used, never a cross-correlation of motor signals.
candidates=[];
for k=1:dataset.numElements
 e=dataset.getElement(k);candidates=[candidates;walk(e.Values,e.Name)]; %#ok<AGROW>
end
assert(~isempty(candidates),'No recorded PX4 timestamp is available for exact clock mapping.');
offset=median(candidates);
end
function found=walk(value,path)
found=[];
if isstruct(value)
 fields=fieldnames(value);
 for j=1:numel(fields)
  found=[found;walk(value.(fields{j}),[path '.' fields{j}])]; %#ok<AGROW>
 end
elseif isa(value,'timeseries') && (endsWith(path,'.timestamp') || strcmp(path,'<timestamp>'))
 t=double(value.Time(:));raw=double(value.Data(:))/1e6;
 if numel(raw)~=numel(t),return;end
 valid=isfinite(raw)&raw>0&t>1;
 if nnz(valid)<20 || range(t(valid))<2,return;end
 fit=polyfit(t(valid),raw(valid),1);
 if abs(fit(1)-1)<0.02,found=median(raw(valid)-t(valid));end
end
end
