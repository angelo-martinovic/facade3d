 function InitializeParallel(nWorkers)
    c = parcluster('local');
    c.NumWorkers = nWorkers;

    poolobj = gcp('nocreate');
    if isempty(poolobj)
        parpool(nWorkers);
    end
 end