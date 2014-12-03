function WaitForCondor(obj)

nWritten = 0;
while(1)
    [~,jobLine]=system('condor_q | grep jobs');
    C = textscan(jobLine,'%d jobs; %d completed, %d removed, %d idle, %d running, %d held, %d suspended');
    nJobs = C{1};
    if nJobs==0
        fprintf('All condor jobs finished!\n');
        return;
    end
    fprintf(repmat('\b',1,nWritten));
    fprintf('%s',jobLine);
    nWritten = length(jobLine);
    pause(1);
end
    

end