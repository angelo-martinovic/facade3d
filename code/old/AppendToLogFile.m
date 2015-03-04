function AppendToLogFile(msg,logFileLocation)

    msg = ['[' datestr(now) '] ' msg];
	f = fopen(logFileLocation,'a');
    fprintf(f,'%s',msg);
    fclose(f);

end