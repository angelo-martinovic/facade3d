% Prepare training and test data
function PrepareData(obj)

    dl = DispatchingLogger.getInstance();
    
    dl.Log(VerbosityLevel.Info,' - Creating correct descriptor format\n');
    tic;
    [obj.Xtrain,obj.Ytrain,obj.Xtest] = obj.facade_unary_class('get_desc');
    elapsedTime = toc;
    dl.Log(VerbosityLevel.Info,sprintf(' - - done. Elapsed time: %.2f seconds.\n',elapsedTime));
    
end