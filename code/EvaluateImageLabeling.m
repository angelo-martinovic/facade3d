function score = EvaluateImageLabeling(config,imageNames,outputFolder)
        dl = DispatchingLogger.getInstance();

        numViews = length(imageNames);
        dl.Log(VerbosityLevel.Info,sprintf(' - Evaluating on %d images.\n',numViews));

        % EVALUATION TASK 1 - Image labeling - vanilla 2d img labeling task
        dl.Log(VerbosityLevel.Debug,sprintf(' - - Loading data...\n'));

        [gt, res] = evaluation_load_folder (get_adr('2D_labels',config), outputFolder,...
            {imageNames}, numViews, config.cm);

        dl.Log(VerbosityLevel.Debug,sprintf(' - - Done! Evaluating...\n'));
        score = evaluation_multilabel(gt,res,config.ignoreClasses + 1);
        dl.Log(VerbosityLevel.Debug,sprintf(' - - Done!\n'));

        % Save the output
        saveFilename = [outputFolder 'results.mat'];
        save(saveFilename,'score');

        dl.Log(VerbosityLevel.Info,...
            sprintf([' - Evaluated the 2D labeling of %d images in %s.'...
            'Results stored in %s. Pascal IoU score: %.2f.\n'],...
            numViews,outputFolder,saveFilename,score.mean_pascal));

    end