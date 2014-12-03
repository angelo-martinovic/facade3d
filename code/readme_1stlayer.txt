## OLD - USE NEW README.txt ##
1. Run VisualSFM
	/esat/sadr/amartino/Code/vsfm/bin/run_VisualSFM

2. Undistort ground truth
	MATLAB > BatchConvertGroundTruthToUndistorted

3. Rectify undistorted images 
	cd /esat/sadr/amartino/Code/rectification
	for i in /usr/data/amartino/monge3dRight/reconstruction.nvm.cmvs/00/visualize/*.jpg; do python rectifyImage.py -t auto -i $i -o ${i%.jpg}.rect.jpg ; done

4. Resize and copy rectified images, rectify undistorted ground truth
	MATLAB > BatchMoveRectifiedData

5. Segment rectified images
	/esat/sadr/amartino/Code/edisonProject/segmentImages.sh /esat/sadr/amartino/monge3dRight/reconstruction.nvm.cmvs/00/visualizeRect/

6. Extract features
	/esat/sadr/amartino/gould/extractFeatures.py /esat/sadr/amartino/monge3dRight/reconstruction.nvm.cmvs/00/visualizeRect/

7. Classify with SVM
	MATLAB
	dl='/usr/data/amartino/monge3dRight/reconstruction.nvm.cmvs/00/visualizeRect/';
	cl='/usr/data/amartino/gould/testMeanShiftNew/output/SVM_cv-FOLD1.mat';
	ClassifyWithSVM(dl,cl,8,225,'monge3d')

8. Create output segmentation maps
	MATLAB
	cloc='/usr/data/amartino/gould/testMeanShiftNew/output/';
	[acc,cm]= ClassifyLabelImages(dl,cloc,8,[0 8],'monge3d');

