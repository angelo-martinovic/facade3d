function cameras = ImportCamerasNVM(filename)
    numCameras = 0;
    f = fopen(filename,'r');
    textscan(f,'%s\n',1);
    numCameras = textscan(f,'%s\n',1);
    numCameras = str2double(numCameras{1});

    cameras = cell(numCameras,11);
    for i=1:numCameras
        cameras  = textscan(f,'%s %f %f %f %f %f %f %f %f %f %f\n',1);
        
    end
%% Postprocessing
% dataArray = dataArray{1};

% numCameras = str2double(dataArray{1});

cameras = struct( ...
    'visualizeImageName',{}, ...
    'originalImageFilename',{}, ...
    'focalLength',{}, ...
    'principalPoint',{}, ...
    'calibrationMatrix',{}, ...
    'translation',{}, ...
    'cameraPosition',{}, ...
    'R_AxisAngle',{}, ...
    'R_Quaternion',{}, ...
    'R_Matrix',{}, ...
    'P',{}, ...
    'normRadDistortion',{}, ...
    'latLngAlt',{} ...
    );
    
index = 2;
for i=1:numCameras

    cameras{i}.visualizeImageName = dataArray{index};       % Filename (of the undistorted image in visualize folder)
    cameras{i}.originalImageFilename = dataArray{index+1};  % Original filename
    cameras{i}.focalLength = str2double(dataArray{index+2});%  Focal Length (of the undistorted image)
    cameras{i}.principalPoint = str2num(dataArray{index+3});% 2-vec Principal Point (image center)
    cameras{i}.K = [cameras{i}.focalLength 0 cameras{i}.principalPoint(1);  % Camera calibration matrix K
         0 cameras{i}.focalLength cameras{i}.principalPoint(2); 
         0 0 1];                                
    cameras{i}.translation = str2num(dataArray{index+4});   %#ok<*ST2NM> % 3-vec Translation T (as in P = K[R T])
    cameras{i}.cameraPosition = str2num(dataArray{index+5});% 3-vec Camera Position C (as in P = K[R -RC])
    cameras{i}.R_AxisAngle = str2num(dataArray{index+6});   % 3-vec Axis Angle format of R
    cameras{i}.R_Quaternion = str2num(dataArray{index+7});  % 4-vec Quaternion format of R
        R_Matrix_row1 = str2num(dataArray{index+8});            % 3x3 Matrix format of R
        R_Matrix_row2 = str2num(dataArray{index+9});
        R_Matrix_row3 = str2num(dataArray{index+10});
    cameras{i}.R_Matrix = [R_Matrix_row1;R_Matrix_row2;R_Matrix_row3];
    cameras{i}.P = cameras{i}.K*[cameras{i}.R_Matrix cameras{i}.translation'];         % Projection matrix
    
    cameras{i}.normRadDistortion = str2num(dataArray{index+11});% [Normalized radial distortion] = [radial distortion] * [focal length]^2
    cameras{i}.latLngAlt = str2num(dataArray{index+12});     % 3-vec Lat/Lng/Alt from EXIF

    index = index + 13;
    
end


