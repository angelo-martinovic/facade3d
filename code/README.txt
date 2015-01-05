Instructions:

Run the script facade_run.m.


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% OLD INSTRUCTIONS! %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
Use the functions in this order:

PrepareData()
    - Creates a work folder in the given directory.
    - Gets all images, rectifies, segments, extracts features, runs detectors on test set.

TrainSVM(classifierName)
    - Uses the output of PrepareData.m to train a superpixel classifier on the training set.
    - Saves the output model in classifier/<classifierName>.mat.

ClassifyWithSVM(classifierName)
    - Uses the pretrained classifier with the given name to classify the images in the test set.
    - Saves the marginal probabilities per superpixel in output-<classifierName>/<imageName>.marginals.txt.

LabelImagesATLAS(classifierName)
    - Collects the calculated marginals and the detector output on the test set.
    - Runs the first two layers of ATLAS.
    - Saves the 2D potentials (svm and detectors) and layer labelings as mat files in
      output-<classifierName>/<imageName>_2Dpotentials.mat .

--------
Third layer
--------
    
Project2DOntoPointCloud(classifierName,modelName)
    - Projects the 2D potentials from output-<classifierName>/<imageName>_2Dpotentials.mat to the point cloud.
    - Also performs majority voting for point cloud labeling.
    - Saves the labeled point cloud in work/pcl/models/<modelName>_maj.ply.
    - Saves the projected unary probabilities in pcl/probs/<modelName>_maj.mat.

LabelPointCloud(en1,en2,en3)     
    - Aggregates the projected unary probabilities from different sources
    - Performs MAP and CRF labelings
    - Saves the labeled point clouds in work/pcl/models/<outputPCLName>.ply
      where <outputPCLName> is generated based on the combination of unaries.

EvaluateAll()
    - Evaluates the labelings produced in work/pcl/models/<outputPCLName>.ply 



SplitPointCloud()
    - Splits a point cloud labeling into individual facades as given in a mat file (Hayko's output).
      The .mat file should contain a single number per point, which is the facade index.
    - Outputs:
            facade IDs in                   work/pcl/split/<modelName>_facadeIDs.mat
            our facade labelings pcls in    work/pcl/split/<modelName>_split_<facadeIdx>_labeling.ply
                           & potentials     work/pcl/split/<modelName>_split_<facadeIdx>_potentials.mat
            gt facade labelings pcls in     work/pcl/split/<modelName>_split_<facadeIdx>_GT.ply
            original color facades pcls in  work/pcl/split/<modelName>_split_<facadeIdx>_colors.ply

FitPlanes()
    - Fits a plane to the set of points of a single facade.
    - Outputs: 
            facade planes in                work/pcl/split/<modelName>_split_<facadeIdx>_plane.mat
   
OrthoImages()
    - Projects the 3D labeling onto the individual facade planes.
    - Outputs:
        ortho facade labelings in           work/pcl/split/<modelName>_split_<facadeIdx>_ortho_labeling.png
        ortho original facade colors in     work/pcl/split/<modelName>_split_<facadeIdx>_ortho_colors.png
        ortho facade potentials in          work/pcl/split/<modelName>_split_<facadeIdx>_ortho_potentials.mat
    

RunThirdLayer()
    - Runs the 2D third layer of ATLAS on the ortho images
    - Can be configured to run either locally (single-core) or over condor (multi-core, default 12 cores)
    - Performs rectification if necessary
    - IMPORTANT: Rectification will fail on remote machines (no python cv2 module). Run first locally!
    - Outputs:
        relabeled ortho facade labelings in work/pcl/split/<modelName>_split_<facadeIdx>_ortho_labeling_layer3.png

OrthoImagesBackProject()
    - Re-projects the ortho facade labelings onto the respective facade point clouds
    - Outputs:
        3rd layer labeling pcls in          work/pcl/split/<modelName>_split_<facadeIdx>_labeling_layer3.ply

ReassemblePointCloud()
    - Combines the parts of the point cloud labeled with the 3rd layer in one large mesh file.
    - Outputs:
        3rd layer labeling pcl in          work/pcl/models/<modelName>_2Dlayer3.ply


------
Preparation of meshes / point clouds
------
1. From Hayko's mesh to a mesh with colors/normals
    load /usr/data/amartino/monge428New/data/0_fullResMesh/mesh.mat
    ExportMesh('/usr/data/amartino/monge428New/data/0_fullResMesh/mesh.ply',vertex',[],[],face',[]);
    meshlab > open mesh > normalize vertex normals > export to 'mesh_normals.ply'
    ProjectImagesOntoPointCloud()
2. Generate depth maps
    BatchGenerateDepthMaps
3. Generate the split information
    SplitMeshToSplitPointCloud
