function  [points,colors,faces] = GetMeshPart(p,c,f,type)
    points = p;
    colors = c;
    faces = f;
    
    load('gt/monge428Split.mat');
    
    
    if strcmp(type,'train')
        indices = trainIndices;
        if exist('trainFaceIndices','var')
            faceIndices = trainFaceIndices;
        end
    elseif strcmp(type,'test')
        indices = testIndices;
        if exist('trainFaceIndices','var')
            faceIndices = testFaceIndices;
        end
    elseif strcmp(type,'sub28')
        indices = sub28Indices;
        if exist('sub28FaceIndices','var')
            faceIndices = sub28FaceIndices;
        end
    elseif isempty(type)
        indices = [trainIndices;testIndices];
        if exist('trainFaceIndices','var')
            faceIndices = [trainFaceIndices;testFaceIndices];
        end
    else
        error('Unknown type.');
    end
    
    
    points = p(indices,:);
    if ~isempty(c)
        colors = c(indices,:);
    end
    if ~isempty(f)
        faces = f(faceIndices,:);
    end
        
    [~,newFaceReferences] = ismember(faces,indices);
    faces = newFaceReferences;

end