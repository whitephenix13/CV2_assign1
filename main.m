
useTest = true;
source_subsample_type = 'random';%'all', 'uniform', 'random', 'informative'
target_subsample_type = 'uniform';%'all', 'uniform', 'random', 'informative'
source_nb_sample = 1000;
target_nb_sample = 1000;
if(useTest)
    pointCloud1 = (load('source.mat'));
    pointCloud1 = pointCloud1.source;
    pointCloud2 = (load('target.mat'));
    pointCloud2 = pointCloud2.target;
    [R,t]= ICP(pointCloud1',pointCloud2',100,0.01,source_subsample_type,source_nb_sample,...
        target_subsample_type,target_nb_sample,true);
else
    pointCloud1 = readPcd('data/0000000001.pcd');
    pointCloud2 = readPcd('data/0000000002.pcd');
    %TODO: prepocess the data: mask the image and the point clouds
    %TODO...
        
    %WARNING: do not transpose the pointCloud if you read from data.
    %Try also not plot points because there might be too many of them

    [R,t]= ICP(pointCloud1,pointCloud2,100,0.01,source_subsample_type,source_nb_sample,...
        target_subsample_type,target_nb_sample,false);
end


