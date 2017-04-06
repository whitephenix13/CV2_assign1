
useTest = true;
if(useTest)
    pointCloud1 = (load('source.mat'));
    pointCloud1 = pointCloud1.source;
    pointCloud2 = (load('target.mat'));
    pointCloud2 = pointCloud2.target;
    [R,t]= ICP(pointCloud1',pointCloud2',100,0.1,true);
else
    pointCloud1 = readPcd('data/0000000001.pcd');
    pointCloud2 = readPcd('data/0000000002.pcd');
    %WARNING: do not transpose the pointCloud if you read from data.
    %Try also not plot points because there might be too many of them
    [R,t]= ICP(pointCloud1,pointCloud2,100,0.01,false);
end


