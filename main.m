useTest = false;
backgroundThreshold = 2;
source_subsample_type = 'uniform';%'all', 'uniform', 'random', 'informative'
target_subsample_type = 'uniform';%'all', 'uniform', 'random', 'informative'
%for informative, we create are own point cloud from the image, for
%consistency reasons, both source and target points clouds need to be
%generated from the same way 
if( (strcmp(source_subsample_type,'informative')) || (strcmp(source_subsample_type,'informative')))
    source_subsample_type='informative';
    target_subsample_type='informative';
end
source_nb_sample = 500;
target_nb_sample = 500;
max_num_iter=100;
tolerance=0.01; %in percentage ie 1%
merging_type='global'; %local, global: local is for section 2.1, global is for section 2.2
if(useTest)
    pointCloud1 = (load('source.mat'));
    pointCloud1 = pointCloud1.source;
    pointCloud2 = (load('target.mat'));
    pointCloud2 = pointCloud2.target;
    [ R,t,A1_transformed,A3 ]= ICP(pointCloud1',pointCloud2',max_num_iter,tolerance,source_subsample_type,source_nb_sample,...
        target_subsample_type,target_nb_sample,true,true);
else
    MergePC( 99, 5, max_num_iter,tolerance,source_subsample_type,...
    source_nb_sample,target_subsample_type,target_nb_sample,backgroundThreshold,merging_type)
end


