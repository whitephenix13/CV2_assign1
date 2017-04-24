%Script to test ICP and the example containing source and target files

backgroundThreshold = 2;
source_subsample_type = 'random';%'all', 'uniform', 'random', 'informative'
target_subsample_type = 'random';%'all', 'uniform', 'random', 'informative'
%for informative, we create are own point cloud from the image, for
%consistency reasons, both source and target points clouds need to be
%generated from the same way
source_nb_sample = 10000;
target_nb_sample = 10000;
max_num_iter=20;
tolerance=0.1; %in percentage ie 5%
merging_type='local'; %local, global: local is for section 2.1, global is for section 2.2
%to do some test, set the first part to 1 and 1 : this will apply ICP to
%the first two frames
noise_sigma = 0;

%A1= load('source.mat');
%A1=A1.source';
%A2= load('target.mat');
%A2=A2.target';
A1_num = '00';
A2_num = '18';
if(strcmp(source_subsample_type,'informative'))
    method ='my' ; % my,experiment, falsecolor,blend,diff
    pcdFilename = strcat('data/00000000',A1_num,'.pcd');
    jpgFilename = strcat('data/00000000',A1_num,'.jpg');
    maskFilename = strcat('data/00000000',A1_num,'_mask.jpg');
    depthFilename = strcat('data/00000000',A1_num,'_depth.png');
    pcdFilename2 = strcat('data/00000000',A2_num,'.pcd');
    jpgFilename2 = strcat('data/00000000',A2_num,'.jpg');
    maskFilename2 = strcat('data/00000000',A2_num,'_mask.jpg');
    depthFilename2 = strcat('data/00000000',A2_num,'_depth.png');
    %[A1,ordered] = run(depthFilename);
    %[A2,ordered] = run(depthFilename2);
    A1 = subsampleCorners( jpgFilename, maskFilename, depthFilename, method);
    A2 = subsampleCorners( jpgFilename2, maskFilename2, depthFilename2, method);
else
    A1=readPcd(strcat('data/00000000',A1_num,'.pcd'));
    A2=readPcd(strcat('data/00000000',A2_num,'.pcd'));
    A1(:,4)=[];
    A2(:,4)=[];
    A1 = removeBackground( A1, 1.6 );
    A2 = removeBackground( A2, 1.6 );
end

[ R,t,A1_transformed,A3,converged,rms_val,i,elapsed_time] = ICP( A1,A2,max_num_iter,tolerance,source_subsample_type,...
    source_nb_sample,target_subsample_type,target_nb_sample,true,true);
R
t