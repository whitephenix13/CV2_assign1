backgroundThreshold = 2;
source_subsample_type = 'random';%'all', 'uniform', 'random', 'informative'
target_subsample_type = 'all';%'all', 'uniform', 'random', 'informative'
%for informative, we create are own point cloud from the image, for
%consistency reasons, both source and target points clouds need to be
%generated from the same way 
source_nb_sample = 1000;
target_nb_sample = 1000;
max_num_iter=50;
tolerance=0.05; %in percentage ie 5%
merging_type='local'; %local, global: local is for section 2.1, global is for section 2.2
%to do some test, set the first part to 1 and 1 : this will apply ICP to
%the first two frames
noise_sigma = 0; 

A1= load('source.mat');
A1=A1.source';
A2= load('target.mat');
A2=A2.target';
[ R,t,A1_transformed,A3,converged,rms_val,i,elapsed_time] = ICP( A1,A2,max_num_iter,tolerance,source_subsample_type,...
    source_nb_sample,target_subsample_type,target_nb_sample,true,true);