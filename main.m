backgroundThreshold = 1.6;
source_subsample_type = 'random';%'all', 'uniform', 'random', 'informative'
target_subsample_type = 'random';%'all', 'uniform', 'random', 'informative'
%for informative, we create are own point cloud from the image
source_nb_sample = 1000;
target_nb_sample = 1000;
%For ICP there are two criterion for stoppping: either the number of
%iteration execeed the maximum number of iteration or the change of rms is
%less than tolerance * original rms .
max_num_iter=50;
tolerance=0.05; %in percentage ie 5%
merging_type='local'; %local, global: local is for section 2.1, global is for section 2.2
%set the noise_sigma value to add some noise to the data 
noise_sigma = 0; 
%MergePC(max image index, step, max number iteration, tolerance for ICP ,
%source subsample type, target subsample type, source number of sample, target number of sample 
%background threshold value, method for merging point cloud, noise value of
%data.
MergePC( 20, 1, max_num_iter,tolerance,source_subsample_type,...
    source_nb_sample,target_subsample_type,target_nb_sample,backgroundThreshold,merging_type,noise_sigma);


