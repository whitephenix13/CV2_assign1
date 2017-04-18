backgroundThreshold = 2;
source_subsample_type = 'random';%'all', 'uniform', 'random', 'informative'
target_subsample_type = 'all';%'all', 'uniform', 'random', 'informative'
%for informative, we create are own point cloud from the image, for
%consistency reasons, both source and target points clouds need to be
%generated from the same way 
if( (strcmp(source_subsample_type,'informative')) || (strcmp(source_subsample_type,'informative')))
    source_subsample_type='informative';
    target_subsample_type='informative';
end
source_nb_sample = 10000;
target_nb_sample = 1000;
max_num_iter=50;
tolerance=0.05; %in percentage ie 5%
merging_type='local'; %local, global: local is for section 2.1, global is for section 2.2
noise_sigma = 0.01; 

nb_test_loop = 10;
compute=true;
if(compute)
for run=1:nb_test_loop
    for i=1:2
        noise=0;
        if(i==2)
            noise= noise_sigma;
        end
        [ PC,R,t,rms,nb_iter,elapsed_time]=MergePC( 1, 1, max_num_iter,tolerance,source_subsample_type,...
            source_nb_sample,target_subsample_type,target_nb_sample,backgroundThreshold,merging_type,noise);
        filename = strcat(num2str(run),'_',source_subsample_type,'_',num2str(source_nb_sample),'_' , num2str(noise));
        save( strcat('ICP _results/',filename,'.mat'),'R','t','rms','nb_iter','elapsed_time');
    end
end
end

accuracy =0.0;
speed = 0.0;
stability_R =0.0;
mean_R = zeros(3,3);
stability_t =0.0;
mean_t = zeros(1,3); 
noise_tol = 0.0; 
for run=1:nb_test_loop
    filename = strcat(num2str(run),'_',source_subsample_type,'_',num2str(source_nb_sample),'_' , num2str(0));
    f=load(strcat('ICP _results/',filename,'.mat'));
    accuracy=accuracy + f.rms;
    speed = speed + f.elapsed_time;
    %R stability
    mean_R=mean_R+R;
    %t stability
    mean_t=mean_t+t;
    %noise tolerance
    filename2 = strcat(num2str(run),'_',source_subsample_type,'_',num2str(source_nb_sample),'_' , num2str(noise_sigma));
    f2=load(strcat('ICP _results/',filename2,'.mat'));
    delta_time = f2.elapsed_time-f.elapsed_time;
    noise_tol=noise_tol+delta_time;
end
accuracy=accuracy/nb_test_loop;
speed=speed/nb_test_loop;
noise_tol=noise_tol/nb_test_loop;

mean_R=mean_R./nb_test_loop;
mean_t=mean_t./nb_test_loop;

for run=1:nb_test_loop
    filename = strcat(num2str(run),'_',source_subsample_type,'_',num2str(source_nb_sample),'_' , num2str(0));
    f=load(strcat('ICP _results/',filename,'.mat'));
    stability_R=stability_R+norm(f.R-mean_R);
    stability_t=stability_t+norm(f.t-mean_t);
end
stability_R=stability_R/nb_test_loop;
stability_t=stability_t/nb_test_loop;

disp(strcat(source_subsample_type,'_',num2str(source_nb_sample),'_' , num2str(noise),':'));
disp(strcat('accuracy:__',num2str(accuracy)));
disp(strcat('speed:__',num2str(speed)));
disp(strcat('stability R:__',num2str(stability_R)));
disp(strcat('stability t:__',num2str(stability_t)));
disp(strcat('noise tolerance:__',num2str(noise_tol)));

