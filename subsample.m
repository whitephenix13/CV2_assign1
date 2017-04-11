function [ sub_point_cloud, indexes ] = subsample( point_cloud, mode,number_sample,img,mask,depth )
%point cloud has to be of size nx3
%mode = 'all', 'uniform', 'random', 'informative'
n =size(point_cloud,1);
if(strcmp(mode,'all'))
    sub_point_cloud=point_cloud;
    indexes= linspace(1,n,n);
elseif(strcmp(mode,'uniform'))
    %make sure we don't extract more points that we have
    number_sample=min(number_sample,n);
    indexes= int64(linspace(1,n,number_sample));
    sub_point_cloud=point_cloud(indexes,:);
elseif(strcmp(mode,'random'))
    %generate number_points random points between 1 and
    %n+1
    r= int64(rand(1,number_sample)* n)+1;
    %make sure the points are between 1 and n
    r(r==(n+1))= n;
    indexes=r;
    sub_point_cloud=point_cloud(indexes,:);
elseif(strcmp(mode,'informative'))
    %Remove background with mask, use corner detection
    %this has already been done in mergePC 
    sub_point_cloud=point_cloud;
else
    warning(strcat('The mode_',mode,' does not exists for subsample'));
    
    
end

