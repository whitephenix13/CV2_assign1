function [ sub_point_cloud, indexes ] = subsample( point_cloud,number_points, mode )
%point cloud has to be of size nx3
%mode = 'all', 'uniform', 'random', 'informative'
n =size(point_cloud,1);
if(strcmp(mode,'all'))
    sub_point_cloud=point_cloud;
    indexes= linspace(1,n,n);
elseif(strcmp(mode,'uniform'))
    indexes= int64(linspace(1,n,number_points));
    sub_point_cloud=point_cloud(indexes,:);
elseif(strcmp(mode,'random'))
    %generate number_points random points between 1 and
    %n+1 
    r= int64(rand(1,number_points)* n);
    %make sure the points are between 1 and n
    r(r==(n+1))= n;
    indexes=r;
    sub_point_cloud=point_cloud(r,:);
elseif(strcmp(mode,'informative'))
    %TODO: No idea what to do there... maybe your corner detection? 
else
    warning(strcat('The mode_',mode,' does not exists for subsample'));
end

