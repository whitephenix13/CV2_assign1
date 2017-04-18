function [ noised_point_cloud ] = add_noise( point_cloud, sigma)
%add noise to the point cloud to test the stability 
noised_point_cloud(:,1)= point_cloud(:,1) + sigma * randn(size(point_cloud,1),1);
noised_point_cloud(:,2)= point_cloud(:,2) + sigma * randn(size(point_cloud,1),1);
noised_point_cloud(:,3)= point_cloud(:,3) + sigma * randn(size(point_cloud,1),1);

end

