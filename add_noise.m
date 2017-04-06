function [ noised_point_cloud ] = add_noise( point_cloud, sigma)
%add noise to the point cloud to test the stability 
noised_point_cloud= point_cloud + sigma * randn(size(point_cloud));
end

