%Remove all elements which have their z coordinate bigger than threshold
function [ pointCloud_filtered ] = removeBackground( pointCloud, threshold )
    filtered_indexes = pointCloud(:,3)<threshold;
    pointCloud_filtered=pointCloud(filtered_indexes,:);

end