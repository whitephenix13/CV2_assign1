function [ PC ] = MergePC( images, s, max_num_iter,tolerance,source_subsample_type,...
    source_nb_sample,target_subsample_type,target_nb_sample,backgroundThreshold,merging_type)
MEMORY_LIMIT = 2000;%MB
%half the memory will be allocated for final_point_cloud, half of it for
%the bsx function in ICP to find min distance between point 
final_pc_size= min(floor(MEMORY_LIMIT*1000000/192),MEMORY_LIMIT*1000000/(64*target_nb_sample));
    
final_point_cloud= zeros(final_pc_size,3); % divide by 3 * 8(octet size) such that the array has a memory usage of MEMORY LIMIT
final_cloud_index= 1;
for k2 = s:s:images % s - step, images - amount of images to proccess
    k = k2 - s;
    pcdFilename = strcat('data/00000000',strNum(k),'.pcd');
    jpgFilename = strcat('data/00000000',strNum(k),'.jpg');
    maskFilename = strcat('data/00000000',strNum(k),'_mask.jpg');
    depthFilename = strcat('data/00000000',strNum(k),'_depth.png');
    pcdFilename2 = strcat('data/00000000',strNum(k2),'.pcd');
    jpgFilename2 = strcat('data/00000000',strNum(k2),'.jpg');
    maskFilename2 = strcat('data/00000000',strNum(k2),'_mask.jpg');
    depthFilename2 = strcat('data/00000000',strNum(k2),'_depth.png');
    
    if exist(pcdFilename, 'file') && (exist(pcdFilename2, 'file'))
        if(~strcmp(source_subsample_type,'informative'))
            pointCloud1 = readPcd(pcdFilename);
            pointCloud2 = readPcd(pcdFilename2);
            pointCloud1(:,4)=[];
            pointCloud2(:,4)=[];
        else
            [pointCloud1,ordered] = run(depthFilename);
            [pointCloud2,ordered] = run(depthFilename2);
        end
        %filter by distance (mask)
        if(strcmp(source_subsample_type,'informative'))
            method ='my' ; % my,falsecolor,blend,diff
            pointCloud1 = subsampleCorners( jpgFilename, maskFilename, depthFilename, method);
            pointCloud2 = subsampleCorners( jpgFilename2, maskFilename2, depthFilename2, method);
        else
            pointCloud1 = removeBackground( pointCloud1, backgroundThreshold );
            pointCloud2 = removeBackground( pointCloud2, backgroundThreshold );
        end
        if(strcmp(merging_type,'local'))
            [ R,transl,A1_transformed,A3 ]= ICP(pointCloud1,pointCloud2,max_num_iter,tolerance,source_subsample_type,source_nb_sample,...
                target_subsample_type,target_nb_sample,false,false);
            %transform all previous points in same coordinate system
            final_point_cloud(1:final_cloud_index,:)= (R * final_point_cloud(1:final_cloud_index,:)')'+transl;
            %merge points
            new_size = final_cloud_index + size(A3,1)-1;
            final_point_cloud(final_cloud_index:new_size,:)=A3;
            final_cloud_index=new_size;
        else %= global
            if(k2==s)
                [ R,t,A1_transformed,A3 ]= ICP(pointCloud1,pointCloud2,max_num_iter,tolerance,source_subsample_type,target_nb_sample,...
                    source_subsample_type,source_nb_sample,false,false);
                final_cloud_index=size(A1_transformed,1);
            else
                [ R,t,A1_transformed,A3 ]= ICP(final_point_cloud(1:final_cloud_index,:),pointCloud2,max_num_iter,tolerance,source_subsample_type,target_nb_sample,...
                    source_subsample_type,source_nb_sample,false,false);
            end
            %transform all previous points in same coordinate system
            final_point_cloud(1:size(A1_transformed,1),:)=A1_transformed;
            %merge points
            new_size = final_cloud_index + size(A3,1)-1;
            final_point_cloud(final_cloud_index:new_size,:)=A3;
            final_cloud_index=new_size;
        end        
    else
        warning(strcat(pcdFilename, ' or ', pcdFilename2, ' is not a file'));
    end
end
final_point_cloud((final_cloud_index+1):end,:)=[]; %remove all unused space
%plot final result
figure
scatter3(final_point_cloud(:,1),final_point_cloud(:,2),final_point_cloud(:,3),'blue');
end

function str = strNum(numb)
if(numb<10)
    str=strcat('0',num2str(numb));
else
    str=num2str(numb);
end
end

