function [ final_point_cloud,R,t,rms,nb_iter,elapsed_time] = MergePC( images, s, max_num_iter,tolerance,source_subsample_type,...
    source_nb_sample,target_subsample_type,target_nb_sample,backgroundThreshold,merging_type,noise_sigma)
MEMORY_LIMIT = 2000;%MB
%half the memory will be allocated for final_point_cloud, half of it for
%the bsx function in ICP to find min distance between point 
final_pc_size= min(floor(MEMORY_LIMIT*1000000/192),MEMORY_LIMIT*1000000/(64*target_nb_sample));
    
final_point_cloud= zeros(final_pc_size,3); % divide by 3 * 8(octet size) such that the array has a memory usage of MEMORY LIMIT
final_cloud_index= 1;
skip_next = false;

Ra = eye(3, 3);
ta = zeros(1, 3);
%test 
for k2 = s:s:images % s - step, images - amount of images to proccess
    k2
    k = k2 - s;
    if((skip_next))
        skip_next=false;
        continue;
    end
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
        %introduce noise
        if(noise_sigma ~=0)
            pointCloud1 = add_noise(pointCloud1,noise_sigma);
            pointCloud2 = add_noise(pointCloud2,noise_sigma);
        end
        %filter by distance (mask)
        if(strcmp(source_subsample_type,'informative'))
            method ='my' ; % my,experiment, falsecolor,blend,diff
            pointCloud1 = subsampleCorners( jpgFilename, maskFilename, depthFilename, method);
            pointCloud2 = subsampleCorners( jpgFilename2, maskFilename2, depthFilename2, method);
        else
            pointCloud1 = removeBackground( pointCloud1, backgroundThreshold );
            pointCloud2 = removeBackground( pointCloud2, backgroundThreshold );
        end
        if(strcmp(merging_type,'local'))
            [ R,t,A1_transformed,new_PC_merged,converged,rms,nb_iter,elapsed_time]= ICP(pointCloud1,pointCloud2,max_num_iter,tolerance,source_subsample_type,source_nb_sample,...
                target_subsample_type,target_nb_sample,false,false);
            if(converged)
                if(k2==s)
                    final_cloud_index=size(A1_transformed,1);
                    final_point_cloud(1:final_cloud_index,:)=A1_transformed;
                end
                %transform all previous points in same coordinate system
                final_point_cloud(1:final_cloud_index,:)= (R * final_point_cloud(1:final_cloud_index,:)')'+t;
                %merge points
                new_size = final_cloud_index + size(new_PC_merged,1)-1;
                final_point_cloud(final_cloud_index:new_size,:)=new_PC_merged;
                final_cloud_index=new_size;
            else
                disp(strcat('Skip frame ', num2str(k2)));
                %the frame k2 is problematic, skip it
                skip_next=true;
            end
        else %= global
            %Match the new point cloud (source) to the final point cloud
            %(target)
            if(k2==s)
                [ R,t,new_PC_merged,finalPC_subsampled,converged,rms,nb_iter,elapsed_time]= ICP(pointCloud2,pointCloud1,max_num_iter,tolerance,...
                    source_subsample_type,source_nb_sample,...
                    target_subsample_type,target_nb_sample,false,true);
                if(converged)
                    final_cloud_index=size(finalPC_subsampled,1);
                    final_point_cloud(1:final_cloud_index,:)=finalPC_subsampled;
                end
            else
                [ R,t,new_PC_merged,finalPC_subsampled,converged,rms,nb_iter,elapsed_time]= ICP(pointCloud2,final_point_cloud(1:final_cloud_index,:),...
                    max_num_iter,tolerance,source_subsample_type,source_nb_sample,...
                    'all',target_nb_sample,false,true);
            end
            if(converged)
                %merge points
                new_size = final_cloud_index + size(new_PC_merged,1)-1;
                final_point_cloud(final_cloud_index:new_size,:)=new_PC_merged;
                final_cloud_index=new_size;
                %tranform the final point cloud into the new point cloud system
                %coordinate: minimize the data shift : Y = (RX')' +t => Y =
                %X' = (Y-t) (R^-1)
                final_point_cloud(1:final_cloud_index,:) = (final_point_cloud(1:final_cloud_index,:) - t) / inv(R);

            else
                %the frame k2 is problematic, skip it 
                skip_next=true;
            end
        end        
    else
        warning(strcat(pcdFilename, ' or ', pcdFilename2, ' is not a file'));
    end
end
final_point_cloud((final_cloud_index+1):end,:)=[]; %remove all unused space
%plot final result
figure
%C=jet(size(final_point_cloud(:,1),1));
%C=C(:,1);
if(images == 1)
    target_size = size(new_PC_merged,1);
    source_size = size(final_point_cloud,1) - target_size;
    %TODO: comment out
    %source
    %scatter3(final_point_cloud(1:source_size,1),final_point_cloud(1:source_size,2),final_point_cloud(1:source_size,3),'blue','.');
    %hold on;
    %target
    %scatter3(final_point_cloud(source_size+1:end,1),final_point_cloud(source_size+1:end,2),final_point_cloud(source_size+1:end,3),'red','.');
else
    C=round(linspace(1,round((images-s)/s)+1,size(final_point_cloud(:,1),1)))';
    colormap jet;
    cmap = colormap;
    fscatter3(final_point_cloud(:,1),final_point_cloud(:,2),final_point_cloud(:,3),C(:,1),cmap);
end
end

function str = strNum(numb)
if(numb<10)
    str=strcat('0',num2str(numb));
else
    str=num2str(numb);
end
end

