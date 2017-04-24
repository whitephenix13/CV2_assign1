%Function to merge all point cloud together to get one coherant point cloud
%input: images: maximum index of the image from data to read
%s: step size (step of 2 would skip one image every time)
%max_num_iter: ICP parameter
%tolerance: ICP parameter
%source_subsample_type: ICP parameter
%source_nb_sample: ICP parameter
%target_subsample_type: ICP parameter
%target_nb_sample: ICP parameter
%backgroundThreshold: depth value to filter out.
%merging_type: 'local'(part 2.1) or 'global'(part 2.2)
%noise_sigma: value of sigma for the noise added to the data

%output: final_point_cloud: the final result of the merging
%Ra,ta: the aggregated rotation and translation to go from frame 0
%to the last frame
%rms,nb_iter,elapsed_time: metrics values
function [ final_point_cloud,Ra,ta,rms,nb_iter,elapsed_time] = MergePC( images, s, max_num_iter,tolerance,source_subsample_type,...
    source_nb_sample,target_subsample_type,target_nb_sample,backgroundThreshold,merging_type,noise_sigma)
%memory allocated for the final point cloud and ICP algorithm: this prevent memory overflow
MEMORY_LIMIT = 2000;%MB
%half the memory will be allocated for final_point_cloud, half of it for
%the bsx function in ICP to find min distance between point
final_pc_size= min(floor(MEMORY_LIMIT*1000000/192),MEMORY_LIMIT*1000000/(64*target_nb_sample));

%final merged result
final_point_cloud= zeros(final_pc_size,3);
%index of the last element added
final_cloud_index= 0;
%vector keeping track of which points in final point cloud belong to which
%point cloud (used for scatter)
scatterColor = zeros(final_pc_size,1);
%next color to use for the next point cloud (used for scatter)
scatterColorIndex=1;
%boolean to indicate that the current merging did not converge and hence,
%that the frame has to be skipped
skip_next = false;

%index of the source frame to read
k=0;
%index of the target frame to read
k2=s;

%aggregated rotation and translation
Ra = eye(3,3);
ta=zeros(3,1);

%loop over all the frames
while (k2<= images) % at each step: k=k2 and k2 = k2+s if no skip;;; k2=k2+s if skip
    %print the current frame being merged
    [k,k2]
    %read all the files containing information of the frames
    pcdFilename = strcat('data/00000000',strNum(k),'.pcd');
    jpgFilename = strcat('data/00000000',strNum(k),'.jpg');
    maskFilename = strcat('data/00000000',strNum(k),'_mask.jpg');
    depthFilename = strcat('data/00000000',strNum(k),'_depth.png');
    pcdFilename2 = strcat('data/00000000',strNum(k2),'.pcd');
    jpgFilename2 = strcat('data/00000000',strNum(k2),'.jpg');
    maskFilename2 = strcat('data/00000000',strNum(k2),'_mask.jpg');
    depthFilename2 = strcat('data/00000000',strNum(k2),'_depth.png');
    
    %depending on the method used, read from the depth file or the pcd.
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
        
        %Method from 2.1 for merging: merge consecutive frame
        merge_frame0 = true;%boolean that indicates if the final coordinate system has to be the one of the first frame or last frame
        if(strcmp(merging_type,'local'))
            %initialize the final result by putting in the first point
            %cloud (frame 0)
            if(merge_frame0)
                if(k==0)
                    %add the elements from frame 0 in coordinate system 0
                    sub_pc1 = subsample( pointCloud1, source_subsample_type, source_nb_sample );
                    final_cloud_index=size(sub_pc1,1);
                    final_point_cloud(1:final_cloud_index,:)=sub_pc1; %frame 0 coordinate system
                    %keep track of point cloud for scatter
                    scatterColor(1:final_cloud_index)=ones(final_cloud_index,1)*scatterColorIndex;
                    scatterColorIndex=scatterColorIndex + 1;
                end
                [ R,t,A1_transformed,new_PC_merged,converged,rms,nb_iter,elapsed_time]= ICP(pointCloud1,pointCloud2,...
                    max_num_iter,tolerance,source_subsample_type,source_nb_sample,...
                    target_subsample_type,target_nb_sample,false,true);
                %R and t are such that they transform from coordinate system
                %n-1 (the one from point cloud 1) to coordinate system n(the
                %one from point cloud 2)
                if(converged)
                    %transform the new point cloud to the coordinate system
                    %of frame 0. For that use the newly computed R and t as
                    %well as the aggregated ones Ra and ta 
                    newRa=R*Ra;
                    newta=R*ta+t;
                    if(~strcmp(source_subsample_type,'informative'))
                        new_pc2=subsample( pointCloud2, source_subsample_type, source_nb_sample );
                    end
                    new_pc2=(new_pc2-newta')/newRa';

                    %add the new point in the coordinate system 0
                    new_size = final_cloud_index + size(new_pc2,1);
                    final_point_cloud(final_cloud_index+1:new_size,:)=new_pc2;
                    
                    %memorize the point cloud for scatter
                    scatterColor(final_cloud_index+1:new_size)=ones(size(new_pc2,1),1)*scatterColorIndex;
                    scatterColorIndex=scatterColorIndex + 1;
                    
                    %update the index
                    final_cloud_index=new_size;
                end
            else
                if(k==0)
                    %add the elements from frame 0 in coordinate system 0
                    sub_pc1 = subsample( pointCloud1, source_subsample_type, source_nb_sample );
                    final_cloud_index=size(sub_pc1,1);
                    final_point_cloud(1:final_cloud_index,:)=sub_pc1; %frame 0 coordinate system
                    %keep track of point cloud for scatter
                    scatterColor(1:final_cloud_index)=ones(final_cloud_index,1)*scatterColorIndex;
                    scatterColorIndex=scatterColorIndex + 1;
                end
                [ R,t,A1_transformed,new_PC_merged,converged,rms,nb_iter,elapsed_time]= ICP(pointCloud1,pointCloud2,...
                    max_num_iter,tolerance,source_subsample_type,source_nb_sample,...
                    target_subsample_type,target_nb_sample,false,true);
                %R and t are such that they transform from coordinate system
                %n-1 (the one from point cloud 1) to coordinate system n(the
                %one from point cloud 2)
                if(converged)
                    %transform all previous points in same coordinate system
                    %convert from n-1 coordinate system to n (R and t match n-1
                    %to n)
                    
                    %the new point cloud to add is the one from point cloud 2
                    %after transformation of the final point cloud in the
                    %correct coordinate system
                    new_pc2=pointCloud2;
                    if(~strcmp(source_subsample_type,'informative'))
                        new_pc2=subsample( pointCloud2, source_subsample_type, source_nb_sample );
                    end
                    %tranform frame 0...n-1 from coordinate system n-1 to from coordinate system n
                    %as our point cloud is transposed compared to normal
                    %definition, the transformation formula is transposed too
                    final_point_cloud(1:final_cloud_index,:)= final_point_cloud(1:final_cloud_index,:)*R'+t';
                    
                    %add the new point in the coordinate system n
                    new_size = final_cloud_index + size(new_pc2,1);
                    final_point_cloud(final_cloud_index+1:new_size,:)=new_pc2;
                    
                    %memorize the point cloud for scatter
                    scatterColor(final_cloud_index+1:new_size)=ones(size(new_pc2,1),1)*scatterColorIndex;
                    scatterColorIndex=scatterColorIndex + 1;
                    
                    %update the index
                    final_cloud_index=new_size;
                    
                else
                    disp(strcat('Skip frame ', num2str(k2)));
                    %the frame k2 is problematic, skip it
                    skip_next=true;
                end
            end
        else %= global
            %Match the new point cloud (source) to the final point cloud
            %(target) and then merge the new point cloud in the final point
            %cloud
            
            %initialize the final point cloud
            if(k==0)
                %initialize with points from point cloud 1 (frame 0)
                A1_sub= subsample( pointCloud1, source_subsample_type, source_nb_sample) ;
                final_cloud_index=size(A1_sub,1);
                final_point_cloud(1:final_cloud_index,:)=A1_sub;
                %memorize point cloud for scatter
                scatterColor(1:final_cloud_index)=ones(final_cloud_index,1)*scatterColorIndex;
                scatterColorIndex=scatterColorIndex + 1;
            end
            [ R,t,new_PC_merged,finalPC_subsampled,converged,rms,nb_iter,elapsed_time]= ICP(pointCloud2,final_point_cloud(1:final_cloud_index,:),...
                max_num_iter,tolerance,source_subsample_type,source_nb_sample,...
                'all',target_nb_sample,false,true);
            %R and t are such that they tranform the target point
            %cloud(point cloud 2) in coordinate system n  into the final
            %point cloud in coordinate system n-1
            if(converged)
                %the new point cloud to add is the one from point cloud 2
                %is coordiante system n
                new_pc2 = subsample( pointCloud2, source_subsample_type, source_nb_sample );
                
                %convert the frame 0 ... n-1 from coordinate system n-1 to
                %coordiante system n. Careful R and t tranform for
                %coordinate system n to coordinate system n-1, hence the
                %inverse formula
                final_point_cloud(1:final_cloud_index,:) = (final_point_cloud(1:final_cloud_index,:)-t') /R';
                
                %merge woth point clouds that are all in coordinate system
                %n
                new_size = final_cloud_index + size(new_pc2,1);
                final_point_cloud(final_cloud_index+1:new_size,:)=new_pc2;
                
                %memorize information for scatter
                scatterColor(final_cloud_index+1:new_size)=ones(size(new_pc2,1),1)*scatterColorIndex;
                scatterColorIndex=scatterColorIndex + 1;
                
                %udpate index
                final_cloud_index=new_size;
                
            else
                %the frame k2 is problematic, skip it
                skip_next=true;
            end
        end
        %aggregate R and t
        Ra=R*Ra;
        ta=R*ta+t;
        
        %figure;
        %colormap jet;
        %cmap = colormap;
        %fscatter3(final_point_cloud(1:final_cloud_index,1),final_point_cloud(1:final_cloud_index,2),final_point_cloud(1:final_cloud_index,3),scatterColor(1:final_cloud_index),cmap);
    else
        warning(strcat(pcdFilename, ' or ', pcdFilename2, ' is not a file'));
    end
    if(skip_next)
        disp('skip')
        %frame k2 is problematic: consider frame k and frame k2+s for next
        %run
        k2=k2+s;
        skip_next=false;
    else
        %consider consecutive frames
        k=k2;
        k2=k2+s;
    end
end

%remove unused space
final_point_cloud((final_cloud_index+1):end,:)=[];
scatterColor((final_cloud_index+1):end,:)=[];


%plot final result
figure
%special plot if only two elements in point cloud
if(images == 1)
    target_size = size(new_PC_merged,1);
    source_size = size(final_point_cloud,1) - target_size;
    %source
    scatter3(final_point_cloud(1:source_size,1),final_point_cloud(1:source_size,2),final_point_cloud(1:source_size,3),'blue','.');
    hold on;
    %target
    scatter3(final_point_cloud(source_size+1:end,1),final_point_cloud(source_size+1:end,2),final_point_cloud(source_size+1:end,3),'red','.');
else
    colormap jet;
    cmap = colormap;
    fscatter3(final_point_cloud(:,1),final_point_cloud(:,2),final_point_cloud(:,3),scatterColor,cmap);
end
end

%function to add a '0' to all numbers < 9
function str = strNum(numb)
if(numb<10)
    str=strcat('0',num2str(numb));
else
    str=num2str(numb);
end
end

