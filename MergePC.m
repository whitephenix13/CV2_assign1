function [ PC ] = MergePC( images, s, max_num_iter,tolerance,source_subsample_type,...
    source_nb_sample,target_subsample_type,target_nb_sample,backgroundThreshold )

images = images - s;

for k = 0:s:images % s - step, images - amount of images to proccess
    k2 = k + s;
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
        %WARNING: do not transpose the pointCloud if you read from data.
        %Try also not plot points because there might be too many of them
        [ R,t,A1_transformed,A3 ]= ICP(pointCloud1,pointCloud2,max_num_iter,tolerance,source_subsample_type,source_nb_sample,...
            target_subsample_type,target_nb_sample,true);
        %TODO: what do we do with the points ? 
    else
        warning(strcat(pcdFilename, ' or ', pcdFilename2, ' is not a file'));
    end 
end
end

function str = strNum(numb)
if(numb<10)
    str=strcat('0',num2str(numb));
else
    str=num2str(numb);
end
end

