function [ PC ] = MergePC( Folder1, images, s )

images = images - 1;

for k = 0:s:images % s - step, images - amount of images to proccess
    
  if k<10
      jpgFilename = sprintf('000000000%d.jpg', k);
      maskFilename = sprintf('000000000%d_mask.jpg', k);
      depthFilename = sprintf('000000000%d_depth.png', k);
  else
      jpgFilename = sprintf('00000000%d.jpg', k);
      maskFilename = sprintf('00000000%d_mask.jpg', k);
      depthFilename = sprintf('00000000%d_depth.png', k);
  end  
  
  FileName1 = fullfile(Folder1, jpgFilename);
  FileName2 = fullfile(Folder1, maskFilename);
  FileName3 = fullfile(Folder1, depthFilename);
  
  if exist(FileName1, 'file')
      
      method = 'my';
      
      [ PC ] = Preprocess( FileName1, FileName2, FileName3, method );
      
      figure(k+1);
      scatter3(PC(:,1),PC(:,2),PC(:,3));
      
  end
  
end

end

