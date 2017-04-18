function [ pointCloud ] = subsampleCorners( imgname, maskname, depthname, method)

%Load image and mask
img = uint16(imread(imgname));
mask = uint16(imread(maskname));
depth = imread(depthname);

%make a better mask from the original
mask(mask==255)=1;

%Remove the background
%Use 'my'
if strcmp(method, 'my')
    K1 = img(:,:,1).*mask.*depth;
    K2 = img(:,:,2).*mask.*depth;
    K3 = img(:,:,3).*mask.*depth;
    
    M1 = img(:,:,1).*mask;
    M2 = img(:,:,2).*mask;
    M3 = img(:,:,2).*mask;
    
    filter_image = cat(3,M1,M2,M3);
    
    res = cat(3,K1,K2,K3);
    
    I1 = 0.21*filter_image(:,:,1) + 0.72*filter_image(:,:,2) + 0.07*filter_image(:,:,3);
    I2 = 0.21*res(:,:,1) + 0.72*res(:,:,2) + 0.07*res(:,:,3);
    
    C = corner(I2, 'N', 1000, 'method', 'MinimumEigenvalue', 'QualityLevel', 0.001);
    F = corner(I1, 'N', 1000, 'method', 'MinimumEigenvalue', 'QualityLevel', 0.0001);
    
    M = [];
    
    for i=1:length(C(:,1))
        if sum(C(i,:)==F(:,:))>0
            continue
        else
            M = [M; C(i,:)];
        end
    end
    
    %figure(1);
    %imshow(res); hold on; plot(M(:,1), M(:,2), 'r*');
elseif strcmp(method, 'experiment')
    
    K1 = img(:,:,1).*mask.*depth;
    K2 = img(:,:,2).*mask.*depth;
    K3 = img(:,:,3).*mask.*depth;
    
    res = cat(3,K1,K2,K3);
    
    mask = (res(:,:,1) == 0) & (res(:,:,2) == 0) & (res(:,:,3) == 0);
    se = strel('disk', 9);
    maskIdx = imdilate(mask,se);
    
    newImg(:,:,1) = uint8(img(:,:,1)) .* uint8(maskIdx);
    newImg(:,:,2) = uint8(img(:,:,2)) .* uint8(maskIdx);
    newImg(:,:,3) = uint8(img(:,:,3)) .* uint8(maskIdx);
    
    finalImg = uint8(img) - newImg;
    
    I3 = 0.21*finalImg(:,:,1) + 0.72*finalImg(:,:,2) + 0.07*finalImg(:,:,3);
    
    M = corner(I3, 'N', 1000, 'method', 'MinimumEigenvalue', 'QualityLevel', 0.001);
    
    %figure(1);
    %imshow(finalImg); hold on; plot(M(:,1), M(:,2), 'r*');
    
elseif strcmp(method, 'falsecolor')
    %Fuse image and mask using falsecolor method
    res = imfuse(img, mask, 'falsecolor');
elseif strcmp(method, 'blend')
    %Fuse image and mask using blend method
    res = imfuse(img, mask, 'blend');
elseif strcmp(method, 'diff')
    %Fuse image and mask using diff method
    res = imfuse(img, mask, 'diff');
end

[~, ordered]= run(depth);

X = zeros(length(M(:,1)),1);
Y = zeros(length(M(:,1)),1);
Z = zeros(length(M(:,1)),1);

for i=1:length(M(:,1))
    X(i,1) = ordered(M(i,2),M(i,1),1);
    Y(i,1) = ordered(M(i,2),M(i,1),2);
    Z(i,1) = ordered(M(i,2),M(i,1),3);
end

X = X(find(X~=0));
Y = Y(find(Y~=0));
Z = Z(find(Z~=0));

pointCloud = [X Y Z];

end
