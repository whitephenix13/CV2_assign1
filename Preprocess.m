function [ C ] = Preprocess( img, mask, method )

%Load image and mask
img = imread(img);
mask = imread(mask);

%make a better mask from the original
mask(mask==255)=1;

%Remove the background
%Use 'my'
if strcmp(method, 'my')
    K1 = img(:,:,1).*mask;
    K2 = img(:,:,2).*mask;
    K3 = img(:,:,3).*mask;
    res = cat(3,K1,K2,K3);   
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

%Convert to grayscale
%Luminosity method
I = 0.21*res(:,:,1) + 0.72*res(:,:,2) + 0.07*res(:,:,3);

%Find corners
C = corner(I);

%Plots
figure(1);
subplot(1,2,1); imshow(res);
subplot(1,2,2); imshow(res); hold on; plot(C(:,1), C(:,2), 'r*');

end

