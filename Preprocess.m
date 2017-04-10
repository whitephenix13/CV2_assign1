function [ PC ] = Preprocess( img, mask, depth, method )

%Load image and mask
img = uint16(imread(img));
mask = uint16(imread(mask));
depth = imread(depth);

%make a better mask from the original
mask(mask==255)=1;

%Remove the background
%Use 'my'
if strcmp(method, 'my')
    K1 = img(:,:,1).*mask.*depth;
    K2 = img(:,:,2).*mask.*depth;
    K3 = img(:,:,3).*mask.*depth;
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

[~, ordered]= run(depth);

X = zeros(length(C(:,1)),1);
Y = zeros(length(C(:,1)),1);
Z = zeros(length(C(:,1)),1);

for i=1:length(C(:,1))
    X(i,1) = ordered(C(i,2),C(i,1),1);
    Y(i,1) = ordered(C(i,2),C(i,1),2);
    Z(i,1) = ordered(C(i,2),C(i,1),3);
end

X = X(find(X~=0));
Y = Y(find(Y~=0));
Z = Z(find(Z~=0));

PC = [X Y Z];

%Plots
figure(1);
subplot(1,3,1); imshow(res);
subplot(1,3,2); imshow(res); hold on; plot(C(:,1), C(:,2), 'r*');
subplot(1,3,3); imshow(depth); hold on; plot(C(:,1), C(:,2), 'r*');
figure(2);
scatter3(X,Y,Z);

end

