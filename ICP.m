function [ R,t ] = ICP( A1,A2 )
%Initialization
R=zeros(3);
t=zeros(3,1);
%For each point in A1, find the closest point in A2
D= min(square_dist(A1,A2)')';%array of size size(A1,1) * 1 that gives the min distances.
%Refine R and t using Singular Value Decomposition
end

%Redefine the distance function in order to use it for matrix. More
%efficient than pdist.
%A1 is a matrix of size n x 3 (each line correspond to a point)
%A2 is a matrix of size m x 3 (each line correspond to a point)
function D = square_dist(A1, A2)
%Let's apply the following formula: ||a-b||^2 = ||a||^2 + ||b||^2 - 2
%dot(a,b) for any vector a,b of same size. 
%We want to apply it to every line(ie: vector) of our matrix A1 and A2. 
%We want D to be a n * m matrix where D(i,j) is the distance between
%A1(i,:) and A2(j,:)
D = bsxfun(@plus,dot(A1',A1'),dot(A2',A2')')'-2*(A1*A2');
end