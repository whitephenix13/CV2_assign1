%This function computes the ICP of two point clouds with respect to some
%parameter and returns the rotation, the translation and some more
%paramters:

%input: A1: source point cloud in the format of nx3 where n is the number
        %of points
       %A2: target point cloud in the same format as A2 
       %max_num_iter: maximum number of iteration before the ICP stops
       %tolerance: stop condition if the rms converged
       %source_subsample_type: 'all', 'uniform', 'random', 'informative'
       %source_number_sample: desired number of sample when source is
            %subsampled
       %target_subsample_type: same as source but for target
       %target_number_sample: same as source but for target
       %plot: set to true to plot the final ICP result
       %log: set to true to have information on the convergence of ICP
       %t_init: optional, give an t vector as initialisation for ICP
       
%output: Ra,ta: rotation and translation such that A1 * Ra' +ta' = A2 
        %A1_transformed: subsampled point cloud of A1 where we applied Ra
        %and ta.
        %A3 cooresponding points to A1 transformed such that the rms of
        %A1_tranformed and A3 is small
        %converged: boolean indicating the convergence of the algorithm
        %total_rms_val: final rms value 
        %i: number of iteration to convergence(or end of algorithm)
        %elapsed_time: elapsed time to convergence (or end of algorithm)
function [ Ra,ta,A1_transformed,A3,converged,rms_val,i,elapsed_time] = ICP( A1,A2,max_num_iter,tolerance,source_subsample_type,...
    source_number_sample,target_subsample_type,target_number_sample,plot,log, R_init, t_init)
tic;
%Initialization of the rotation and translation.
if(nargin < 11)
    Ra=eye(3,3);
    ta=zeros(3,1);
else
    Ra=R_init;
    ta=t_init;
end
rms_val= 1000000; %rms of each iteration 
%subsample source and target
A1_sub = subsample( A1, source_subsample_type, source_number_sample );
A2_sub = subsample( A2, target_subsample_type, target_number_sample );

A1_transformed= A1_sub *Ra'+ta';%point cloud to match with the target. At each iteration
%this point cloud gets closer to the target.
A3=zeros(size(A1_sub));%closest point from target point cloud (A2_sub) to A1_transformed

%ICP main iteration loop 
for i=1:max_num_iter
    i
    %apply the random subsample at each loop
    if(i>1)
        if(strcmp(source_subsample_type,'random'))
            A1_sub = subsample( A1, source_subsample_type, source_number_sample );
            A1_transformed = (Ra * A1_sub'+ta)';
        end
        if(strcmp(target_subsample_type,'random'))
            A2_sub = subsample( A2, target_subsample_type, target_number_sample );
        end
    end
    %For each point in A1, find the closest point in A2
    Ind= min_square_dist(A1_transformed,A2_sub);%array of size 1 * size(A1,1)that gives the min distances and their indexes
    %use only the closest point in A2 and disregard the others
    A3= A2_sub(Ind,:);
    %Refine R and t using Singular Value Decomposition: based on Least-Squares
    %Rigid Motion Using SVD paper
    %Compute the centroids
    p = sum(A1_transformed) ./ size(A1_transformed,1); %size 1x3
    q = sum(A3) ./ size(A3,1); %size 1x3
    %Compute the centered vectors
    X = A1_transformed - p;
    Y = A3 - q;
    %Compute covariance matrix
    S = (X'*Y);%S is a 3x3 matrix
    %Compute the SVD decomposition
    [U,SIG,V]= svd(S);
    %Evaluate rotation
    diag_values= ones(1,3);
    diag_values(end) = det(V*U');
    R= V * diag(diag_values) * U';%tranformation for regular point cloud representation
    %retrieve translation
    t = q' - R*p';%get a 3x1 translation vector
    %check if the algorithm converged: RMS unchanged
    A1_transformed = (R * A1_transformed'+t)';
    new_rms = RMS(A1_transformed,A3);
    delta_val = abs(rms_val - new_rms);
    %aggregate the rotation and the translation
    Ra=R*Ra;
    ta=R*ta+t;
    %stop condition 
    if(delta_val<=(tolerance*rms_val) || (rms_val < power(10,-15)))
        elapsed_time=toc;
        converged=true;
        if(log)
            disp(strcat('Converged with rms of_',num2str(rms_val), '_after_',num2str(i),'_iterations in _',num2str(elapsed_time)));
        end
        break;
    else
        rms_val=new_rms;
    end
end
if(i==max_num_iter)
    elapsed_time=toc;
    converged=false;
    if(log)
        warning(strcat('Fail converging after_', num2str(max_num_iter),'_iterations: rms=_',num2str(rms_val),'_ time :_', num2str(elapsed_time)));
    end
    elapsed_time=100000;
end
if(plot)
    figure
    scatter3(A1_transformed(:,1),A1_transformed(:,2),A1_transformed(:,3),'blue','.');
    hold on ;
    scatter3(A3(:,1),A3(:,2),A3(:,3),'red','.');

end
end

%Redefine the distance function in order to use it for matrix. More
%efficient than pdist.
%A1 is a matrix of size n x 3 (each line correspond to a point)
%A2 is a matrix of size m x 3 (each line correspond to a point)
%return the indexes such that A2(ind,:) are the closest points to A1
function ind = min_square_dist(A1, A2)
%Let's apply the following formula: ||a-b||^2 = ||a||^2 + ||b||^2 - 2
%dot(a,b) for any vector a,b of same size.
%We want to apply it to every line(ie: vector) of our matrix A1 and A2.
%We want D to be a n * m matrix where D(i,j) is the distance between
%A1(i,:) and A2(j,:)
ind= zeros(size(A1,1),1);
%limit memory usage to 1 GB 
MEMORY_LIMIT = 1000000000;%B
num_elem = floor(MEMORY_LIMIT/(32*size(A2,1)));%number of element of A1 considered at a time
for i=1:(ceil(size(A1,1)/num_elem))
        %elements from ((i-1) * num_elem) +1 to i * num_elem 
        elem_ind = ((i-1) * num_elem)+ 1;
        last_elem_ind = elem_ind + num_elem -1;
        if(last_elem_ind>size(A1,1))
            last_elem_ind=size(A1,1);
        end
        D = bsxfun(@plus,dot(A1(elem_ind:last_elem_ind,:)',A1(elem_ind:last_elem_ind,:)'),dot(A2'...
            ,A2')')'-2*(A1(elem_ind:last_elem_ind,:)*A2');
        [dist,index]=min(D,[],2);%minimum on the line 
         clear D;
        ind(elem_ind:last_elem_ind)=index;
end
end

function val = RMS(A1_transf,A2)
val = sum(sum(((A1_transf-A2).^2),2));
end