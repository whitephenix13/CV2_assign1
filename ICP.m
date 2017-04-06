function [ R,t ] = ICP( A1,A2,max_num_iter,tolerance,source_subsample_type,...
    source_number_sample,target_subsample_type,target_number_sample,plot)
%tolerance is in percentage
%Initialization
tic;
R=zeros(3);
t=zeros(1,3);
rms_val  = 10000000;

%subsample source and target
A1_sub = subsample( A1, source_subsample_type, source_number_sample );
A2_sub = subsample( A2, target_subsample_type, target_number_sample );

A1_transformed= A1_sub;
A3=zeros(size(A2_sub));
if(plot)
    figure
    plot3(A1_sub(:,1),A1_sub(:,2),A1_sub(:,3),'color','blue');
    hold on ;
    plot3(A2_sub(:,1),A2_sub(:,2),A2_sub(:,3),'color','red');
end

for i=1:max_num_iter
    %apply the random subsample at each loop
    if(i>1)
        if(strcmp(source_subsample_type,'random'))
            A1_sub = subsample( A1, source_subsample_type, source_number_sample );
            A1_transformed = (R * A1_sub')' +t;
        end
        if(strcmp(target_subsample_type,'random'))
            A2_sub = subsample( A2, target_subsample_type, target_number_sample );
        end
    end
    %For each point in A1, find the closest point in A2
    [Dist,Ind]= min(square_dist(A1_transformed,A2_sub),[],2);%array of size 1 * size(A1,1)that gives the min distances and their indexes
    %use only the closest point in A2 and disregard the others
    A3= A2_sub(Ind,:);
    %Refine R and t using Singular Value Decomposition: based on Least-Squares
    %Rigid Motion Using SVD paper
    %Compute the centroids
    p = sum(A1_sub) ./ size(A1_sub,1);
    q = sum(A3) ./ size(A3,1);
    %Compute the centered vectors
    X = A1_sub - p;
    Y = A3 - q;
    %Compute covariance matrix
    S = (X'*Y)/size(X,1);%S is a 3x3 matrix
    %Compute the SVD decomposition
    [U,SIG,V]= svd(S);
    %Evaluate rotation
    diag_values= ones(1,size(X,2));
    diag_values(end) = det(V*U');
    R= V * diag(diag_values) * U';
    %retrieve translation
    t = (q' - R*p')';%get a 1x3 translation vector
    %check if the algorithm converged: RMS unchanged
    A1_transformed = (R * A1_sub')' +t;
    new_rms = RMS(A1_transformed,A3);
    delta_val = abs(rms_val - new_rms);
    if(delta_val<(tolerance*rms_val))
        elapsed_time=toc;
        disp(strcat('Converged with rms of_',num2str(rms_val), '_after_',num2str(i),'_iterations in _',num2str(elapsed_time)));
        break;
    else
        rms_val = new_rms;
    end
end
if(i==max_num_iter)
    elapsed_time=toc;
    warning(strcat('Fail converging after_', num2str(max_num_iter),'_iterations: rms=_',num2str(rms_val),'_ time :_', num2str(elapsed_time)));
end
if(plot)
    figure
    %plot3(A1_transformed(:,1),A1_transformed(:,2),A1_transformed(:,3),'color','blue');
    scatter3(A1_transformed(:,1),A1_transformed(:,2),A1_transformed(:,3),'blue')
    hold on ;
    %plot3(A3(:,1),A3(:,2),A3(:,3),'color','red');
    scatter3(A3(:,1),A3(:,2),A3(:,3),'red');
end
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

function val = RMS(A1_transf,A2)
val = sum(sum(((A1_transf-A2).^2),2));
end