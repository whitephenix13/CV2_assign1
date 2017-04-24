Most of the hyperparameters are in the main.m file that is the file to run if you want the final results.
MergePC.m is the file where we apply the merge method to the point clouds
Note that in that file, there is a hyperparamter l90 to choose which type of 2.1 merging you want 
(ie: all in frame 0 or all in last frame).
ICP.m is the file where our ICP algorithm is implemented.
testICP.m: this file can be used to test ICP on a specified source and target.
ICP_result.m is the file that we ran to get the metrics of the first part. 