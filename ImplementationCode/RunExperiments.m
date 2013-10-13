function [ output_args ] = RunExperiments(  )
%RUNEXPERIMENTS Summary of this function goes here
%   Detailed explanation goes here
open SparceAdjacency.mat
sparceAdjacency = ans.sparceAdjacency;

path = WeightedAStar(  sparceAdjacency, [], [], 832, 1231, 1.0 )

end

