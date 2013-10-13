function [ outpath, explored ] = WeightedAStar(  adjacencyMatrix, costMatrix1, costMatrix2, start, goal, weight )
%WEIGHTEDASTAR Summary of this function goes here
%   Detailed explanation goes here

outpath = [];
if(start == goal)
    outpath = start;
    return;
end
startNode.id   = start;
startNode.g    = 0;
startNode.h    = 0;
startNode.cost = 0;
startNode.path = [startNode.id];
%Dummy node added to deal with how matlab treats struct arrays - never used
dummyNode.id = -1;
dummyNode.g = -1;
dummyNode.h = -1;
dummyNode.cost = inf;
dummyNode.path = [dummyNode.id];

global frontierNodes;

frontierNodes = [startNode, dummyNode];
explored = [];
num_explored = 1;

while max(size((frontierNodes))) > 1
        lowestCostNode = popLowestCostNode();
        adjacencyMatrix(:,lowestCostNode.id) = 0;
        num_explored = num_explored + 1;
        explored = [explored; lowestCostNode.id, num_explored];
        if lowestCostNode.id == goal
            outpath = lowestCostNode.path;
            return;
        end
        costs = findAdjacenciesAndCosts(lowestCostNode.id, adjacencyMatrix, costMatrix1, costMatrix2);
        for edge = 1:length(costs(:,1)) 
            g     = lowestCostNode.cost + costs(edge,2)*weight + costs(edge,3)*(1-weight);
            findInfrontierNodes = getNodeById(costs(edge,1));
            if(min(size(findInfrontierNodes))) ~= 0
                if g < findInfrontierNodes.g
                    findInfrontierNodes.g = g;
                    findInfrontierNodes.cost = findInfrontierNodes.g + findInfrontierNodes.h;
                    findInfrontierNodes.path = [lowestCostNode.path, newNode.id];
                    replaceNodeInFrontier(findInfrontierNodes);
                end
            %If it isn't in the explored set, and not in the frontier set
            else
                newNode.id   = costs(edge,1);
                newNode.g    = g;
                %Heuristic double counts the euclidean distance currently
                newNode.h    = costMatrix1(lowestCostNode.id,edge)*weight + costMatrix1(lowestCostNode.id,edge)*(1-weight);
                newNode.cost = newNode.g + newNode.h;
                newNode.path = [lowestCostNode.path, newNode.id];
                frontierNodes = [frontierNodes, newNode];
            end   
        end
end
outpath = [];

end

function lowestCostNode = popLowestCostNode
    global frontierNodes;
    lowestCostNode = frontierNodes([frontierNodes.cost] == min([frontierNodes.cost]));

    if max(size(lowestCostNode)) > 1
        lowestCostNode = lowestCostNode(1);
    end
    frontierNodes([frontierNodes.id] == lowestCostNode.id) = [];
end

function replaceNodeInFrontier(newNode)
    global frontierNodes;
    frontierNodes([frontierNodes.id] == newNode.id) = newNode;
end

function result = getNodeById(id)
    global frontierNodes;
    result = frontierNodes([frontierNodes.id] == id);
end
            
%%Returns row vectors in format:
% ID1, cost11, cost12
% ID2, cost21, cost22
% ID3, cost31, cost32
function result = findAdjacenciesAndCosts(id, adjacencyMatrix, costMatrix1, costMatrix2)
    [outNeighbors] = find(adjacencyMatrix(id,:));
    cost1 = costMatrix1(id,outNeighbors);
    cost2 = costMatrix2(id,outNeighbors);
    result = [outNeighbors', cost1', cost2'];
end

function [euclideanDistance] = EuclideanDistance(x1,y1,x2,y2)
    euclideanDistance =  sqrt(s(x1-x2).^2 + (y1-y2).^2);
end
