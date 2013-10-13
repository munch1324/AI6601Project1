function [ outpath ] = WeightedAStar(  adjacencyMatrix, costMatrix1, costMatrix2, start, goal, weight )
%WEIGHTEDASTAR Summary of this function goes here
%   Detailed explanation goes here

outpath = [];
if(start == goal)
    outpath = start;
    return;
end
startNode.id   = start;
startNode.cost = 0;
startNode.path = [startNode.id];
dummyNode.id = -1;
dummyNode.cost = inf;
dummyNode.path = [dummyNode.id];

frontier = [startNode, dummyNode];
explored = [];
num_explored = 0;

while max(size((frontier))) > 1
        lowestCostNode = frontier([frontier.cost] == min([frontier.cost]));
       
        if max(size(lowestCostNode)) > 1
            lowestCostNode = lowestCostNode(1);
        end
        frontier([frontier.id] == lowestCostNode.id) = [];
        explored = [explored, lowestCostNode];
        if lowestCostNode.id == goal
            outpath = lowestCostNode.path;
            return;
        end
        [discard, edges] = find(adjacencyMatrix(lowestCostNode.id,:));
        for edge = edges
            %child = State(graph.node[edge[1]], node, node.pathCost + distance(node.node, graph.node[edge[1]]))
            newNode.cost     = lowestCostNode.cost + 1;%UCS - replace with distance  %distance(node.node, graph.node[edge[1]])
            newNode.id =edge;
            newNode.path = [lowestCostNode.path, newNode.id];
            frontier = [frontier, newNode];
%             if child in frontier:
%                 out = next((x for x in frontier if (x==child)))
%                 if out.g > g:
%                     out.setAStarPathCost(g,out.h)
%                     out.parent = node
% 
%             if (child not in explored) and (child not in frontier):
%                 h =  distance(goal.node, graph.node[edge[1]])
%                 child.setAStarPathCost(g,h)
%                 frontier.append(child)
%                 num_explored = num_explored + 1
    
            end
end

