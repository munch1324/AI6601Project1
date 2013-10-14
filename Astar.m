function [path, distance,explored] = Astar(dg,Cdist,Ccrime,start,goal,weight,xy)
%
path = [];
distance = 0;
explored = [];

if start == goal
    return
end

horf = EuDist(xy(start,:),xy(goal,:));
startNode = createNode(start,0,horf,horf,[]);

openList(1) = startNode;
closedList = struct('id',{},'g',{},'h',{},'f',{},'path',{});
num_explored = 0;

while ~isempty(openList)
    ind = cost_sort(openList);
    node = openList(ind);
    openList(ind) = [];
    if node.id == goal
        path = node.path;
        for i = 1 : length(path)-1
            distance = distance + Cdist(path(i),path(i+1));
        end
        return
    end
    closedList(end+1) = node;
    ids = findAdjacent(node,dg);
    for edge = ids
        wg = weight*Cdist(node.id,edge)+(1-weight)*Ccrime(node.id,edge);
        g = node.g + wg;
        h = EuDist(xy(edge,:),xy(node.id,:))*floor(weight);
        f = g+h;
        child = createNode(edge,g,h,f,node.path);
        
        if findDuplicate(closedList,child)
            continue
        end
        id = findDuplicate(openList,child);
        if id
            if openList(id).g > child.g
                openList(id) = child;
            end
        else
            openList(end+1) = child;
            num_explored = num_explored + 1;
            explored = [explored;child.id, num_explored];
        end
    end
end

end

%%
function dup = findDuplicate(list,node)
n = length(list);
dup = 0;
for i = 1 : n
    if list(i).id == node.id
        dup = i;
        return;
    end
end

end

%%
function node = createNode(id,g,h,f,path)
node.id   = id;
node.g    = g;
node.h    = h;
node.f    = f;
node.path = [path;node.id];
end

%%
function ids = findAdjacent(node,dg)
[~,ids,~] = find(dg(node.id,:));
end

%%
function ind = cost_sort(openList)
n = length(openList);
costs = zeros(n,1);
for i = 1 : n
    costs(i) = openList(i).f;
end
[~,ind] = sort(costs);
ind = ind(1);
end

%%
function [euclideanDistance] = EuDist(n1,n2)
euclideanDistance =  sqrt(sum((n1-n2).^2));
end