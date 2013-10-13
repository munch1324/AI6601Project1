clear all
clc

plot_flag = 1;

%% Prepare Connectivity, Distance, Crime Cost Matrices
% parsed_osm = Data convereted from the osm xml file to matlab struct
% dg(i,j) = Connectivity of node i to node j
% dist(i,j) = Distance of edge(i->j)
% crime(i,j) = Crime intensity of edge(i->j)
% crime_node(i) = Crime value at node i
% var = variance for generating the crime map and values

var = 0.00001;
[parsed_osm, dg, dist, crime, crime_node] = osm2matlab(var);
[num_nodes,~] = size(crime_node);

%% Expriments:
% node_xy(:,i) = lat and long of node i
% weights = Combination proportion of crime and distance objectives



node_xy = parsed_osm.node.xy;
weights = [0, 0.5, 1];

num_exp = 100;
start_nodes = zeros(1,num_exp);
goal_nodes  = zeros(1,num_exp);

t = 1;
while t < num_exp
    start = randi(num_nodes);
    goal  = randi(num_nodes);
    [path explored] = WeightedAStar(dg,dist,crime,start,goal,0,node_xy);
    if isempty(path)
        continue
    else
        for weight = 1:length(weights)
            [path explored] = WeightedAStar(dg,dist,crime,start,goal,weights(weight),node_xy)
            if weight == 1
                hold off
            end
            if plot_flag
                subplot(2,4,weight);
                plot(xy(1,start),xy(2,start),'r.','Markersize',30);
                hold on
                title(sprintf('%f - %d',weights(weight),length(explored(:,1))));
                plot(xy(1,path),xy(2,path),'c');
                plot(xy(1,goal),xy(2,goal),'g.','Markersize',30);
                colormap('gray');
                scatter3(xy(1,explored(:,1)),xy(2,explored(:,1)),explored(:,2)',5,explored(:,2)');
                subplot(2,4,4 + weight);
                plot(crimeNode(path));
            end
        end
        if plot_flag
            subplot(2,4,4);
            plot(xy(1,start),xy(2,start),'r.');
            hold on
            scatter3(xy(1,explored(:,1)),xy(2,explored(:,1)),crimeNode(explored(:,1)),5,crimeNode(explored(:,1)));
        end
    end
    start_nodes(t) = start;
    goal_nodes(t) = goal;
    t = t + 1;
end
