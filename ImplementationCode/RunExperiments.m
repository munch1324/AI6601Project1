function [ output_args ] = RunExperiments(  )
%RUNEXPERIMENTS Summary of this function goes here
%   Detailed explanation goes here
open osm2matlab.mat
sparceAdjacency = ans.dg;
dist  = ans.dist;
crime = ans.crime;
xy    = ans.parsed_osm.node.xy; 
crimeNode = ans.crime_node;
weights = [0, 0.5, 1];

rng(3,'twister');
for i = 1:1000
    start = randi(39358)
    goal  = randi(39358)
    [path explored] = WeightedAStar(  sparceAdjacency, dist, crime, start, goal, 0.5 )
    if length(path) > 0
        for weight = 1:length(weights)
            [path explored] = WeightedAStar(  sparceAdjacency, dist, crime, start, goal, weights(weight) )
            if weight == 1
                hold off
            end
            subplot(1,4,weight);
            
            hs = plot(xy(1,start),xy(2,start),'r.');
            set(hs, 'Markersize',30);
            hold on
            title(sprintf('%f - %d',weights(weight),length(explored(:,1))));
            plot(xy(1,path),xy(2,path),'c');
            hg = plot(xy(1,goal),xy(2,goal),'g.');
            set(hg, 'Markersize',30);
            colormap('gray');
            scatter3(xy(1,explored(:,1)),xy(2,explored(:,1)),explored(:,2)',5,explored(:,2)');
        end
        subplot(1,4,4);
        hs = plot(xy(1,start),xy(2,start),'r.');
        hold on
        scatter3(xy(1,explored(:,1)),xy(2,explored(:,1)),crimeNode(explored(:,1)),5,crimeNode(explored(:,1)));
        iii = 1;
    end
end

end

