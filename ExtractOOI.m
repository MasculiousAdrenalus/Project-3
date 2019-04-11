%%
function r = ExtractOOI(ranges, intensity, mh)
%load('DataForProject02/Laser__2C.mat', 'dataL');
r.N = 0;
r.Centers = [];
r.Sizes   = [];
r.Color = [];

cluster_X =[];
cluster_Y = [];
cluster = [];

angles = [0:360]'*0.5* pi/180 ;         % associated angle, for each individual range in a scan
%angles = dataL.angles;
Xpos = -cos(angles).*ranges;
Ypos = sin(angles).*ranges+0.46;

%find the gaps
edges = [0 (find(sqrt(diff(Xpos).^2+diff(Ypos).^2) > 0.15))'];
r.N = length(edges)-1; %not including zero
%find centers and diameters

for i = 1:r.N
    cluster_X = Xpos(edges(i)+1:edges(i+1));
    cluster_Y = Ypos(edges(i)+1:edges(i+1));
    cluster = [cluster_X cluster_Y];
    temp = [cluster_X(1) cluster_Y(1) ; cluster_X(end) cluster_Y(end)];
    r.Sizes(i) = pdist(temp,'euclidean');%norm(range(cluster_X),range(cluster_Y));
    r.Centers(:,i) = [mean(cluster_X),mean(cluster_Y)];
    if (r.Sizes(i) < 0.05 || r.Sizes(i) > 0.2 )
        r.Sizes(i) = -1;
    end
    temp = [];
    temp = max(intensity(edges(i)+1:edges(i+1)) > 0);
    if (temp) > 0
        r.Color(i) = 1;
    else
        r.Color(i) = 0;
    end
    
end
%gets rid of small objects
% r.Color(find(r.N < 2)) = [];
% r.Centers(:,find(r.N < 2)) = [];
% r.Sizes(:,find(r.N < 2)) = [];
%gets rid of small Diameter objects

r.Color(find(r.Sizes == -1)) = [];
r.Centers(:,find(r.Sizes == -1)) = [];
r.Sizes(find(r.Sizes == -1)) = [];
r.N = length(r.Sizes);
% assignin('base','edges',edges);

return;
end
% ---------------------------------------
