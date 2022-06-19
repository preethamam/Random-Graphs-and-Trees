function [G,coor] = RandomGeometricGraph(node_num, radius, pos, dim, method, showplot)
    
%%***********************************************************************%
%*                  Random Geometric Graph Generator                    *%
%*                  Generates Random Geometric Graphs                   *%
%*                                                                      *%
%*                                                                      *%
%* Author: Dr. Preetham Manjunatha & Zhenghao Li                        *%
%* Github link: https://github.com/preethamam                           *%
%* Date: 05/15/2022                                                     *%
%************************************************************************%
%
%************************************************************************%
%
% Usage: [G, coor]              = RandomGeometricGraph(node_num, radius, pos, dim, method, showplot)
%        
% Inputs:
%           node_num            - Number of nodes in the graph
%           radius              - Radius that nodes within this radius will have edges connected
%           pos                 - Optional positions for nodes in the graph
%           dim                 - Dimension of the graph
%           method              - The method to build the graph
%                                 'BruteForce' for O(n^2) complexity
%                                 'KDTree' for O(nlogn) complexity
%           showplot            - 1 to plot the graph, 0 to skip
%
% Outputs: 
%
%           G                   - Graph object for the random geometric graph
%           coor                - coordinates of all each point in the graph
%
%--------------------------------------------------------------------------
% Example 1: Generate a 2D geometric graph with 25 nodes using brutal force method
% node_num = 25;      
% radius = 0.5;  
% pos = [];
% dim = 2;       
% method = 'BruteForce';         
% showplot = 1;
% [G,coor] = RandomGeometricGraph(node_num, radius, pos, dim, method, showplot);
%
% Example 2: Generate a 2D geometric graph with 25 nodes using KDTree method
% node_num = 25;      
% radius = 0.5;  
% pos = [];
% dim = 2;       
% method = 'KDTree';         
% showplot = 1;
% [G,coor] = RandomGeometricGraph(node_num, radius, pos, dim, method, showplot);
%
% Example 3: Generate a 3D geometric graph with 25 nodes using KDTree method
% node_num = 25;      
% radius = 0.1;  
% pos = rand(100,3);
% dim = 3;       
% method = 'KDTree';         
% showplot = 1;
% [G,coor] = RandomGeometricGraph(node_num, radius, pos, dim, method, showplot);


%------------------------------------------------------------------------------------------------------------------------
% nargin check
if nargin < 2
    error('Not enough input arguments.');
elseif nargin > 6
    error('Too many input arguments.');
end

if nargin == 2
    %-----------------------
    pos = [];
    dim = 2;
    method = 'KDTree';
    showplot = 0;
end

if nargin == 3
    %-----------------------
    dim = 2;
    method = 'KDTree';
    showplot = 0;
end

if nargin == 4
    %-----------------------
    method = 'KDTree';
    showplot = 0;
end

if nargin == 5
    %-----------------------
    showplot = 0;
end

%------------------------------------------------------------------------------------------------------------------------
% Get the coordinate points
if isempty(pos)
    coor = rand(dim, node_num); % create random coordinates
else
    coor = pos; % use input pos if given
end

% Initialze the source and target points
s = [];
t = [];

% Random geometric graph
switch method
    % Brute force O(n^2)
    case 'BruteForce'
        for i = 1 : node_num
            for j = i : node_num
                dist = sqrt(sum((coor(:,i)-coor(:,j)).^2));
                if dist < radius
                    s = [s i];
                    t = [t j];
                end
            end
        end

    % KDTree Search O(nlogn)
    case 'KDTree'
        MdlKDT = KDTreeSearcher(coor');
        IdxKDT = rangesearch(MdlKDT,coor',radius);
        for i = 1 : node_num
            idxs = IdxKDT{i};
            idxscpy = idxs;
            idxscpy(idxscpy<i) = i;
            s = [s repmat(i, 1, length(idxs))];  
            t = [t idxscpy];
        end
end

% Create a graph object
G = graph(s, t, 'omitselfloops');

% Plot the graph
if (showplot)
    if dim == 2
        figure;
        p = plot(G,'XData',coor(1,:),'YData',coor(2,:));
        p.MarkerSize = 3;
        title('Random Geomtric Graph')
        xlabel('X'); ylabel('Y');
        
    elseif dim == 3
        figure;
        p = plot(G,'XData',coor(1,:),'YData',coor(2,:), 'ZData',coor(3,:));
        p.MarkerSize = 3;
        title('Random Geomtric Graph')
        xlabel('X'); ylabel('Y');
    end
end
end