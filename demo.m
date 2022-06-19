%//%************************************************************************%
%//%*                    Random graphs and trees                           *%
%//%*                                                                      *%
%//%*          Authors: Dr. Preetham Manjunatha & Zhenghao Li              *%
%//%*             GitHub: https://github.com/preethamam                    *%
%//%*                                                                      *%
%//%************************************************************************%
%//%*                                                                      *%                             
%//%*             University of Southern california,                       *%
%//%*             Los Angeles, California.                                 *%
%//%************************************************************************%

%% Start parameters
%--------------------------------------------------------------------------
clear; close all; clc;
Start = tic;

%% Inputs
%--------------------------------------------------------------------------
h = 480; w = 640;
numPoints = 60;

%-------------
% Erodos Renyi
p = 0.05;            % probability of connection between the edges
showPlotER = 0;

%-------------
% Random geomtric graph
radius = 75;         % Radius to connect the two vertices 

%-------------
% RRT
startpos    = [1 h]; % randi([1,h],1,2)
endpos      = [w 1]; % randi([1,h],1,2)
boundary    = [1 w 1 h]; % boundary [xmin xmax ymin ymax]
iteration   = 50;        % Number of iterations
stepSize    = 30;        % Step size to move forward
dim         = 2;         % 2D or 3D
obst_num    = 0;         % Include random obstacles (0 to n)
obst_r      = 35;        % Random obstacles radius
showPlot    = 0;         % Display plot (0 or 1)

%-------------
% RRT*
startpos_star    = [1 h]; % randi([1,h],1,2)
endpos_star      = [w 1]; % randi([1,h],1,2)
boundary_star    = [1 w 1 h]; % boundary [xmin xmax ymin ymax]
iteration_star   = 50;        % Number of iterations
stepSize_star    = 30;        % Step size to move forward
radius_star      = 30;        % Search radius
dim_star         = 2;         % 2D or 3D 
obst_num_star    = 0;         % Include random obstacles (0 to n) 
obst_r_star      = 35;        % Random obstacles radius
showPlot_star    = 0;         % Display plot (0 or 1)

%% Pre-processing
%--------------------------------------------------------------------------
% Boudary (rectangular or square)
%--------------------------------------------------------------------------
imX1 = 1; imX2 = w;
imY1 = 1; imY2 = h;
VoronoiBoundary = [imX1 imX2 imY1 imY2]; % [xmin, xmax, ymin, ymax]

%--------------------------------------------------------------------------
% Define X and Y points for random graph
%--------------------------------------------------------------------------
% rng(1)
X = randi([imX1,imX2], 1, numPoints);
Y = randi([imY1,imY2], 1, numPoints);

%% Random graph generation methods

%-------------
% Erodos Renyi
[G1,n,m] = ErdosRenyiRandomgraph(numPoints,p, X, Y, showPlotER);

figure(1);
image = false(h,w);
imshow(image)
hold on
p1 = plot(G1,'XData',X,'YData',Y, 'NodeColor', 'yellow', ...
        'EdgeColor', 'yellow', 'NodeLabelColor', 'yellow', ...
        'LineWidth', 2);
p1.MarkerSize = 5;
hold off

%-------------
% Random geomtric graph
pos = [X; Y];
[G2,coord] = RandomGeometricGraph(numPoints, radius, pos);

figure(2);
image = false(h,w);
imshow(image)
hold on
p2 = plot(G2,'XData',X,'YData',Y, 'NodeColor', 'yellow', ...
        'EdgeColor', 'yellow', 'NodeLabelColor', 'yellow', ...
        'LineWidth', 2);
p2.MarkerSize = 5;
hold off


%-------------
% RRT
G3 = RRT(startpos, endpos, boundary, iteration, stepSize, dim, obst_num, obst_r, showPlot);

X3 = G3.Nodes.XData';
Y3 = G3.Nodes.YData';

figure(3);
image = false(h,w);
imshow(image)
hold on
p3 = plot(G3,'XData',X3,'YData',Y3, 'NodeColor', 'yellow', ...
        'EdgeColor', 'yellow', 'NodeLabelColor', 'yellow', ...
        'LineWidth', 2);
p3.MarkerSize = 5;
hold off


%-------------
% RRT*
G4 = RRTStar(startpos_star, endpos_star, boundary_star, iteration_star, stepSize_star, ...
    radius_star, dim_star, obst_num_star, obst_r_star, showPlot_star);

X4 = G4.Nodes.XData';
Y4 = G4.Nodes.YData';

figure(4);
image = false(h,w);
imshow(image)
hold on
p4 = plot(G4,'XData',X4,'YData',Y4, 'NodeColor', 'yellow', ...
        'EdgeColor', 'yellow', 'NodeLabelColor', 'yellow', ...
        'LineWidth', 2);
p4.MarkerSize = 5;
hold off
        

%% End parameters
%--------------------------------------------------------------------------
Runtime = toc(Start);
disp(Runtime);