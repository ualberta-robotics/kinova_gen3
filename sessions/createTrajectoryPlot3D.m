function createTrajectoryPlot3D(X1, Y1, Z1, title_name)
%CREATEFIGURE(X1, Y1, Z1)
%  X1:  scatter3 x
%  Y1:  scatter3 y
%  Z1:  scatter3 z

%  Auto-generated by MATLAB on 24-Jul-2019 21:37:41

% Create figure
figure1 = figure;
colormap(jet);

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create scatter3
scatter3(X1,Y1,Z1,'MarkerEdgeColor',[0 0 1],'Marker','*','LineWidth',1);

% Create zlabel
zlabel('Z');

% Create ylabel
ylabel('Y');

% Create xlabel
xlabel('X');

% Create title
title({title_name},'HorizontalAlignment','center',...
    'FontWeight','bold');

% Uncomment the following line to preserve the X-limits of the axes
xlim(axes1,[0.0 0.6]);
% Uncomment the following line to preserve the Y-limits of the axes
ylim(axes1,[-0.3 0.3]);
% Uncomment the following line to preserve the Z-limits of the axes
zlim(axes1,[0.0 0.6]);
view(axes1,[63.6999999999998 41.2]);
grid(axes1,'on');