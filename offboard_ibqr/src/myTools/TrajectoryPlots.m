%% Plots and analysis for iBQR trajectories based on PX4-ROS2 offboard node
% Author Marco Nunez - maez@alumni.stanford.edu

close all
clear
clc

%% Load the trajectory data
% Each cell contains the vehicle's odometry information for a single trajectory
% Within each cell, the data is arranged as follows:
    % columns:
        % col 1 = timestamp:    microseconds since system start
        % col 2 = rx:           x-position in NED frame
        % col 3 = ry:           y-position in NED frame
        % col 4 = rz:           z-position in NED frame
        % col 5 = vx:           x-velocity in NED frame
        % col 6 = vx:           y-velocity in NED frame
        % col 7 = vx:           z-velocity in NED frame
        % col 8 = qx:           x-quaternion rotation from FRD body frame to NED frame
        % col 9 = qy:           y-quaternion rotation from FRD body frame to NED frame
        % col 10 = qz:          z-quaternion rotation from FRD body frame to NED frame
        % col 11 = qw:          w-quaternion rotation from FRD body frame to NED frame
        % col 12 = r:           roll speed in body fixed frame
        % col 13 = p:           pitch speed in body fixed frame
        % col 14 = y:           yaw speed in body fixed frame
    % rows:
        % number of data points collected
        
% position only
% fName = "logs/Trajectory_Log_2021-08-04-21:09:30.txt";

% position and velocity
% fName = "logs/Trajectory_Log_2021-08-04-21:13:13.txt";
% fName = "logs/Trajectory_Log_2021-08-04-21:19:39.txt";

% position, velocity, and acceleration
fName = "logs/Trajectory_Log_2021-10-13-12-08-29.txt";

% acceleration only
% fName = "logs/Trajectory_Log_2021-08-04-20:59:17.txt";

trajs = getLogData(fName);

% Load reference trajectory
ref = importdata("logs/Reference_Trajectory.txt").data(:,:);

% Define and load the column indices for easier readability
global timestamp rx ry rz vx vy vz qx qy qz qw r p y ax ay az
loadIndices();

% Call each global variables once so matlab does not squiggle them for being unused...
timestamp;rx;ry;rz;vx;vy;vz;qx;qy;qz;qw;r;p;y;ax;ay;az;

%% Plot positions for each logged trajectory

fig1 = figure('Renderer', 'painters', 'Position',[50 50 1280 720]);
blue = [0 0.4470 0.7410];
orange = [0.8500 0.3250 0.0980];

plot3(ref(:,rx), ref(:,ry), ref(:,rz), '--', 'LineWidth', 2, 'Color', blue);
hold on

for i=1:length(trajs)
    % Notice that we are transforming back to ENU coordinates for these plots
    plot3(trajs{i}(:,ry), trajs{i}(:,rx), -trajs{i}(:,rz), 'LineWidth', 2, 'Color', orange);
end

title('iBQR Trajectories from Offboard Node','FontSize',20,'FontWeight','b')

xlabel('x-dir [m]','FontSize',20,'FontWeight','b') % x-axis label %
ylabel('y-dir [m]','FontSize',20,'FontWeight','b') % y-axis label %
zlabel('z-dir [m]','FontSize',20,'FontWeight','b') % z-axis label %
legend('Reference', 'Actual (Sim)')

set(gcf,'Color','w') % Set background color to white %
set(gca,'LineWidth',2,'FontSize',16,'FontWeight','bold') % axis properties
set(gca,'Color','none')
grid on


%% Plot x-y-z tracking errors

figure(2); hold on
N = length(ref);

tiledlayout(3,1)
ax1 = nexttile; ylabel(ax1,'x error');
ax2 = nexttile; ylabel(ax2,'y error');
ax3 = nexttile; ylabel(ax3,'z error');
hold([ax1 ax2 ax3],'on')
for i=1:length(trajs)
    % x
    traj_x = resample(double(trajs{i}(:,ry)),N,length(trajs{i}));
    plot(ax1,ref(:,rx) - traj_x);
    % y
    traj_y = resample(double(trajs{i}(:,rx)),N,length(trajs{i}));
    plot(ax2,ref(:,ry) - traj_y);
    % z
    traj_z = resample(double(-trajs{i}(:,rz)),N,length(trajs{i}));
    plot(ax3,ref(:,rz) - traj_z);
end




