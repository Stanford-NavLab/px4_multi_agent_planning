%% Sweep through a set of trajectories 

clear; clc

% load planning model
load('quadrotor_linear_planning_model.mat');

%% Parameters

v_max = 3.0;

% initial conditions
v_0 = zeros(3,1);
a_0 = zeros(3,1);

% home location
home = [0;0;7];

% trajectory parameters
V_pk = combvec([-v_max,v_max],[-v_max,v_max],[-v_max,v_max]);

% %% Iterate through trajectories
% 
% for i = 1:length(K)
%     v_pk = V_pk(:,i);
% end


%% Analyze 

v_pk = [1; 1; -3];
k = [v_0, a_0, v_pk];
ref = k * LPM.position;
ref = ref + home;

fName = "logs/Trajectory_Log_2021-08-26-17-14-19.txt";
trajs = getLogData(fName);

% Define and load the column indices for easier readability
global timestamp rx ry rz vx vy vz qx qy qz qw r p y ax ay az
loadIndices();

% Call each global variables once so matlab does not squiggle them for being unused...
timestamp;rx;ry;rz;vx;vy;vz;qx;qy;qz;qw;r;p;y;ax;ay;az;

%% Plot positions for each logged trajectory

fig1 = figure('Renderer', 'painters', 'Position',[50 50 1280 720]);
blue = [0 0.4470 0.7410];
orange = [0.8500 0.3250 0.0980];

axis equal
plot3(ref(1,:), ref(2,:), ref(3,:), '--', 'LineWidth', 2, 'Color', blue);
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
    plot(ax1,ref(1,:)' - traj_x);
    % y
    traj_y = resample(double(trajs{i}(:,rx)),N,length(trajs{i}));
    plot(ax2,ref(2,:)' - traj_y);
    % z
    traj_z = resample(double(-trajs{i}(:,rz)),N,length(trajs{i}));
    plot(ax3,ref(3,:)' - traj_z);
end
