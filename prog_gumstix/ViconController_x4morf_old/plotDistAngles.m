clear all; close all;

RobotParameters;
load Received_AnglesDist.mat; % Experiments data

time = Received_Data(1,:);
Ts = time(2)-time(1);
udp_stream = Received_Data(2:113,:);

angles_est = 180/pi*udp_stream(3:5,:);
angles_ref = 180/pi*udp_stream(12:14,:);
angles_dist = udp_stream(47:51,:);

AnglesMag = 180/pi*angles_dist(1:3,:);
dt = angles_dist(4,:);
mode = angles_dist(5,:);

i_scale = find(mode==4);    % Windows cut

%% Plot
figure;
for i = 1:3
    subplot(3,1,i); hold on; 
    plot(time(i_scale)-time(i_scale(1)),AnglesMag(i,i_scale));
    plot(time(i_scale)-time(i_scale(1)),angles_est(i,i_scale),'r');
end