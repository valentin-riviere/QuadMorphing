clear all; close all;

RobotParameters;
load Received.mat; % Experiments data

time = Received_Data(1,:);
Ts = time(2)-time(1);
udp_stream = Received_Data(2:113,:);

angles_est = 180/pi*udp_stream(3:5,:);
angles_ref = 180/pi*udp_stream(12:14,:);
torques_dist = udp_stream(47:51,:);

torquesMag = torques_dist(1:3,:);
dt = torques_dist(4,:);
mode = torques_dist(5,:);

i_scale = find(mode==1);    % Windows cut

%% Plot
figure;
angles_dist = zeros(3,length(i_scale));
for i = 1:3
    angles_dist(i,:) = 180/pi*cumtrapz(cumtrapz(torquesMag(i,i_scale)/I(i,i)))*Ts^2;
    i_stop = find(torquesMag(i,i_scale)~=0,1,'last'); angles_dist(i,i_stop:end) = angles_dist(i,i_stop);   % Last step for integration
    subplot(3,1,i); hold on; 
    plot(time(i_scale)-time(i_scale(1)),angles_dist(i,:));
    plot(time(i_scale)-time(i_scale(1)),angles_est(i,i_scale),'r');
end

figure;
for i = 1:3
    subplot(3,1,i); plot(time(i_scale)-time(i_scale(1)),torquesMag(i,i_scale));
end