load Received.mat;

time = Received_Data(1,:);
udp_stream = Received_Data(2:120,:);
vicon = Received_Data(121:125,:);

t_begin = 66; t_end = t_begin+5;
i_begin = find(time>=t_begin,1); i_end = find(time>=t_end,1);
scale = i_begin:i_end;

figure; plot(vicon(1,scale),vicon(4,scale));