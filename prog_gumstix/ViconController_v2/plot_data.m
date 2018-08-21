%load Received.mat;

n_data_udp_recv = 130;
n_data_udp_sent = 133;

N = length(Received_Data(1,:));

time = Received_Data(1,:);
udp_recv = Received_Data(2:n_data_udp_recv+1,:);
vicon = Received_Data(n_data_udp_recv+2:n_data_udp_recv+11,:);
udp_sent = Received_Data(n_data_udp_recv+22:n_data_udp_recv+22+n_data_udp_sent-1,:);
start = Received_Data(n_data_udp_recv+22+n_data_udp_sent,:);

odroid = udp_recv(120:120+8,:);

%% Time and scale (select only last scenario)
d_start = [0 diff(start)];
i_start = find(d_start>0,1,'last'); i_stop = find(d_start<0,1,'last');
if (i_stop <= i_start) i_stop = N; end
scale = i_start:i_stop;

%% Physical vars
t = time(scale)-time(i_start);
x = vicon(1,scale);
y = vicon(2,scale);
z = vicon(3,scale);
V = vicon(4:6,scale);
ttc = odroid(9,scale);

%% Filtering
n_ttc_fil = 50;
coeff_avg_fil = ones(1,n_ttc_fil)/n_ttc_fil;
ttc_fil = filter(coeff_avg_fil,1,ttc);

%% Plot
figure; hold on; plot(t,ttc); plot(t,ttc_fil,'r');