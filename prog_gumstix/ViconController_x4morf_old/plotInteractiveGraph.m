function plotInteractiveGraph(src,event,xyz_vicon,i0,i_f,i_u,i_r,axLim,gap,x4,angles_est,gamma)
   
    gx = gap.position(1); gy = gap.position(2); gw = gap.width;
    L = x4.l_body; l = x4.l_arm; r_p = x4.r_prop;
    psi = angles_est(3,round(src.Value));
    gam = gamma(round(src.Value));
    % Drone position
    p_r = xyz_vicon(:,round(src.Value));
    p_body = [p_r(1)-cosd(psi)*L/2 p_r(1)+cosd(psi)*L/2 p_r(2)-sind(psi)*L/2 p_r(2)+sind(psi)*L/2];
    p_arm1 = [p_body(2)+cosd(psi+gam-90)*l/2 p_body(2)-cosd(psi+gam-90)*l/2 p_body(4)+sind(psi+gam-90)*l/2 p_body(4)-sind(psi+gam-90)*l/2];
    p_arm2 = [p_body(1)+cosd(psi-gam-90)*l/2 p_body(1)-cosd(psi-gam-90)*l/2 p_body(3)+sind(psi-gam-90)*l/2 p_body(3)-sind(psi-gam-90)*l/2];
    p_p1 = [p_arm1(1)-r_p p_arm1(3)-r_p 2*r_p 2*r_p];
    p_p2 = [p_arm1(2)-r_p p_arm1(4)-r_p 2*r_p 2*r_p];
    p_p3 = [p_arm2(1)-r_p p_arm2(3)-r_p 2*r_p 2*r_p];
    p_p4 = [p_arm2(2)-r_p p_arm2(4)-r_p 2*r_p 2*r_p];

subplot(1,2,1);
    hold off;
    % Plot trajectory
   	plot(xyz_vicon(1,i0:round(src.Value)),xyz_vicon(2,i0:round(src.Value)));
    hold on;
    % Plot folding/unfolding/active recovery line
    p = plot([xyz_vicon(1,i_f) xyz_vicon(1,i_f)],[-10 10],'g'); p.Color(4) = 0.4;
    p = plot([xyz_vicon(1,i_u) xyz_vicon(1,i_u)],[-10 10],'g'); p.Color(4) = 0.4;
    p = plot([xyz_vicon(1,i_r) xyz_vicon(1,i_r)],[-10 10],'m'); p.Color(4) = 0.4;
    % Plot wall
    plot([gx gx],[gy+gw/2 10],'r',[gx gx],[gy-gw/2 -10],'r');
    % Plot drone
    plot(p_body(1:2),p_body(3:4),'k');
    plot(p_arm1(1:2),p_arm1(3:4),'g',p_arm2(1:2),p_arm2(3:4),'g');
    rectangle('Position',p_p1,'Curvature',[1 1]);
    rectangle('Position',p_p2,'Curvature',[1 1]);
    rectangle('Position',p_p3,'Curvature',[1 1]);
    rectangle('Position',p_p4,'Curvature',[1 1]);
    % Axes limits
    ax = gca; ax.Position = [0.04 0.2 0.42 0.78]; ax.XLim = [axLim(1) axLim(2)]; ax.YLim = [axLim(3) axLim(4)]; axis equal;

subplot(1,2,2);
    hold off;
    % Plot trajectory
    plot(xyz_vicon(1,i0:round(src.Value)),xyz_vicon(3,i0:round(src.Value)));
    hold on;
    % Plot folding/unfolding/active recovery line
    p = plot([xyz_vicon(1,i_f) xyz_vicon(1,i_f)],[-10 10],'g'); p.Color(4) = 0.4;
    p = plot([xyz_vicon(1,i_u) xyz_vicon(1,i_u)],[-10 10],'g'); p.Color(4) = 0.4;
    p = plot([xyz_vicon(1,i_r) xyz_vicon(1,i_r)],[-10 10],'m'); p.Color(4) = 0.4;
    % Plot wall
    plot([gx gx],[-10 10],'r');
    % Axes limits
    ax = gca; ax.Position = [0.54 0.2 0.42 0.78]; ax.XLim = [axLim(1) axLim(2)]; ax.YLim = [axLim(5) axLim(6)]; axis equal;
    
end