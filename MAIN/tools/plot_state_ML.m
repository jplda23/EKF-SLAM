function plot_state_ML(real, odom, pose_est, mu, sigma, landmarks, timestep, N_ML, z, window)

    clf;
    hold on
    grid("on")
    h1 = plot(real.x, real.y);
    h2 = plot(odom.x, odom.y);
    h3 = plot(pose_est.x, pose_est.y);
    h4 = plot(landmarks(:,2), landmarks(:,3), 'ks', 'markersize', 10, 'linewidth', 2);
    legend("Real Trajectory","Odometry","Estimated Position","Landmarks")
    drawprobellipse(mu(1:3), sigma(1:3,1:3), 0.6, [0 0.75 1]);
    xlabel("x(m)")
    ylabel("y(m)")
    
    for i=1:N_ML
        plot(mu(2*i+ 2),mu(2*i+ 3), 'ro', 'markersize', 10, 'linewidth', 2)
        drawprobellipse(mu(2*i+ 2:2*i+ 3), sigma(2*i+ 2:2*i+ 3,2*i+ 2:2*i+ 3), 0.6, 'r');
    end

%     for i=1:size(z,2)
% 	    mX = mu(2*z(i).id+2);
% 	    mY = mu(2*z(i).id+3);
%     	line([mu(1), mX],[mu(2), mY], 'color', 'k', 'linewidth', 1);
%     end

    drawrobot(mu(1:3), 0.3);
    legend([h1 h2 h3 h4],{'Real Trajectory','Odometry','Estimated Position','Landmarks'})
%     xlim([-2, 12])
%     ylim([-2, 12])
    hold off

    if window
      figure(1);
      drawnow;
      %pause(0.1);
    else
       saveas(gcf,"./imagens/camera/image_"+timestep+".png");
    end
end