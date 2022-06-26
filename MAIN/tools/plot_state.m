function plot_state(real, odom, pose_est, mu, sigma, landmarks, timestep, observedLandmarks, z, window, saved_mu, saved_sigma)

    clf;
    hold on
    grid("on")
    h1 = plot(real.x, real.y);
    h2 = plot(odom.x, odom.y);
    h3 = plot(pose_est.x, pose_est.y);
    h4 = plot(landmarks(:,2), landmarks(:,3), 'ks', 'markersize', 10, 'linewidth', 2);
    drawprobellipse(mu(1:3), sigma(1:3,1:3), 0.6, [0 0.75 1]);

%     for i = 1: 106:length(saved_mu)
%         drawprobellipse(saved_mu(1:3,i), saved_sigma{i,1}(1:3,1:3), 0.6, [0 0.75 1]);
%     end

    xlabel("x(m)")
    ylabel("y(m)")
    title("Robot's trajectory (m)")
    for i=1:length(observedLandmarks)
        if observedLandmarks(i)
	        plot(mu(2*i+ 2),mu(2*i+ 3), 'ro', 'markersize', 10, 'linewidth', 2)
   	        drawprobellipse(mu(2*i+ 2:2*i+ 3), sigma(2*i+ 2:2*i+ 3,2*i+ 2:2*i+ 3), 0.6, 'r');
        end
    end
    for i=1:size(z,2)
        if ~isnan(z(1).id)
	        mX = mu(2*z(i).id+2);
	        mY = mu(2*z(i).id+3);
    	    line([mu(1), mX],[mu(2), mY], 'color', 'k', 'linewidth', 1);
        end
    end

    h5 = drawrobot(mu(1:3), 0.3);
    h6 = plot(real.x(1), real.y(1), 'x', 'markersize', 10, 'linewidth', 2);
    legend([h1 h2 h3 h4 h6],{'Real Trajectory','Odometry','Estimated Position','Landmarks','Starting Point'})
    xlim([-5, 5])
    ylim([-6, 4])
%     xlim([-2, 12])
%     ylim([-2, 12])

    hold off

    if window
      figure(1);
      drawnow;
    else
       saveas(gcf,"./imagens/camera/image_"+timestep+".png");
    end
end
