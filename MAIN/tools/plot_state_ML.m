function plot_state_ML(real, odom, pose_est, mu, sigma, landmarks, timestep, observedLandmarks, z, window)

    clf;
    hold on
    grid("on")
    drawprobellipse(mu(1:3), sigma(1:3,1:3), 0.6, [0 0.75 1]);
    plot(real.x, real.y);
    plot(odom.x, odom.y);
    plot(pose_est.x, pose_est.y);
    plot(landmarks(:,2), landmarks(:,3), 'ks', 'markersize', 10, 'linewidth', 2);
%     for i=1:length(observedLandmarks)
% 	    if observedLandmarks(i)
% 	        plot(mu(2*i+ 2),mu(2*i+ 3), 'bo', 'markersize', 10, 'linewidth', 5)
%    	        drawprobellipse(mu(2*i+ 2:2*i+ 3), sigma(2*i+ 2:2*i+ 3,2*i+ 2:2*i+ 3), 0.6, 'b');
%         end
%     end

%     for i=1:size(z,2)
% 	    mX = mu(2*z(i).id+2);
% 	    mY = mu(2*z(i).id+3);
%     	line([mu(1), mX],[mu(2), mY], 'color', 'k', 'linewidth', 1);
%     end

    drawrobot(mu(1:3), 0.3);
%     xlim([-4, 4.5])
%     ylim([-4, 2.5])
    hold off

    if window
      figure(1);
      drawnow;
      %pause(0.1);
    else
       saveas(gcf,"./imagens/camera/image_"+timestep+".png");
    end
end