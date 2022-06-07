%% Inicialize

clear
close all;
clc

%% Initialize

%Prediction 1 
N = 12;
X_predicted = zeros(2*N+3, 1);
X_real = zeros(2*N+3, 1);
Fx = zeros(3, 2*N+3); 
G = zeros(2*N+3, 2*N+3); 

X_predicted(1) = 0.1;
X_predicted(2) = 0.1;
X_real(1) = 0.1;
X_real(2) = 0.1;

Fx(1,1) = 1;
Fx(2,2) = 1;
Fx(3,3) = 1;

%Prediction 2
Sigma_predicted = 20*eye(2*N+3, 2*N+3);
Sigma_predicted(1,1) = 0.1;
Sigma_predicted(2,2) = 0.1;
Sigma_predicted(3,3) = 0.1;

R = [0.01^2 0 0;
    0 0.01^2 0;
    0 0 deg2rad(1)^2];

Q = [0.1^2 0;
    0 deg2rad(3)^2];

%Correction 
K = zeros(2*N+3, 2);    

%% Create an environment

landmarks = [0.25 0.15 0;
             0.35 0.7 0;
             0.2 0.75 0; 
             0.35 0.4 0; 
             0.35 0.6 0; 
             0.22 0.18 0;
             0.15 0.65 0;
             0.15 0.58 0;
             0.14 0.50 0;
             0.2 0.47 0;
             0.24 0.42 0;
             0.3 0.3 0];

for a = 1:N
    
  X_predicted(2*(a-1) + 4) = landmarks(a,1);  
  X_predicted(2*(a-1) + 5) = landmarks(a,2); 

end 

delta_d = 0.035;
delta_theta = wrapTo2Pi([0, 0.3, 0.5, 0.6, 0.7, 1, 1.2, 1.3, 1.4, 1.3, 1.3, 1.3, 1.3, 1.2, 1, 1.2, 1.3, 1.5, 1.9, 2.1, 2.4, 2.7, 2.9, 3.1, -2.9, -2.5, -2.2, -2, -1.7, -1.5, -1.3, -1, -0.7, -0.5, -0.2, -0.4, -0.8, -1.1, -1.4, -1.5, -1.5, -1.5, -1.7, -1.7, -1.8]);

% Plot 
predictions_x = zeros(2*N+3, length(delta_theta)); 
predictions_x(:, 1) = X_predicted;
reals_x = zeros(2*N+3, length(delta_theta));
reals_x(:, 1) = X_real;
predicted_sem_landmarks = zeros(2*N+3, length(delta_theta));
predicted_sem_landmarks(:, 1) = X_predicted;

for i = 1:length(delta_theta)-1

     g_motion = [delta_d * cos(delta_theta(i));
                 delta_d * sin(delta_theta(i));
                 delta_theta(i+1) - delta_theta(i)];

      G_motion = [0 0 -delta_d*sin(delta_theta(i));
                  0 0 delta_d*cos(delta_theta(i));
                  0 0 0];

      X_real = X_real + Fx'*g_motion;
      X_real(3) = wrapTo2Pi(X_real(3));
      X_predicted = X_predicted + Fx'*g_motion + wgn(2*N+3, 1, -48);
      X_predicted(3) = wrapTo2Pi(X_predicted(3));
      X_predicted_sem_landmarks = X_predicted;

      G = eye(2*N+3) + Fx'*G_motion*Fx;

      reals_x(:, i+1) = X_real;
      predicted_sem_landmarks(:, i+1) = X_predicted_sem_landmarks;
   
      Sigma_predicted = G*Sigma_predicted*G' + Fx'*R*Fx;
        
      for a = 1:length(landmarks(:, 1))
               
          range = sqrt((X_real(1) - landmarks(a, 1))^2 + (X_real(2) - landmarks(a, 2))^2);

          if range < 0.2
           
            phi = wrapTo2Pi(atan2(X_real(2) - landmarks(a, 2), X_real(1) - landmarks(a, 1)) - X_real(3));
            %aux_phi = phi / abs(phi);   
            %phi = mod(phi, aux_phi*2*pi)
            

            if abs (phi) < 75 * pi/180  
                 
               landmarks(a, 3) = landmarks(a, 3) + 1;

                z = [range;   %%%%%%%%%%
                     phi];

                dx = X_predicted(1) - landmarks(a, 1);
                dy = X_predicted(2) - landmarks(a, 2);
                range_estimated = sqrt((dx)^2 + (dy)^2);

                phi_estimated = wrapTo2Pi(atan2(dy, dx) - X_predicted(3));
                %aux_phi_estimated = phi_estimated / abs(phi_estimated);
                %phi_estimated = mod(phi_estimated, aux_phi_estimated*2*pi)

                z_estimated = [range_estimated;
                               phi_estimated];
            
                F = zeros(5, 2*N+3);
                F(1,1) = 1;
                F(2,2) = 1;
                F(3,3) = 1;
                F(4, 2*(a-1) + 4) = 1;
                F(5, 2*(a-1) + 5) = 1;
                
                H = 1/range_estimated^2 * [range_estimated*dx -range_estimated*dy 0 -range_estimated*dx range_estimated*dy;
                    dy dx -1 -dy -dx] * F;
        
                K = Sigma_predicted*H'*inv(H*Sigma_predicted*H'+Q);
    
                X_predicted = X_predicted + K*(z - z_estimated);
                X_predicted(3) = wrapTo2Pi(X_predicted(3));
 
                Sigma_predicted = (eye(2*N+3) - K*H)*Sigma_predicted;

            end    

          end    

      end     

      predictions_x(:, i+1) = X_predicted;
      
end  


figure()
rectangle('Position',[0.1 0.1 0.35 0.65])
axis([0 1.2 0 0.8])
hold on
plot(predictions_x(1,:),predictions_x(2,:))
plot(reals_x(1,:),reals_x(2,:))
plot(predicted_sem_landmarks(1,:),predicted_sem_landmarks(2,:))
plot(predictions_x(4,:), predictions_x(5,:))
scatter(landmarks(:,1), landmarks(:,2))
legend('Trajectory', 'Real', 'Trajectory_no_landmarks')
xlabel("X")
ylabel("Y")
title("Generated trajectory of the robot")

% figure()
% rectangle('Position',[0.1 0.1 0.35 0.65])
% axis([0 1.2 0 0.8])
% hold on
% plot(1:length(delta_theta),predictions_x(3,:))
% plot(1:length(delta_theta),reals_x(3,:))
% plot(1:length(delta_theta),predicted_sem_landmarks(3,:))
% scatter(landmarks(:,1), landmarks(:,2))
% legend('Trajectory', 'Real', 'Trajectory_no_landmarks')
% xlabel("X")
% ylabel("Y")
% title("Generated trajectory of the robot")
