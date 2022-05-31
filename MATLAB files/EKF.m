%% Inicialize
clear
close all;
clc

%% Import Data
load("data.mat"); %Get Rosbag Information
%A = dlmread('landmarks.txt'); % Get Landmark Information

%% Inicialize algorith

N = 10;                                 % Número de Landmarks
X_predicted = zeros(2*N+3, 1);          % Vetor de Estado
Sigma = 99E99 * eye(2*N+3, 2*N+3);      % Matriz das Covariâncias
Fx = zeros(3, 2*N+3);                   % Matriz Desnecessária
G = zeros(2*N+3, 2*N+3);                % Jaconiano da função de Movimento
K = zeros(2*N+3, 2);                    % Kalman Gain

Q = [5^2 0;
    0 5^2];                             % Covariância sensores landmarks;

R = [5^2 0 0;
    0 5^2 0;
    0 0 5^2];                           % Covariância sensores odometria

Sigma(1:3, :) = 0;

Fx(1,1) = 1;
Fx(2,2) = 1;
Fx(3,3) = 1;

X_predicted(1) = Odometria(1,2);
X_predicted(2) = Odometria(1,3);
X_predicted(3) = Odometria(1,4);

predictions_x = zeros(2*N+3, length(Odometria));                  
Kalman_gains = zeros(2*N+3, 2, length(Odometria));

%% EKF

for i = 2:length(xPoints)

    delta_d = sqrt((xPoints(i) - xPoints(i-1))^2 + (yPoints(i) - yPoints(i-1))^2);
    delta_theta = atan2(yPoints(i) - yPoints(i-1), xPoints(i) - xPoints(i-1));
    dif_theta = theta(i) - theta(i-1);

    % Predict
    
    g_motion = [delta_d * cos(delta_theta);
                delta_d * sin(delta_theta);
                theta(i-1) + dif_theta];

    G_motion = [0 0 -delta_d*sin(delta_theta);
                0 0 delta_d*cos(delta_theta);
                0 0 0];
    % Definir antes de atualizar X_predicted para garantir que temos Xt e
    % não Xt+1

    X_predicted = X_predicted + Fx'*g_motion;
    % Supostamente X_predicted = g(x_passado, input); - Logo está ERRADO!

    G = eye(2*N+3) + Fx'*G_motion*Fx;
    Sigma_predicted = G*Sigma*G' + Fx'*R*Fx;


    for j = 1:N_vistos         % Para todos os landmarks que foram vistos
    
        if j == 5               % Tirar isto, só para não entrar nesta parte do código
            id_seen = 5;         % tirar o id do ROS Ter atenção ao id=0;
            
            if Sigma(3*id_seen + 3) == 99E99 && Sigma(3*id_seen + 4) == 99E99   % Caso nunca tenha visto o landmark
    
                X_predicted(3*id_seen + 3) = X_predicted(1) + d_observation * cos(phi_observation + X_predicted(3));
                X_predicted(3*id_seen + 4) = X_predicted(2) + d_observation * sin(phi_observation + X_predicted(3)); 
                % Inicializou-as com as coordenadas do robot + trig do aruco.
        
            end
    
            delta = [X_predicted(3*id_seen + 3) - X_predicted(1);
                    X_predicted(3*id_seen + 4) - X_predicted(2)];
    
            q = delta'*delta;
            
            z_estimated = [sqrt(q);
                          atan2(delta(2), delta(1))];
            
            F = zeros(5, 2*N+3);
            F(1,1) = 1;
            F(2,2) = 1;
            F(3,3) = 1;
            F(4, 2*id_seen + 3) = 1;
            F(4, 2*id_seen + 4) = 1;
    
            H = 1/q * [sqrt(q)*delta(1) -sqrt(q)*delta(2) 0 -sqrt(q)*delta(1) sqrt(q)*delta(2)
                delta(2) delta(1) -1 -delta(2) -delta(1)] * F;
    
            K = Sigma_predicted*H'*inv(H*Sigma_predicted*H'+Q);
    
            X_predicted = X_predicted + K*( z - H*X_predicted );    
     
            Sigma_predicted = (eye(2*N+3) - K*H)*Sigma_predicted;
        end
    end


    %Outputs
    predictions_x(:, i) = X_predicted;                  
    Kalman_gains(:,:, i) = K;
end
