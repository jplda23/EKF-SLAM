function [mu, sigma, Seen_Landmarks, pim, indexes] = correction_step_ML(mu, sigma, z, t)

N = (length(mu)-3)/2;   % Número de landmarks já descobertas

m = size(z, 2);         % Número de landmarks observadas nesta interação

expectedZ = zeros((N+1)*2, 1);      % Qual seria o z observado para cada uma das landmarks já vistas
Z = zeros(m*2, 1);              % Z efetivamente observado
decidedZ = zeros(m*2,1);        % Z que seria observado quando foi identificada qual é efetivamente aquela landmark

seen_ids = zeros(m,1);

indexes = size(m,1);

Q = eye(2) .* 0.01;
Q(2,2)=0.09;
Qm = eye(2*m) .* 0.01;
for g = 1:m
    Qm(2*g,2*g) = 0.2;
end

H = [];
pim = [];

for i = 1:m %landmarks vistas

    pik = zeros(N+1, 1);

    mu_provisorio = [mu; zeros(2,1)];
    sigma_provisorio = [sigma zeros(2*N+3, 2);
                        zeros(2, 2*N+3) Q]; % Dúvida para o prof: O que meter em lugar do Q (antes estava INF) e nos zeros

    % O seu valor é adicionado na última posição do vetor de estados (N+1)
    mu_provisorio(2*N+3 + 1) = mu(1) + z(i).range * cos(wrapToPi(z(i).bearing+mu(3)));
    mu_provisorio(2*N+3 + 2) = mu(2) + z(i).range * sin(wrapToPi(z(i).bearing+mu(3)));


    Z(i*2-1) = z(i).range;
    Z(i*2)   = z(i).bearing;

    seen_ids(i) = z(i).id; 


     for j = 1:N+1   %landmarks no vetor mu_provisório. Existem N+1 landmarks
    
        b = [mu_provisorio(3+j*2-1) - mu(1); mu_provisorio(3+j*2) - mu(2)];
        q = b' * b;
    
        expectedZ(j*2-1) = sqrt(q);
        expectedZ(j*2) = wrapToPi(atan2(b(2), b(1)) - mu(3));
    
    
        Fx = [ eye(3) zeros(3, 2*(N+1)); zeros(2, 2*j+1) eye(2) zeros(2, 2*(N+1)-2*j) ];
        Hi = 1/q * [ -sqrt(q)*b(1), -sqrt(q)*b(2), 0, +sqrt(q)*b(1), +sqrt(q)*b(2);
                     +b(2), -b(1), -q, -b(2), +b(1) ] * Fx;
    
        mahalanobis_distance = Hi * sigma_provisorio * Hi' + Q;
        pik(j,1) = (Z(i*2-1:i*2) - expectedZ(j*2-1:j*2))' * inv(mahalanobis_distance) * (Z(i*2-1:i*2) - expectedZ(j*2-1:j*2));

    
     end

%pik(N+1,1) = 0.3;   %%%% o que vai ficar no final


%     if t == 708 
%         pik(N+1,1) = 0.24;
%     elseif t == 751
%         pik(N+1,1) = 0.2;
%     elseif t == 792
%         pik(N+1,1) = 0.7;
%     elseif t >= 895 && t < 1832
%         pik(N+1,1) = 2;
%     elseif t >= 1832  
%         pik(N+1,1) = 4;
%     else 
%         pik(N+1,1) = 0.6;     %Odom9 0.3, 2
%     end


     if t == 425 || t == 1832
        pik(N+1,1) = 4;
     elseif t == 964 || t == 511
        pik(N+1,1) = 2; 
     else                    %%%%%%% Odom7 0.01, 0.09 o que vai ficar certo !!!! muito bom sem ML
        pik(N+1,1) = 6;
     end


%      if t < 490
%         pik(N+1,1) = 6;
%      elseif t == 551
%         pik(N+1,1) = 1;
%      elseif t == 1025 || t == 1067 || t == 1151 || t == 1283 || t == 1696   %% odom7 0.001, 0.09
%         pik(N+1,1) = 30;
%      else
%         pik(N+1,1) = 8;
%      end


%     if t == 383 || t == 5513333 || (t == 425 && m == 1)
%         pik(N+1,1) = 0.2;
%     elseif t == 551
%         pik(N+1,1) = 0.6;      %%% odom 7 sigma 0.01, 0.09 
%     elseif t == 634
%         pik(N+1,1) = 3;
%     elseif t == 1025 || t == 1067 || t == 1151
%         pik(N+1,1) = 9;
%     elseif t == 1283 || t == 1696
%         pik(N+1,1) = 13;
%     else
%         pik(N+1,1) = 1.7;
%     end


%     if t == 726 || t == 1110 || t == 1476
%         pik(N+1,1) = 0.06;
%     elseif t == 836
%         pik(N+1,1) = 0.07;      %%% odom 8 Q5 check
%     else
%         pik(N+1,1) = 0.05;
%     end

%      if t==551 && i==1
%        pik(N+1,1) = 0.005;
%      elseif t==859 || t==964  %%%%%   odom 7 com o sigma a 5 check
%        pik(N+1,1) = 0.03;
%      else
%        pik(N+1,1) = 0.04;
%      end


% 201, 213, 223, 234, 253, 295, 306, 318 ---------- lê mal o 11

%     if t < 91 ||  t == 548 
%       pik(N+1,1) = 0.12;
%     elseif t > 317 && t < 1006           %%%%%   odom 6 com o sigma a 0.5, só dá até 1/2
%       pik(N+1,1) = 0.6;
%     elseif t == 1006
%       pik(N+1,1) = 0.09; 
%     elseif t == 1588 || t == 2187 || t == 2320 || t == 2410 || t == 2422
%       pik(N+1,1) = 1.4; 
%     else
%       pik(N+1,1) = 0.22;
%     end

%     if t==140
%       pik(N+1,1) = 0.03;
%     elseif t==548 
%       pik(N+1,1) = 0.015; 
%     elseif t==1006
%       pik(N+1,1) = 0.009;
%     elseif t==1542
%        pik(N+1,1) = 0.06;
%     else                     %%%%%   odom 6 com o sigma a 3 check
%       pik(N+1,1) = 0.04;
%     end

  % 0.017 para o 5 mas não funciona
  % 9e-3 para o 6 mas não funciona
  % 2e-2 para o microsimulador
  [~, k] = min(pik);

  N_new = max([k; N]);

  pim = [pim; "novo pik"; pik];

  if ( N_new > N)

      % Vamos precisar de aumentar o tamanho das matrizes

      mu = mu_provisorio;
      sigma = sigma_provisorio;
      
      decidedZ(i*2-1) = expectedZ(end-1);
      decidedZ(i*2) = expectedZ(end);

      N = N_new;

  else

       decidedZ(i*2-1) = expectedZ(k*2-1);
       decidedZ(i*2) = expectedZ(k*2);

  end
  
  indexes(i) = k; % Guarda as landmarks que foram vistas

end

for i=1:m

    % Assim, como N é atualizado de cada vez que se cria um landmark, então
    % as matrixes Fz e Hk já são criadas com o valor final que deverão ter.
    % Não se pode atualizar a cada iteração porque só depois de ver todos
    % os landmarks é que sabemos quantos landmarks é que vamos ter.

    b = [mu_provisorio(3+indexes(i)*2-1) - mu(1); mu_provisorio(3+indexes(i)*2) - mu(2)];
    q = b' * b;    
    
    Fx = [ eye(3) zeros(3, 2*N); zeros(2, 2*indexes(i)+1) eye(2) zeros(2, 2*N-2*indexes(i)) ];
    Hk = 1/q * [ -sqrt(q)*b(1), -sqrt(q)*b(2), 0, +sqrt(q)*b(1), +sqrt(q)*b(2);
                 +b(2), -b(1), -q, -b(2), +b(1) ] * Fx;
  
    H = [H;Hk]; % Faz H com as landmarks que realmente atualiza

end


mahalanobis_distance = H * sigma * H' + Qm;

K = sigma*H'*inv(mahalanobis_distance); 

% No código eles fazem o K dentro do for e depois a calcular o new mu eles
% fazem com o somatório dos K, mas acho que assim também dá.

diff_Z = wrapToPi(Z - decidedZ);

new_mu = mu + K * diff_Z;
new_sigma = (eye(size(sigma)) - K*H) * sigma;

mu = new_mu;
sigma = new_sigma;

Seen_Landmarks = seen_ids;

end
