function [mu, sigma, Seen_Landmarks, pim, indexes] = correction_step_ML(mu, sigma, z)

N = (length(mu)-3)/2;   % Número de landmarks já descobertas

m = size(z, 2);         % Número de landmarks observadas nesta interação

expectedZ = zeros((N+1)*2, 1);      % Qual seria o z observado para cada uma das landmarks já vistas
Z = zeros(m*2, 1);              % Z efetivamente observado
decidedZ = zeros(m*2,1);        % Z que seria observado quando foi identificada qual é efetivamente aquela landmark

seen_ids = zeros(m,1);

indexes = size(m,1);

Q = eye(2) .* 0.1;
Qm = eye(2*m) .* 0.1;

H = [];
pim = [];


for i = 1:m %landmarks vistas

    pik = zeros(N+1, 1);

    mu_provisorio = [mu; zeros(2,1)];
    sigma_provisorio = [sigma zeros(2*N+3, 2);
                        zeros(2, 2*N+3) Q];%sigma(1:2, 1:2)]; 

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

  pik(N+1,1) = 0.3;%3.5e-2;
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
