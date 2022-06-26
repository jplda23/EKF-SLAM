function [mu, sigma, Seen_Landmarks, pim, indexes, landmarks_list, sigma_list, counters] = correction_step_ML_with_list(mu, sigma, z, landmarks_list, sigma_list, counters)

N = (length(mu)-3)/2;   % Número de landmarks já descobertas

N_prov = (length(landmarks_list))/2;

m = size(z, 2);         % Número de landmarks observadas nesta interação

expectedZ = zeros((N+1)*2, 1);  % Qual seria o z observado para cada uma das landmarks já vistas
Z = [];              % Z efetivamente observado
decidedZ = [];        % Z que seria observado quando foi identificada qual é efetivamente aquela landmark

seen_ids = zeros(m,1);

Z_1 = zeros(2*m,1);

indexes = [];
indexes_prov = [];

Q = eye(2) .* 0.1;


decidedZ_prov = [];
Z_prov = [];

H = [];
H_prov = [];
pim = [];

atualizacao_lista = false;

expectedZ_prov = zeros((N_prov+1)*2, 1);


for i = 1:m %landmarks vistas

    pik = zeros(N+1, 1);

    mu_provisorio = [mu; zeros(2,1)];
    sigma_provisorio = [sigma zeros(2*N+3, 2);
                        zeros(2, 2*N+3) sigma(1:2, 1:2)]; 

    % O seu valor é adicionado na última posição do vetor de estados (N+1)
    mu_provisorio(2*N+3 + 1) = mu(1) + z(i).range * cos(wrapToPi(z(i).bearing+mu(3)));
    mu_provisorio(2*N+3 + 2) = mu(2) + z(i).range * sin(wrapToPi(z(i).bearing+mu(3)));


    Z_1(i*2-1,1) = z(i).range;
    Z_1(i*2,1)   = z(i).bearing;

    seen_ids(i) = z(i).id; 


  for j = 1:N+1   %landmarks no vetor mu_provisório. Existem N+1 landmarks

    b = [mu_provisorio(3+j*2-1) - mu(1); mu_provisorio(3+j*2) - mu(2)];
    q = b' * b;

    expectedZ(j*2-1,1) = sqrt(q);
    expectedZ(j*2,1) = wrapToPi(atan2(b(2), b(1)) - mu(3));


    Fx = [ eye(3) zeros(3, 2*(N+1)); zeros(2, 2*j+1) eye(2) zeros(2, 2*(N+1)-2*j) ];
    Hi = 1/q * [ -sqrt(q)*b(1), -sqrt(q)*b(2), 0, +sqrt(q)*b(1), +sqrt(q)*b(2);
                 +b(2), -b(1), -q, -b(2), +b(1) ] * Fx;

    mahalanobis_distance = Hi * sigma_provisorio * Hi' + Q;
    pik(j,1) = (Z_1(i*2-1:i*2) - expectedZ(j*2-1:j*2))' * inv(mahalanobis_distance) * (Z_1(i*2-1:i*2) - expectedZ(j*2-1:j*2));

    
  end

  pik(N+1,1) = 3.5e-4;
  [~, k] = min(pik);

  N_new = max([k; N]);

  pim = [pim; "novo pik"; pik];

  if ( N_new > N)

      % Se não conseguimos associar uma landmark, procuramos no vetor dos
      % provisórios.

           for l=1:N_prov

               bp = [landmarks_list(l*2-1) - mu(1); landmarks_list(l*2) - mu(2)];
               qp = bp' * bp;

               expectedZ_prov(l*2-1) = sqrt(qp);
               expectedZ_prov(l*2) = wrapToPi(atan2(bp(2), bp(1)) - mu(3));

               Fxp = [ eye(3) zeros(3, 2*(N_prov)); zeros(2, 2*l+1) eye(2) zeros(2, 2*(N_prov)-2*l) ];
               Hip = 1/qp * [ -sqrt(qp)*bp(1), -sqrt(qp)*bp(2), 0, +sqrt(qp)*bp(1), +sqrt(qp)*bp(2);
                 +bp(2), -bp(1), -qp, -bp(2), +bp(1) ] * Fxp;
           
               mahalanobis_distance_prov = Hip * sigma_list * Hip' + Q;
               pik_prov(l,1) = (Z_1(i*2-1:i*2) - expectedZ_prov(l*2-1:l*2))' * inv(mahalanobis_distance_prov) * (Z_1(i*2-1:i*2) - expectedZ_prov(l*2-1:l*2));
           end
               
           pik_prov(N_prov+1,1) = 3.5e-1;
           [~, k_prov] = min(pik_prov);
        
           N_new_prov = max([k_prov; N_prov]);
        
           if (N_new_prov > N_prov)

               landmarks_list = [landmarks_list;
                                 mu_provisorio(2*N+3 + 1);
                                 mu_provisorio(2*N+3 + 2)];
               counters = [counters; 1; 1];

               sigma_list = [sigma_list zeros(2*N_prov +3, 2);
                            zeros(2, 2*N_prov + 3) sigma(1:2, 1:2)];

               N_prov = N_new_prov;

           else

                decidedZ_prov = [decidedZ_prov;
                                expectedZ_prov(k_prov*2-1);
                                expectedZ_prov(k_prov*2)];


                Z_prov = [Z_prov;
                          z(i).range;
                          z(i).bearing];
                
                counters(2*k_prov-1) = counters(2*k_prov-1) + 1;

                atualizacao_lista = true;
                indexes_prov(i) = k_prov;

           end
         

%       mu = mu_provisorio;
%       sigma = sigma_provisorio;
%       
%       decidedZ(i*2-1) = expectedZ(end-1);
%       decidedZ(i*2) = expectedZ(end);
% 
%       N = N_new;

  else


       decidedZ = [decidedZ;
                   expectedZ(k*2-1);
                   expectedZ(k*2)];

       Z = [Z;
            z(i).range;
            z(i).bearing];

       indexes = [indexes; k]; % Guarda as landmarks que foram vistas

  end
  

end

if(~isempty(indexes))

    for i=1:length(indexes)

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

end

indexes_prov(indexes_prov == 0) = []; 

for i = 1:length(indexes_prov)

    bp = [landmarks_list(indexes_prov(i)*2-1) - mu(1); landmarks_list(indexes_prov(i)*2) - mu(2)];
    qp = bp' * bp;

    expectedZ_prov(i*2-1,1) = sqrt(qp);
    expectedZ_prov(i*2,1) = wrapToPi(atan2(bp(2), bp(1)) - mu(3));

    Fxp = [ eye(3) zeros(3, 2*(N_prov)); zeros(2, 2*indexes_prov(i)+1) eye(2) zeros(2, 2*(N_prov)-2*indexes_prov(i)) ];
    Hip = 1/qp * [ -sqrt(qp)*bp(1), -sqrt(qp)*bp(2), 0, +sqrt(qp)*bp(1), +sqrt(qp)*bp(2);
                 +bp(2), -bp(1), -qp, -bp(2), +bp(1) ] * Fxp;
    
    H_prov = [H_prov; Hip];
end

if(~isempty(indexes))

    Qm = eye(2*length(indexes)) .* 0.1;

    mahalanobis_distance = H * sigma * H' + Qm;
    
    K = sigma*H'*inv(mahalanobis_distance); 
    
    % No código eles fazem o K dentro do for e depois a calcular o new mu eles
    % fazem com o somatório dos K, mas acho que assim também dá.
    
    diff_Z = wrapToPi(Z - decidedZ);
    
    new_mu = mu + K * diff_Z;
    new_sigma = (eye(size(sigma)) - K*H) * sigma;
    
    mu = new_mu;
    sigma = new_sigma;

end

sigma_list(1:3, 1:3) = sigma(1:3, 1:3); 

if (atualizacao_lista)

    Q_prov = eye(2*length(indexes_prov)).*0.1;

    mahalanobis_distance_prov = H_prov * sigma_list * H_prov' + Q_prov;
    K_prov = sigma_list*H_prov'*inv(mahalanobis_distance_prov);
    diff_Z_prov = wrapToPi(Z_prov - decidedZ_prov);
    landmarks_list = landmarks_list + K_prov(4:end, :) * diff_Z_prov;
    sigma_list = (eye(size(sigma_list)) - K_prov*H_prov) * sigma_list;

end

Seen_Landmarks = seen_ids;


for s = 2:2:length(counters)/2
    counters(s) = counters(s) + 1; % Aumentar o tempo de todos
end


s = 1;

while s <= (length(counters)/2)

    if counters(2*s-1) > 25
        % Se foi visto mais do que X vezes
        % passar para o mu

        mu = [mu; landmarks_list(2*s-1:2*s)];
        landmarks_list(2*s-1:2*s) = [];
        counters(2*s-1:2*s) = [];
        sigma =  [sigma zeros(2*N+3, 2);
                  zeros(2, 2*N+3) sigma_list(2*s-1:2*s, 2*s-1:2*s)];
        sigma_list(3+2*s-1:3+2*s, :) = [];
        sigma_list(:,3+2*s-1:3+2*s) = [];

    end

    s = s+1;
end

s = 1;

while s <= (length(counters)/2)
    
    if counters(2*s) > 150
        % Se já passou X tempo sem ser visto
        % apagar o número

        landmarks_list(2*s-1:2*s) = [];
        counters(2*s-1:2*s) = [];
        sigma_list(3+2*s-1:3+2*s, :) = [];
        sigma_list(:, 3+2*s-1:3+2*s) = [];

    end

 s = s+1;

end
