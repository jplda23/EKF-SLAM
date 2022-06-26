function [mu, sigma] = prediction_step(mu, sigma, u, first)

N = (length(mu)-3)/2;
Fx = [ eye(3) zeros(3, N*2) ];

new_mu = mu + Fx' * [ u.t*cos( wrapToPi( mu(3) + u.r1 ) );
                      u.t*sin( wrapToPi( mu(3) + u.r1 ) );
                      wrapToPi( u.r1 + u.r2 ) ];

Gx = [0, 0, -u.t*sin( mu(3) + u.r1 );
      0, 0, u.t*cos( mu(3) + u.r1 );
      0, 0, 0 ];

G = eye(2*N+3) + (Fx' * Gx * Fx);

motionNoise = 0.0001;
R3 = diag([motionNoise, motionNoise, motionNoise]);
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

if (first == false)
  new_sigma = G*sigma*G' + R;
  sigma = new_sigma;
end

mu = new_mu;

end
