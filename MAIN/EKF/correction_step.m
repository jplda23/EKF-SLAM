function [mu, sigma, observedLandmarks] = correction_step(mu, sigma, z, observedLandmarks)

N = (length(mu)-3)/2;

m = size(z, 2);

Z = zeros(m*2, 1);
expectedZ = zeros(m*2, 1);

H = [];

for i = 1:m
	landmarkId = z(i).id;

	if(observedLandmarks(landmarkId)==false)
        mu(3+landmarkId*2-1) = mu(1) + z(i).range * cos(wrapToPi(z(i).bearing+mu(3)));
        mu(3+landmarkId*2)   = mu(2) + z(i).range * sin(wrapToPi(z(i).bearing+mu(3)));
		
		observedLandmarks(landmarkId) = true;
end

    Z(i*2-1) = z(i).range;
    Z(i*2)   = z(i).bearing;
	
    b = [mu(3+landmarkId*2-1) - mu(1); mu(3+landmarkId*2) - mu(2)];
    q = b' * b;
    expectedZ(i*2-1) = sqrt(q);
    expectedZ(i*2) = wrapToPi(atan2(b(2), b(1)) - mu(3));

    Fx = [ eye(3) zeros(3, 2*N); zeros(2, 2*landmarkId+1) eye(2) zeros(2, 2*N-2*landmarkId) ];
    Hi = 1/q * [ -sqrt(q)*b(1), -sqrt(q)*b(2), 0, +sqrt(q)*b(1), +sqrt(q)*b(2);
                 +b(2), -b(1), -q, -b(2), +b(1) ] * Fx;
	
	H = [H;Hi];
end

Q = eye(2*m) .* 0.2;
for g = 1:m
    Q(2*g,2*g) = 2;
end

K = sigma*H'*inv(H*sigma*H'+Q);

diff_Z = wrapToPi(Z - expectedZ);

new_mu = mu + K * diff_Z;
new_sigma = (eye(size(sigma)) - K*H) * sigma;

mu = new_mu;
sigma = new_sigma;

end
