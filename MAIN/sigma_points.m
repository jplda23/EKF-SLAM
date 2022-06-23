function Q_new = sigma_points(Q, u)

    points = zeros(5,2);

    n = 5;
    
    lambda = 3-n;

    points(1,:) = u;
    
    aux = sqrt((n+lambda)*Q);
    points(2,:) = u + aux(1,:);
    points(3,:) = u + aux(2,:);
    points(4,:) = u - aux(1,:);
    points(4,:) = u - aux(2,:);
    
    
    Q_new = cov(points(:,1), points(:,2));

end

