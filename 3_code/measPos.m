function zk = measPos(xk, feature_found, fov_dist_min, fov_dist_max, fov_theta)

% Bearing measurement
pEk = reshape(xk(4:end), 2, []) - xk(1:2);
zk = pEk(:,feature_found);

% Vectorize measurements in [x1; y1; x2; y2; ..] format
zk = zk(:);

end

