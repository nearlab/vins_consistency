function zk = meas_localize(xk, fs, feature_found, fov_dist_min, fov_dist_max, fov_theta)

% Bearing and range measurement
pEk = fs - xk(1:2);
theta_fs = atan2(pEk(2,:),pEk(1,:))';
range_fs = sqrt(sum(pEk.^2,1))';
zk = [theta_fs(feature_found) - xk(3);
	  range_fs(feature_found)];

end

