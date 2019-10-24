function fov_bool = isInFOV(xk, fs, fov_dist_min, fov_dist_max, fov_theta)
	pEk = fs - xk(1:2);
	phiRk = xk(3);
	normEk = sqrt(sum(pEk.^2,1));
	bool_max = normEk < fov_dist_max;
	bool_min = normEk > fov_dist_min;
	bool_theta = sum(pEk.*[cos(phiRk); sin(phiRk)],1)./normEk > cos(fov_theta/2);

	fov_bool = bool_max & bool_min & bool_theta;
end
