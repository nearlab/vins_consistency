function H = delh_localize(xk, fs, feature_found, v, omega)

% Partial derivative for a bearing measurement
pEk = fs - xk(1:2);
i_f = find(feature_found > 0);
sosEk = sum(pEk.^2,1)';
normEk = sqrt(sosEk);

% Bearing measurements
H_bear = [pEk(2,i_f)'./sosEk(i_f), -pEk(1,i_f)'./sosEk(i_f), ones(length(i_f),1)];

% Range measurements
H_range = [-pEk(1,i_f)'./normEk(i_f), -pEk(2,i_f)'./normEk(i_f), zeros(length(i_f),1)];

H = [H_bear; H_range];

end

