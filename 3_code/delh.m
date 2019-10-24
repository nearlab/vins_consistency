function H = delh(xk, feature_found, v, omega)

% Partial derivative for a bearing measurement
pLk = reshape(xk(4:end), 2, []);
pEk = pLk - xk(1:2);
i_f = find(feature_found > 0);
sosEk = sum(pEk.^2,1)';
normEk = sqrt(sosEk);

% Bearing measurements
H_bear = [pEk(2,i_f)'./sosEk(i_f), -pEk(1,i_f)'./sosEk(i_f), ones(length(i_f),1)];

% Range measurements
H_range = [-pEk(1,i_f)'./normEk(i_f), -pEk(2,i_f)'./normEk(i_f), zeros(length(i_f),1)];

A_bear = zeros(length(i_f),length(xk)-3);
A_range = zeros(length(i_f), length(xk)-3);

for i=1:length(i_f)
	A_bear(i,2*i_f(i)-1:2*i_f(i)) = [-pEk(2,i_f(i))./sosEk(i_f(i)), pEk(1,i_f(i))./sosEk(i_f(i))];
	A_range(i,2*i_f(i)-1:2*i_f(i))= [pEk(1,i_f(i))./normEk(i_f(i)), pEk(2,i_f(i))./normEk(i_f(i))];
end

H = [H_bear, A_bear; H_range, A_range];

end
