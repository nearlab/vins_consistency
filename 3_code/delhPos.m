function H = delhPos(xk, feature_found, v, omega)

% Partial derivative for a bearing measurement
pLk = reshape(xk(4:end), 2, []);
pEk = pLk - xk(1:2);
phiRk = xk(3);
i_f = find(feature_found > 0);

% relative feature position measurements
DCM = [-sin(phiRk), cos(phiRk); -cos(phiRk), -sin(phiRk)];
D = [cos(phiRk), -sin(phiRk); sin(phiRk), cos(phiRk)];
fs = D*pLk;
H = [repmat(DCM, length(i_f),1), fs(:)];

A = zeros(length(i_f), length(xk)-3);
A = , -repmat(DCM, length(i_f),1)
for i=1:length(i_f)
	A_bear(i,2*i_f(i)-1:2*i_f(i)) = [-pEk(2,i_f(i))./sosEk(i_f(i)), pEk(1,i_f(i))./sosEk(i_f(i))];
	A_range(i,2*i_f(i)-1:2*i_f(i))= [pEk(1,i_f(i))./normEk(i_f(i)), pEk(2,i_f(i))./normEk(i_f(i))];
end

H = [H_bear, A_bear; H_range, A_range];

end

