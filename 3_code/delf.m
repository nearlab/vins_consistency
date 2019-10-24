function F = delf(tk, xk, v, omega)

% Function: Calculates continuous time partial derivative of the state transition function
% Inputs: 	xk - 5 x 1 state of 2D robot position, 1D heading in radians and 2D feature position
%			tk - current time tk
%			v - robot velocity at time tk
%			omega - robot angular velocity at time tk
% Outputs:	F - Linearized state transition function

phiRk = xk(3);
pLk = xk(4:end);
n = length(xk);
F = zeros(n);
F(:,3) = [-v*sin(phiRk); v*cos(phiRk); 0; zeros(size(pLk))];

end
