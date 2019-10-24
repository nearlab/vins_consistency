function xdot = dyn_localize(tk, xk, v, omega)
	pRk = xk(1:2);
	phiRk = xk(3);

	xdot = [v*cos(phiRk); v*sin(phiRk); omega];

end
