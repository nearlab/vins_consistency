function xdot = dyn(tk, xk, v, omega)
	pRk = xk(1:2);
	phiRk = xk(3);
	pLk = xk(4:end);

	xdot = [v*cos(phiRk); v*sin(phiRk); omega; zeros(size(pLk))];

end
