function phidot = Phidot(tk, Phik, xk, v, omega)

n = sqrt(length(Phik));
Phik = reshape(Phik,[n, n]);
phidot = delf(tk, xk, v, omega)*Phik;

phidot = phidot(:);

end
