function phidot = Phidot_localize(tk, Phik, xk, v, omega)

n = sqrt(length(Phik));
Phik = reshape(Phik,[n, n]);
phidot = delf_localize(tk, xk, v, omega)*Phik;

phidot = phidot(:);

end

