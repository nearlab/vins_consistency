clear 

% Params
N 		= 100;
n_Monte	= 1;
p = 1;
l = 3;		% Size of process noise which doesn't go into truth
T = 0.05;
N_feat = 20;	% Number of features

% Feature positions
rng(10);
r_min = 0.8;
r_max = 1.4;
cent = [1; 0];

radii = r_min + (r_max - r_min)*rand(1, N_feat);
thetas = 2*pi*rand(1, N_feat);
fs = [cent(1) + radii.*cos(thetas); cent(2) + radii.*sin(thetas)];
% fs = [0.8, 1, 1.5, 2. 1.5, 0.5;
% 	1.2,  1, 0.9, 0.3, -0.8, -1];	% Size (2 x num_features)
% fs = [0; -1.2] + [2.3, 0; 0, 2.4]*rand(2,N_feat);

% Camera FOV params
fov_dist_min = 0.1;
fov_dist_max = 2;
fov_theta = pi/3;	% Total angle range in radians

% Initialization
x0 = [0; 0; pi/2];
n = length(x0);
P0 = diag([0.0001, 0.0001, deg2rad(0.001)]);
t0 = 0; tk = 0;
Qk = diag([0.001, 0.001, deg2rad(0.01)]);
Rbear = deg2rad(5);			% Measurement noise for bearing measurement (radians)
Rrange = 0.5;				% Measurement noise for range measurement (meters)
Gk = eye(l);

% Monte Carlo histories
XHIST = zeros(n, N+1, n_Monte);
XHATHIST = zeros(n, N+1, n_Monte);
PHIST = zeros(n^2, N, n_Monte);

for j = 1:n_Monte
tic
    xhat0			= x0 + chol(P0)'*randn(n,1);
	Pk 				= P0;
	Phist 			= [];
	Pbarhist		= [];
	zhist 			= {};
	xhist 			= x0;
	xhat 			= xhat0;
	xhathist 		= xhat;

	% EKF
	for k =1:N
		xk = xhist(:,end);

		% Calculate true control input
		% Circle: constant velocity and omega
		v = 1;
		omega = -1;

		% Truth propagation
		[~, Xode] = rk4fixed(@ (t,x) dyn_localize(t, x, v, omega), [tk, tk+T], xk, 100);
		xkp1 = Xode(end,:)';%+ Gk*chol(Qk)'*randn(l,1);
		xhist = [xhist xkp1];
		feature_found = isInFOV(xkp1, fs, fov_dist_min, fov_dist_max, fov_theta);
		n_seen = length(feature_found(feature_found > 0));

		% The net Rk is Rbearing for n_seen features and Rrange for n_seen features
		Rk = blkdiag(Rbear*eye(n_seen), Rrange*eye(n_seen));
		if sum(feature_found) > 0
			zkp1 = meas_localize(xkp1, fs, feature_found, fov_dist_min, fov_dist_max, fov_theta)...
					+ chol(Rk)'*randn(2*sum(feature_found),1);
			zhist{end+1} = zkp1;
		end

	    % Propagation
		[~, Xodebar] = rk4fixed(@ (t,x) dyn_localize(t, x, v, omega), [tk, tk+T], xhat, 100);
		xbar = Xodebar(end,:)';

		% Discretized state transition matrix Phik
		Phi0 = eye(n);
		[~, Phiode] = rk4fixed(@ (t, Phi) Phidot_localize(t, Phi, xhat, v, omega), [tk, tk+T], Phi0(:), 100);
		Phik = reshape(Phiode(end,:), [n, n]);
		Pbar = Phik*Pk*Phik' + Gk*Qk*Gk';

		% Update
		if sum(feature_found) > 0
			Hk = delh_localize(xbar, fs, feature_found, v, omega);
	    	Wk = Pbar*Hk'*((Hk*Pbar*Hk' + Rk)\eye(size(Rk)));
	    	Pk = (eye(n)-Wk*Hk)*Pbar*(eye(n)-Wk*Hk)' + Wk*Rk*Wk';
			nu = zhist{end} - meas_localize(xbar, fs, feature_found, fov_dist_min, fov_dist_max, fov_theta);
	    	xhat = xbar + Wk*nu;
		else
			xhat = xbar;
			Pk = Pbar;
		end

		% Update time
		tk = tk + T;

	    % Saving time history
	    xhathist = [xhathist xhat];
		Pbarhist = [Pbarhist Pbar(:)];
	    Phist = [Phist Pk(:)];
	end
	XHIST(:,:,j) = xhist;
	XHATHIST(:,:,j) = xhathist;
	PHIST(:,:,j) = Phist;

toc
end

thist = T*(0:N);

plotting

