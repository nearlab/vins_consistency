pos 				= [6    7    3.5000    1.3];
papersize 		 	= pos(3:4);
fontsize	 		= 6;

% Estimates vs Truth
% c = {'r','b','k'};
ylabels = {'X pos (m)','Y pos (m)','\phi heading (rad)'};
titles = {'X_{robot} \pm 3\sigma','Y_{robot} \pm 3\sigma', 'Heading \pm 3\sigma'};
for i=1:3
	f(i) = figure(i); clf; hold on;
	for k=1:n_Monte
		plot(thist, XHIST(i,:,k)-XHATHIST(i,:,k), 'Color',[.7 .7 .7],'linewidth',1)
	end
	plot(thist(2:end), 3*mean(sqrt(PHIST(i+n*(i-1),:,:)),3), 'k','linewidth',1,'linestyle','--')
	plot(thist(2:end),-3*mean(sqrt(PHIST(i+n*(i-1),:,:)),3), 'k','linewidth',1,'linestyle','--')
	hold off;
	xlabel('time (sec)')
	ylabel(ylabels{i})
	title(titles{i})
end

f40 = figure(40); clf; hold on;
j = 1;
rob = plot(nan, nan, 'go', 'MarkerSize',8,'markerfacecolor','r');
robpath = plot(nan, nan, 'r-.', 'linewidth',2);
robestpath = plot(nan, nan, 'k');
robhead = quiver(XHIST(1,1,j), XHIST(2,1,j), 0.1*cos(XHIST(3,1,j)),0.1*sin(XHIST(3,1,j)),'linewidth',2);
esthead = quiver(XHIST(1,1,j), XHIST(2,1,j), 0.1*cos(XHATHIST(3,1,j)),0.1*sin(XHATHIST(3,1,j)),'linewidth',2);
feat_true = plot(fs(1,:), fs(2,:), 'ko', 'MarkerSize', 8,'linewidth',2,'markerfacecolor','y');
feat = plot(nan, nan, 'bo', 'MarkerSize', 8,'linewidth',2,'markerfacecolor','c');
fov = patch(nan, nan, 'red','facecolor',[0.9100 0.4100 0.1700], 'facealpha',0.5,'EdgeColor','none');
axis([-1.5 3.5 -2.5 2.5])
axis square
set(gca,'position',[-0.05 0.05 1 0.92],'units','normalized')
legend({'True Robot pos','True robot path','Est Robot path','True Heading','Est Heading','True feature pos','Est feature pos','Camera FOV'}, 'location', 'southeast')
for i=1:N
	feat_est = reshape(XHATHIST(4:end,i,j),2, []);
	set(feat,'XData',feat_est(1,:), 'YData',feat_est(2,:));
	set(rob,'XData',XHIST(1,i,j), 'YData',XHIST(2,i,j));
	set(robpath,'XData',XHIST(1,1:i,j), 'YData',XHIST(2,1:i,j));
	set(robestpath,'XData',XHATHIST(1,1:i,j), 'YData',XHATHIST(2,1:i,j));
	robhead.XData = XHIST(1,i+1,j); robhead.YData = XHIST(2,i+1,j);
	robhead.UData = 0.5*cos(XHIST(3,i+1,j)); robhead.VData = 0.5*sin(XHIST(3,i+1,j));
	esthead.XData = XHIST(1,i+1,j); esthead.YData = XHIST(2,i+1,j);
	esthead.UData = 0.5*cos(XHATHIST(3,i+1,j)); esthead.VData = 0.5*sin(XHATHIST(3,i+1,j));
	[xfov, yfov] = gen_fov(XHIST(1,i,j), XHIST(2,i,j), XHIST(3,i,j), fov_dist_min, fov_dist_max, fov_theta);
	set(fov, 'XData',xfov,'YData',yfov)
	drawnow;

	% Record GIF
	if 1
		frame = getframe(f40); 
		im = frame2im(frame); 
		[imind,cm] = rgb2ind(im,256); 

		% Write to the GIF File 
		if i == 1 
			imwrite(imind,cm,'img/ekfslam.gif','gif','DelayTime',0.1,'Loopcount',inf); 
		else 
			imwrite(imind,cm,'img/ekfslam.gif','gif','DelayTime',0.1,'WriteMode','append'); 
		end 
	end
	% pause(0.01)
end

hold off;



function [xfov, yfov] = gen_fov(x, y, phi, fov_dist_min, fov_dist_max, fov_theta, nsegments)  
if nargin<7
    nsegments=50;
end

th = (phi-fov_theta/2):fov_theta/nsegments:(phi+fov_theta/2);
xmax = fov_dist_max * cos(th) + x;
ymax = fov_dist_max * sin(th) + y;
xmin = fov_dist_min * cos(th) + x;
ymin = fov_dist_min * sin(th) + y;

xfov = [xmax, fliplr(xmin), xmax(1)];
yfov = [ymax, fliplr(ymin), ymax(1)];
end
