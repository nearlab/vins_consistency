function theta = normAngle(angle)
% Both angle and theta are in radians
if(angle >= pi)
	theta = angle - 2*pi;
else if(angle < -pi)
	theta = angle + 2*pi;
else
	theta = angle;
end

end
