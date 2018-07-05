function q=axangle2quat(r,theta)
q1=cos(theta/2);
q2=-sin(theta/2)*r(:,1);
q3=-sin(theta/2)*r(:,2);
q4=-sin(theta/2)*r(:,3);
q=[q1 q2 q3 q4];
end
