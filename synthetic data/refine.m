function [r_out,t_out,E] = refine(r,t,x,y1,y2,S0,Project_center,plane_z,plane_y,t_lu,epsilon,sigma)
%REFINE
% S0=mean(x,2);
lb=[-pi,-pi,-pi,t_lu(1),t_lu(2),t_lu(3)]';
ub=[pi,pi,pi,t_lu(4),t_lu(5),t_lu(6)]';
fun=@(Q) GMM(x,y1,y2,sigma,S0,Q(1:3),Q(4:6),Project_center,plane_z,plane_y);
options = optimoptions('fmincon','display','off');
T = fmincon(fun,[r;t],[],[],[],[],lb,ub,[],options);
r_out=T(1:3);
t_out=T(4:6);

%---------------
R0=rotationVectorToMatrix(r_out);
x_=R0*(x-S0)+S0+t_out;
x_3D=Project_point(x_,plane_z);
x_2D=x_3D(1:2,:);
N=size(x,2);
E=0;
for j=1:N
        flag=sum(sum((x_2D(:,j)-y1).^2)<((epsilon).^2));  
        E=E+logical(flag);
end
end

