function [r_out,t_out,E] = refine(y1,y2,x,r,t,S0,t_lu,epsilon,sigma,f,f2,tans_s,tans_s2,Tx_LAT,xs_LAT,ys_LAT,zs_LAT,Tx_AP,xs_AP,ys_AP,zs_AP)
%REFINE
% S0=mean(x,2);
y1=y1*0.1540-tans_s;
y2=y2*0.1540-tans_s2;
epsilon=epsilon*0.1540;
% lb=[[-0.023691,0.0093211,0.021836]-0.000005,[-8.0709,-7.3605,8.841]-0.1]';
% ub=[[-0.023691,0.0093211,0.021836]+0.000005,[-8.0709,-7.3605,8.841]+0.1]';
lb=[-pi,-pi,-pi,t_lu(1),t_lu(2),t_lu(3)]';
ub=[pi,pi,pi,t_lu(4),t_lu(5),t_lu(6)]';
fun=@(Q) GMM(x,y1,y2,Q(1:3),Q(4:6),S0,sigma,f,f2,Tx_LAT,xs_LAT,ys_LAT,zs_LAT,Tx_AP,xs_AP,ys_AP,zs_AP);
options = optimoptions('fmincon','display','off');
T = fmincon(fun,[r;t],[],[],[],[],lb,ub,[],options);
r_out=T(1:3);
t_out=T(4:6);

%---------------
R0_best=rotationVectorToMatrix(r);
x__best=R0_best*(x-S0)+S0+t;
x_2d_best=ProjectPoint(x__best,f);

N_best=size(x,2);
E_best=0;
for j_best=1:N_best
        flag=sum(sum(sum((x_2d_best(:,j_best)-y1).^2)<(epsilon).^2));  
        E_best=E_best+logical(flag);
end

%---------------
R0=rotationVectorToMatrix(r_out);
x_=R0*(x-S0)+S0+t_out;
x_2d=ProjectPoint(x_,f);

N=size(x,2);
E=0;
for j=1:N
        flag=sum(sum(sum((x_2d(:,j)-y1).^2)<(epsilon).^2));  
        E=E+logical(flag);
end
end