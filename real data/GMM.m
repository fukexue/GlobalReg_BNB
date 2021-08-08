function [E] = GMM(x,y1,y2,r,t,S0,sigma,f,f2,Tx_LAT,xs_LAT,ys_LAT,zs_LAT,Tx_AP,xs_AP,ys_AP,zs_AP)
% S0=mean(x,2);
R0=rotationVectorToMatrix(r);
x_=R0*(x-S0)+S0+t;
x_2D1=ProjectPoint(x_,f);
x_2=inv(Tx_LAT)*(Tx_AP*[x_;zeros(1,size(x_,2))]+[xs_AP;ys_AP;zs_AP;1]-[xs_LAT;ys_LAT;zs_LAT;1]);
x_2D2=ProjectPoint(x_2,f2);

%     figure,
%     hold on;
%     plot(y1(1,:),y1(2,:),'b.');%真实2d点
%     plot(x_2D1(1,:),x_2D1(2,:),'r.');%旋转以后的投影2d点
%     
%     
%     figure,
%     hold on;
%     plot(y2(1,:),y2(2,:),'b.');%真实2d点
%     plot(x_2D2(1,:),x_2D2(2,:),'r.');%旋转以后的投影2d点
    
    
M=size(x,2);
N1=size(y1,2);
N2=size(y2,2);
E=0;
for i=1:M
    for j=1:N1
        E=E-2*exp(-norm(x_2D1(:,i)-y1(:,j))/sigma);
    end
    for j=1:M
        if i~=j
            E=E+exp(-norm(x_2D1(:,i)-x_2D1(:,j))/sigma);
        end
    end
end
for i=1:M
    for j=1:N2
        E=E-2*exp(-norm(x_2D2(:,i)-y2(:,j))/sigma);
    end
    for j=1:M
        if i~=j
            E=E+exp(-norm(x_2D2(:,i)-x_2D2(:,j))/sigma);
        end
    end
end
end

