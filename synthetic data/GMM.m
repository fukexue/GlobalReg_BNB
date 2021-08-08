function [E] = GMM(x,y1,y2,sigma,S0,r,t,Project_center,plane_z,plane_y)


R0=rotationVectorToMatrix(r);
x_=R0*(x-S0)+S0+t;
x_2D1=Project_point(x_,plane_z);
x_2D2=Project_point_Y(x_,Project_center,plane_y);
M=size(x,2);
N1=size(y1,2);
N2=size(y2,2);
E=0;
for i=1:M
    for j=1:N1
        E=E-2*exp(-norm(x_2D1(:,i)-y1(:,j))/sigma);
    end
    for j=1:M
        E=E+exp(-norm(x_2D1(:,i)-x_2D1(:,j))/sigma);
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

