function [extracted,f,tans_s] = Proj_extract2(extrated_point,Tx,xs,ys,zs)

% X_=extrated_point';
Y_1=extrated_point;
%%  Y_3是投影得到的2D点
[L] = Get_plane (Tx);
P0=[xs,ys,zs]';%P0:1*3
Y=ProjectPoint2(Y_1,P0,L); %Y_1在P0点投影到L上
Y_=[Y;ones(1,size(Y,2))];
%%
Y_2=inv(Tx)*Y_;
Y_3d=Y_2(1:3,:);
%%
Y_p=[Y_1;ones(1,size(Y_1,2))];
tans_Y=inv(Tx)*Y_p;
tans_Y=tans_Y(1:3,:);
tans_source=inv(Tx)*[xs,ys,zs,1]';
tans_source=tans_source(1:3,:);
%% source 变到原点
tans_Y=tans_Y-tans_source;
Y_3d=Y_3d-tans_source;
f=Y_3d(3,1);
tans_s=inv(Tx)*[xs,ys,zs,1]';
extracted=tans_Y;
tans_s=tans_s(1:2,:);
end