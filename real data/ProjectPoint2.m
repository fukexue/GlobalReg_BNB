function P=ProjectPoint2(Point,P0,L)
%L：投影平面
%P1：被投点坐标
%P0：投影点
M=size(Point,2);

Point=[Point;ones(1,M)];
P0=[P0;1];%<----转成其次坐标
k=-L*P0./(L*(Point-P0));
P=k.*(Point-P0)+P0;%<----投影

P(end,:)=[];%<----转成非齐次坐标

end