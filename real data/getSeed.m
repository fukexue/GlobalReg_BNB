function [seeds] = getSeed(orign,side,num_x,num_y,num_z)
%GETSEED 
L=orign;
U=L+side;
seeds=[];
step=side./([num_x;num_y;num_z]*2);
star_point=L+[step(1);step(2);step(3)];
for x=1:num_x
    for y=1:num_y
        for z=1:num_z
            if x==1&&y==1&&z==1
                seeds=[seeds,star_point];  
            else
                seeds=[seeds,star_point+[(x-1)*step(1)*2;(y-1)*step(2)*2;(z-1)*step(3)*2]];  
            end
        end
    end
end



% seeds_=divided([L;U],num);
% seeds=0.5*(seeds_(1:3,:)+seeds_(4:6,:));
% seeds=[seeds,[0;0;0]];
end

