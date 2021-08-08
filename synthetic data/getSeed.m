function [seeds] = getSeed(orign,side,num)
%GETSEED 
L=orign;
U=L+side;
seeds_=divided([L;U],num);
seeds=0.5*(seeds_(1:3,:)+seeds_(4:6,:));
end

