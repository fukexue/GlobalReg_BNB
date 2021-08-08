function out=Branch(branch)
% dividing the input branch by 2^3
%   
branch_seg_where=[4,1,4,1,4,1,4,1;2,5,5,2,2,5,5,2;3,3,3,6,6,6,6,3;7,4,7,4,7,4,7,4;5,8,8,5,5,8,8,5;6,6,6,9,9,9,9,6];
M=[branch(1:3),0.5*(branch(1:3)+branch(4:end)),branch(4:end)];
out=M(branch_seg_where);
% a=branch(1:3);      %lower
% b=branch(4:end);    %upper
% c=0.5*(a+b);        %middle
% M=[a,c,b];
% out=zeros(6,8);
% for i=1:8
%     out(1,i)= M(1,bitget(i,1)+1);
%     out(2,i)= M(2,bitget(i,2)+1);
%     out(3,i)= M(3,bitget(i,3)+1);
%     out(4,i)= M(1,bitget(i,1)+2);
%     out(5,i)= M(2,bitget(i,2)+2);
%     out(6,i)= M(3,bitget(i,3)+2);
% end

end