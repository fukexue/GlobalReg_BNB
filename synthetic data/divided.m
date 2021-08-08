function out=divided(branch,num)

a=branch(1:3);
b=branch(4:end);
M=length(a);
N=length(b);
if(M~=N)
    error('wrong num input');
end

bound=zeros(M,num+1);
for i=1:M
        bound(i,:)=linspace(a(i),b(i),num+1);
end

k=(num)^M;
out=zeros(M*2,k);

b=zeros(M+1,1);
for i=1:M+1
    b(i)=num.^(i-1);
end
index=zeros(M,1);
for i=0:k-1
    for j=1:M
       index(j)= floor(mod(i,b(j+1))/b(j))+1;
    end
    for j=1:M
        out(j,i+1)=bound(j,index(j));
        out(j+M,i+1)=bound(j,index(j)+1);
    end
end
end