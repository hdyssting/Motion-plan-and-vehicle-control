function [z,i]=GPAD(G,W,H,F,Geq,Weq,maxiter,epsG,epsV)
% Minimize .5*z'*H*z+F'*z
%　　　s.t. G*z<=W
%           Geq*z=Weq

%假设z为n*1维
%则H为n*n,为正定阵
%G为l*n
%W为l*1
%Geq为m*n
%Weq为m*1
%F为n*1

%maxiter 最大迭代次数
%epsG
%epsV

%返回值是规划结果z和迭代次数i



F=F(:)';
m=numel(W); %返回W矩阵中的元素个数
%增广G和W矩阵
G=[G;Geq]; %(l+m)*n
W=[W;Weq];

M=G*(H\G');  %这是一个(l+m)*(l+m)矩阵
%Precondioning 预处理
Sm=full((diag(diag(M)))).^(1/2);%取出方阵M中的对角元素，然后用其构成一个对角阵,这里的full并没有什么用
M=Sm*M*Sm;
G=Sm*G;
W=Sm*W;

L=norm(M,'fro');  %frobenius范数
L=1/L; %范数的倒数
iMG=H\G';%n*(l+m)
iMc=H\F';%F之前被转置过...  n*1
keepgoing=1;  %没用的变量
i=0;
n=size(W,1);
y=zeros(n,1); %n = (l+m)
y0=y;

while keepgoing && (i<maxiter)

    beta=(i-1)/(i+2).*(i>0);
    
    w=y+beta*(y-y0);
    z=-iMG*w-iMc;
    
    s=L*G*z-L*W;
    
    y0=y;
    
    %Check termination conditions
    if all(s<=L*epsG)
        gapL=-w'*s;
        if gapL<=L*epsV
            return 
        end
    end
    y=w+s;
    y(1:m)=max(y(1:m),0);
    i=i+1;
    
end

