function [z,i]=GPAD(G,W,H,F,Geq,Weq,maxiter,epsG,epsV)
% Minimize .5*z'*H*z+F'*z
%������s.t. G*z<=W
%           Geq*z=Weq

%����zΪn*1ά
%��HΪn*n,Ϊ������
%GΪl*n
%WΪl*1
%GeqΪm*n
%WeqΪm*1
%FΪn*1

%maxiter ����������
%epsG
%epsV

%����ֵ�ǹ滮���z�͵�������i



F=F(:)';
m=numel(W); %����W�����е�Ԫ�ظ���
%����G��W����
G=[G;Geq]; %(l+m)*n
W=[W;Weq];

M=G*(H\G');  %����һ��(l+m)*(l+m)����
%Precondioning Ԥ����
Sm=full((diag(diag(M)))).^(1/2);%ȡ������M�еĶԽ�Ԫ�أ�Ȼ�����乹��һ���Խ���,�����full��û��ʲô��
M=Sm*M*Sm;
G=Sm*G;
W=Sm*W;

L=norm(M,'fro');  %frobenius����
L=1/L; %�����ĵ���
iMG=H\G';%n*(l+m)
iMc=H\F';%F֮ǰ��ת�ù�...  n*1
keepgoing=1;  %û�õı���
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

