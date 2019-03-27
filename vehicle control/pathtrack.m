%�켣���ٷ���
%���ڵ����⣺
%1.�ο��켣���������⣬��Xref��Yref������������2���룬3�����
%  ���ֻ�ܿ����������������
%  ����ϲ�����Ĳο��켣�Ǳ仯�ģ���ôXref��YrefӦ����α仯�أ�
%2.

clc;clear;
Nx = 3;
Np = 15;
Nc = 2;
l = 1;

State_Initial = zeros(Nx,1);
State_Initial(1,1) = 0;
State_Initial(2,1) = 0;
State_Initial(3,1) = pi/6;

Q = 100*eye(Np+1);
R = 100*eye(Np+1);

N = 100;
T = 0.05;
Xref = zeros(Np,1);
Yref = zeros(Np,1);
PHIref = zeros(Np,1);

for j=1:1:N
    for Nref = 1:1:Np
        Xref(Nref,1) = (j+Nref-1)*T;%����涨��X�����ϣ��ο��켣��Զ��ÿ����ǰ��Np*T;���������ƺ���̫����
        Yref(Nref,1) = 2;
        PHIref(Nref,1) = 0;%û���õ�
    end
    
    lb = [0.8;-0.44;0.8;-0.44];
    ub = [1.2;0.44;1.2;0.44];%0.8<�ٶ�<1.2  -0.44<ת��Ƕ�<0.44
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    options = optimset('Algorithm','active-set');
    [A,fval,exitflag] = fmincon(@(x)MY_costfunction(x,State_Initial,Np,Nc,T,Xref,Yref,PHIref,Q,R),[0;0;0;0],A,b,Aeq,beq,lb,ub,[],options);
    v_actual = A(1);
    deltaf_actual = A(2);
    
    X00(1) = State_Initial(1,1);
    X00(2) = State_Initial(2,1);
    X00(3) = State_Initial(3,1);
    
    %ʵ�ʹ���
    XOUT = dsolve('Dx-v_actual*cos(z)=0','Dy-v_actual*sin(z)=0','Dz-v_actual*tan(deltaf_actual)/1=0','x(0)=X00(1)','y(0)=X00(2)','z(0)=X00(3)');
    t = T;
    State_Initial(1,1) = eval(XOUT.x);
    State_Initial(2,1) = eval(XOUT.y);
    State_Initial(3,1) = eval(XOUT.z);
    
    figure(1);
    plot(State_Initial(1,1),State_Initial(2,1),'b*');
    axis([0 5 0 3]);
    hold on;
    plot([0 5],[2 2],'r--');
end

function cost = MY_costfunction(x,State_Initial,Np,Nc,T,Xref,Yref,PHIref,Q,R)
cost = 0;
l = 1;
X = State_Initial(1,1);
Y = State_Initial(2,1);
PHI = State_Initial(3,1);

X_predict = zeros(Np,1);
Y_predict = zeros(Np,1);
PHI_predict = zeros(Np,1);

X_error = zeros(Np+1,1);
Y_error = zeros(Np+1,1);
PHI_error = zeros(Np+1,1);

v = zeros(Np,1);
delta_f = zeros(Np,1);

for i=1:1:Np
    if i == 1
        v(i,1)=x(1);
        delta_f(i,1)=x(2);
        X_predict(i,1) = X+T*v(i,1)*cos(PHI);
        Y_predict(i,1) = Y+T*v(i,1)*sin(PHI);
        PHI_predict(i,1)=PHI+T*v(i,1)*tan(delta_f(i,1))/l;
    else
        v(i,1)=x(3);
        delta_f(i,1)=x(4);
        X_predict(i,1)=X_predict(i-1)+T*v(i,1)*cos(PHI_predict(i-1,1));
        Y_predict(i,1)=Y_predict(i-1)+T*v(i,1)*sin(PHI_predict(i-1,1));
        PHI_predict(i,1)=PHI_predict(i-1)+T*v(i,1)*tan(delta_f(i-1,1));
    end

X_real = zeros(Np+1,1);%��һ���ֵ�ʵ����Щ����
Y_real = zeros(Np+1,1);
X_real(1,1) = X;
X_real(2:Np+1,1)=X_predict;
Y_real(1,1) = Y;
Y_real(2:Np+1,1) = Y_predict;
X_error(i,1)=X_real(i,1)-Xref(i,1);
Y_error(i,1)=Y_real(i,1)-Yref(i,1);
PHI_error(i,1)=PHI_predict(i,1)-PHIref(i,1);
end

cost = cost+Y_error'*R*Y_error+X_error'*Q*X_error;

end