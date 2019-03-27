function [sys,x0,str,ts] = pathplan(t,x,u,flag)

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitialSizes;
    case 2
        sys = mdlUpdates(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u);
    case {1,4,9}
        sys = [];
    otherwise
        error(['unhandled flag = ',num2str(flag)]);
end


function [sys,x0,str,ts] = mdlInitialSizes

sizes = simsizes;
sizes.NumContStates = 0;
sizes.NumDiscStates = 5;
sizes.NumOutputs = 10;
sizes.NumInputs = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

x0 = [0.001;0.0001;0.0001;0.00001;0.00001;];
str = [];
ts = [0.1 0.001];%路径规划的周期设定为100ms

function sys = mdlUpdates(t,x,u)
sys = x;

function sys = mdlOutputs(t,x,u)

tic
Nx = 5;
Np = 15;
Nc = 2;
Nobs = 6;
T = 0.1;

y_dot = u(1)/3.6;
x_dot = u(2)/3.6;
phi = u(3)*pi/180;
phi_dot = u(4)*pi/180;
Y = u(5);
X = u(6);

shape = 2.4;
dx1 = 25;dx2 = 21.95;
dy1 = 4.05;dy2 = 5.7;
Xs1 = 27.19;Xs2 = 56.46;
X_phi=1:1:220;
z1 = shape/dx1*(X_phi-Xs1)-shape/2;
z2 = shape/dx2*(X_phi-Xs2)-shape/2;
Y_ref = dy1/2.*(1+tanh(z1))-dy2/2.*(1+tanh(z2));

State_Initial = zeros(Nx,1);
State_Initial(1,1) = y_dot;
State_Initial(2,1) = x_dot;
State_Initial(3,1) = phi;
State_Initial(4,1) = Y;
State_Initial(5,1) = X;

X_obstacle = zeros(Nobs,1);
X_obstacle(1:2) = 30;
X_obstacle(3:4) = 35;
X_obstacle(5:6) = 32.5;
Y_obstacle = zeros(Nobs,1);
Y_obstacle(1) = 0.5;
Y_obstacle(2)=1;
Y_obstacle(3)=0.5;
Y_obstacle(4)=1;
Y_obstacle(5)=0.5;
Y_obstacle(6)=1;

Yref=(Y_ref(1,round(State_Initial(5,1))+1:round(State_Initial(5,1))+15))';
Q = 100*eye(Np);
R = 20*eye(Nc);
S = 100;

mu = 0.4;
g = 9.8;
lb = [-mu*g;-mu*g];
ub = [mu*g;mu*g];
A=[];
b=[];
Aeq=[];
beq=[];

[A,fval,exitflag] = fmincon(@(x)MY_costfunction(x,State_Initial,Np,Nc,Nobs,T,Yref,Q,R,S,X_obstacle,Y_obstacle),[0;0;],A,b,Aeq,beq,lb,ub,[]);

fprintf('exitflag=%d\n',exitflag);

y_dot_predict = zeros(Np,1);
x_dot_predict = zeros(Np,1);
phi_predict = zeros(Np,1);
Y_predict = zeros(Np,1);
X_predict = zeros(Np,1);

for i=1:1:Np
    if i==Nc-1
        ay(i)=A(1);
        y_dot_predict(i,1)=State_Initial(1,1)+T*ay(i);
        x_dot_predict(i,1)=State_Initial(2,1);
        phi_predict(i,1)=State_Initial(3,1)+T*ay(i)/State_Initial(2,1);
        Y_predict(i,1)=State_Initial(4,1)+T*(State_Initial(2,1)*sin(State_Initial(3,1))+State_Initial(1,1)*cos(State_Initial(3,1)));
        X_predict(i,1)=State_Initial(5,1)+T*(State_Initial(2,1)*cos(State_Initial(3,1))-State_Initial(1,1)*sin(State_Initial(3,1)));
    else
        ay(i)=A(2);
        y_dot_predict(i,1)=y_dot_predict(i-1,1)+T*ay(i);
        x_dot_predict(i,1)=State_Initial(2,1);
        phi_predict(i,1)=phi_predict(i-1,1)+T*ay(i)/x_dot_predict(i-1,1);
        Y_predict(i,1)=Y_predict(i-1)+T*(State_Initial(2,1)*sin(phi_predict(i-1))+y_dot_predict(i-1)*cos(phi_predict(i-1)));
        X_predict(i,1)=X_predict(i-1)+T*(State_Initial(2,1)*cos(phi_predict(i-1))-y_dot_predict(i-1)*sin(phi_predict(i-1)));
    end
end

Parameter_X_Y=polyfit(X_predict,Y_predict,4);
Parameter_X_PHI = polyfit(X_predict,phi_predict,4);
OutPut(1:5)=Parameter_X_Y;
OutPut(6:10)=Parameter_X_PHI;
sys = OutPut;
toc;

function cost=MY_costfunction(x,State_Initial,Np,Nc,Nobs,T,Yref,Q,R,S,X_obstacle,Y_obstacle)
cost=0;
y_dot=State_Initial(1,1);
x_dot=State_Initial(2,1);
phi=State_Initial(3,1);
Y=State_Initial(4,1);
X_start=State_Initial(5,1);

y_dot_predict=zeros(Np,1);
x_dot_predict=zeros(Np,1);
phi_predict=zeros(Np,1);
Y_predict=zeros(Np,1);
X_predict=zeros(Np,1);
Y_error=zeros(Np,1);
J_obst=zeros(Np,1);
ay=zeros(Np,1);

for i=1:1:Np
    if i==Nc-1
        ay(i,1)=x(1);
        y_dot_predict(i,1)=y_dot+T*ay(i,1);
        x_dot_predict(i,1)=x_dot;
        phi_predict(i,1)=phi+T*ay(i,1)/x_dot;
        Y_predict(i,1)=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
        X_predict(i,1)=X_start+T*(x_dot*cos(phi)-y_dot*sin(phi));
        for j=1:1:Nobs
            J_obst(i,1)=J_obst(i,1)+1/((X_predict(i,1)-X_obstacle(j,1))^2+(Y_predict(i,1)-Y_obstacle(j,1))^2+0.000001);
        end
    else
        ay(i,1)=x(2);
        y_dot_predict(i,1)=y_dot_predict(i-1,1)+T*ay(i,1);
        x_dot_predict(i,1)=x_dot;
        phi_predict(i,1)=phi_predict(i-1,1)+T*ay(i)/x_dot_predict(i-1,1);
        Y_predict(i,1)=Y_predict(i-1)+T*(x_dot*sin(phi_predict(i-1))+y_dot_predict(i-1)*cos(phi_predict(i-1)));
        X_predict(i,1)=X_predict(i-1)+T*(x_dot*cos(phi_predict(i-1))-y_dot_predict(i-1)*sin(phi_predict(i-1)));
        for p=1:1:Nobs
            J_obst(i,1)=J_obst(i,1)+1/((X_predict(i,1)-X_obstacle(p,1))^2+(Y_predict(i,1)-Y_obstacle(p,1))^2+0.000001);
        end
    end
    Y_error(i,1)=Y_predict(i,1)-Yref(i,1);
end
cost=cost+Y_error'*Q*Y_error+ay(1:2)'*R*ay(1:2)+S*sum(J_obst(:));
    
        
        







