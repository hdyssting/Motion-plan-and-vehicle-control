clc;clear;

MaxSpeed = 50/3.6;
MaxLonAcc = 2;
MaxLatAcc = 1;
MaxCurvature = 1;
MaxRoadWidth = 7;
DeltaRoad = 1;
DeltaT = 0.2;
MaxT = 4;
MinT = 4;
TargetSpeed = 20/3.6;
DeltaSpeed = 5/3.6;
NumSpeedSample = 1;

surroundingCar_s_dot = 10/3.6;
surroundingCar_d = 0;


Kj = 0.1;
Kt = 0.1;
Kd = 1;
Klat = 1;
Klon = 1;

d_i = -MaxRoadWidth:DeltaRoad:MaxRoadWidth;
T_i = MinT:DeltaT:MaxT;
delta_s_dot_i = TargetSpeed-NumSpeedSample*DeltaSpeed:DeltaSpeed:TargetSpeed+NumSpeedSample*DeltaSpeed;

originalcoorref = struct('s',1:300,'d',zeros(1,300),'cost',0,'s_dot',0,'s_ddot',0,'s_dddot',0,'d_dot',0,'d_ddot',0,'d_dddot',0);
reftrajectory = [0.0, 50.0, 100.5, 150.0, 200.5, 250.0, 300.0;
                 0.0, -4.0,  1.0,  6.5,  8.0, 10.0,  6.0 ];
ob = [20 10;
      30 6;
      30 5;
      35 7;
      50 12];            
coorref = frenet_cartesian_conversion(reftrajectory,originalcoorref);

figure;
plot(coorref.x,coorref.y,'k','LineWidth',2);hold on;
% plot(ob(:,1),ob(:,2),'o');

t = 0:DeltaT:T_i;%improper
surroundingCar_s0 = 30;
surroundingCarFrenetLine = struct('s',surroundingCar_s0+surroundingCar_s_dot.*t,'d',surroundingCar_d*ones(1,length(t)),'cost',0,'s_dot',surroundingCar_s_dot,'s_ddot',0,'s_dddot',0,'d_dot',0,'d_ddot',0,'d_dddot',0);

d0 = 0;
d0_dot = 0;
d0_ddot = 0;

s0_dot = 10/3.6;
s0 = 0;


for step=1:500
    tic;
    allCandidateFrenetLines = [];
    for i=1:length(d_i)
        for j=1:length(T_i)
            dcoff = quintic_polynomial(d0,d0_dot,d0_ddot,d_i(i),0,0,T_i(j));
            frenetLine.t = t;
            frenetLine.dcoff = dcoff;
            frenetLine.d = dcoff(6)*t.^5+dcoff(5)*t.^4+dcoff(4)*t.^3+dcoff(3)*t.^2+dcoff(2)*t.^1+dcoff(1)*t.^0;
            frenetLine.d_dot = 5*dcoff(6)*t.^4+4*dcoff(5)*t.^3+3*dcoff(4)*t.^2+2*dcoff(3)*t.^1+dcoff(2)*t.^0;
            frenetLine.d_ddot = 20*dcoff(6)*t.^3+12*dcoff(5)*t.^2+6*dcoff(4)*t.^1+2*dcoff(3)*t.^0;
            frenetLine.d_dddot = 60*dcoff(6)*t.^2+24*dcoff(5)*t.^1+6*dcoff(4)*t.^0;
            
            for k=1:length(delta_s_dot_i)
                scoff = quartic_polynomial(s0,s0_dot,0,0,delta_s_dot_i(k),0,T_i(j));
                frenetLine.scoff = scoff;
                frenetLine.s = scoff(5)*t.^4+scoff(4)*t.^3+scoff(3)*t.^2+scoff(2)*t.^1+scoff(1)*t.^0;
                frenetLine.s_dot = 4*scoff(5)*t.^3+3*scoff(4)*t.^2+2*scoff(3)*t.^1+scoff(2)*t.^0;
                frenetLine.s_ddot = 12*scoff(5)*t.^2+6*scoff(4)*t.^1+2*scoff(3)*t.^0;
                frenetLine.s_dddot = 24*scoff(5)*t.^1+6*scoff(4)*t.^0;
                Jd = sum(frenetLine.d_dddot.^2);
                Js = sum(frenetLine.s_dddot.^2);
                deltaspeed = TargetSpeed - frenetLine.s_dot(end);
                dcost = Kj*Jd + Kt*T_i(j) + Kd * frenetLine.d(end)^2;
                scost = Kj*Js + Kt*T_i(j) + Kd*deltaspeed^2;
                pcost_car = potential_vehicle(frenetLine,surroundingCarFrenetLine);

                totalcost = Klat*dcost + Klon * scost+pcost_car;
                frenetLine.cost = totalcost;
                allCandidateFrenetLines = [allCandidateFrenetLines,frenetLine];
            end
        end
    end
    
    
    
    allCandidateCartesianLines = [];
    for count=1:length(allCandidateFrenetLines)
        originalcoor = allCandidateFrenetLines(count);
        coor = frenet_cartesian_conversion(reftrajectory,originalcoor);
        allCandidateCartesianLines = [allCandidateCartesianLines,coor];
    end
    
    okind = [];
    for count=1:length(allCandidateCartesianLines)
        if any(allCandidateCartesianLines(count).s_dot>min(MaxSpeed,sqrt(MaxLatAcc./abs(allCandidateCartesianLines(count).k))))
            continue;
        elseif any(abs(allCandidateCartesianLines(count).s_ddot)>MaxLonAcc)
            continue;
        elseif any(abs(allCandidateCartesianLines(count).k)>MaxCurvature)
            continue;
        elseif ~check_collision(allCandidateCartesianLines(count),ob)
            continue;
        end
        
        okind = [okind,allCandidateCartesianLines(count)];
    end
    
    bestLine = [];
    mincost = inf;
    for count=1:length(okind)
        if okind(count).cost<=mincost
            mincost = okind(count).cost;
            bestLine = okind(count);
        end
    end
    
    s0 = bestLine.s(2);
    s0_dot = bestLine.s_dot(2);
    d0 = bestLine.d(2);
    d0_dot = bestLine.d_dot(2);
    d0_ddot = bestLine.d_ddot(2);
    
    surroundingCar_s0 = surroundingCarFrenetLine.s(2);
    surroundingCarFrenetLine = struct('s',surroundingCar_s0+surroundingCar_s_dot.*t,'d',surroundingCar_d*ones(1,length(t)),'cost',0,'s_dot',surroundingCar_s_dot,'s_ddot',0,'s_dddot',0,'d_dot',0,'d_ddot',0,'d_dddot',0);
    surroundingCarCartesianLine = frenet_cartesian_conversion(reftrajectory,surroundingCarFrenetLine);
    toc;
    h1=plot(bestLine.x,bestLine.y,'r*');
    h2=plot(surroundingCarCartesianLine.x,surroundingCarCartesianLine.y,'bo');
    title(strcat('sdot:  ',num2str(bestLine.s_dot(1))))
    pause(0.1);
    
    photo = getframe(gcf);
    imind = frame2im(photo);
    [imind,cm] = rgb2ind(imind,256);
    if step == 1
        imwrite(imind,cm,'test.gif','GIF', 'Loopcount',inf,'DelayTime',0.1);
    else
        imwrite(imind,cm,'test.gif','GIF','WriteMode','append','DelayTime',0.1);
    end 
    
    delete(h1);
    delete(h2);
    
end



function flag = check_collision(cartesianLine,ob)
% RobotRadius = 2;
% for i=1:size(ob,1)
%     d = (cartesianLine.x-ob(i,1)).^2+(cartesianLine.y-ob(i,2)).^2;
%     if any(d<=RobotRadius^2)
%         flag = 0;return;
%     end
% end
flag = 1;return;
end










