
clear
clc
close all

data = load("KoH_4400_Feb_2021.mat").KoH4400Feb2021;
data2 = load("2021 UTV-EMC-KOH Race_UPDATED.mat").data1;
time = data(:,1);
latG = data(:,2);
lonGpow = data(:,3);
verG = data(:,5);
gSpeed = data(:,6).*(5./18);
gSpeed(end) = 0;
bPressFront = data(:,12);
bPressRear = data(:,13);
fuelUsed = data(:,23);
tripDist = data(:,34);
gpsAlt = data(:,42)./3.281;
gpsLat = data(:,43);
gpsLon = data(:,44);
gpsAlt2 = data2(:,3);
gpsLat2 = data2(:,1);
gpsLon2 = data2(:,2);
origin = [gpsLat(1),gpsLon(1),gpsAlt(1)];
origin2 = [gpsLat2(1),gpsLon2(1),gpsAlt2(1)];
wgs84 = wgs84Ellipsoid;
[gpsX,gpsY,gpsZ] = geodetic2ned(gpsLat,gpsLon,gpsAlt,gpsLat(1),gpsLon(1),gpsAlt(1),wgs84);
[gpsX2,gpsY2,gpsZ2] = latlon2local(gpsLat2,gpsLon2,gpsAlt2,origin2);
gpsSpeed = data(:,48).*(5./18).*1.6;
engineSpeed = data(:,357); % RPM
throttlePedal = data(:,358);
throttleValve = data(:,359);
cltTemp = (5/9)*(data(:,361)-32);
% gSpeed(find(gSpeed>40.1)) = 40.1;
fr_new = data(:,411)


for i = 2:(length(gpsX)-1)
    dist(i) = sqrt(((gpsX(i)-gpsX(i-1)).^2) + ((gpsY(i)-gpsY(i-1)).^2));
    if dist(i)>40
        dist(i) = dist(i-2);
    end
end

for i = 1:(length(gpsLat))
    if gpsLat(i)==0 || gpsLon(i)==0
        gpsLat(i)=gpsLat(i-1);
        gpsLon(i) = gpsLon(i-1);
    end
end
[gpsX,gpsY,gpsZ] = latlon2local(gpsLat,gpsLon,gpsAlt,origin);
vertGrade = atand(diff(gpsZ)./dist');
vertGrade(find(abs(vertGrade)>=70))=70;
vertGrade = medfilt1(vertGrade,10);


mass = 2300;
fr = 0.25;
Cd = 0.454;
Af = 4.5;
rho_air = 1.224;
g=9.8;
acc = diff(gSpeed);

% Limiting Acceleration Noise
acc(find(abs(acc)>=9.8*1.2))=1.2.*9.8;

vertGrade = [deg2rad(vertGrade)];
rr = fr.*mass.*g.*cos(vertGrade).*gSpeed(1:end-1);
aero =  0.5.*Cd.*Af.*power(gSpeed(1:end-1),2).*rho_air.*gSpeed(1:end-1);
grade = mass.*g.*sin(vertGrade).*gSpeed(1:end-1);
force  = mass.*(acc) + fr.*mass.*g.*cos(vertGrade) + 0.5.*Cd.*Af.*power(gSpeed(1:end-1),2).*rho_air + mass.*g.*sin(vertGrade);
instPower = force.*gSpeed(1:end-1)./1000;
posPower = instPower;
posPower(find(posPower<0)) = 0;
posPower(find(isnan(posPower))) = 0;
negPower = instPower;
negPower(find(negPower>0)) = 0;
negPower(find(isnan(negPower))) = 0;
posEnergy = trapz(time(1:end-1),posPower);
negEnergy = trapz(time(1:end-1),negPower);
Energy_consumed = posEnergy/3600;
figure
plot(instPower)
hold on
ylabel("Power[kW]");
yyaxis right
plot(gSpeed)
xlabel("Time[m]");
ylabel("Speed[m/s]");



figure % Instant Power
gpsZ(find(gpsZ<0))=0;
stem3(gpsX(4300:end),gpsY(4300:end),gpsZ(4300:end))
hold on
instPower(find(instPower>400e3))=0;
hSc=scatter3(gpsX(4300:end), gpsY(4300:end), gpsZ(4300:end), 60, instPower(4299:end), 'filled')
title("GPS and Instant Power")
a = colorbar
ylabel(a,"Instant Power[kW]")
xlabel("X[m]");
ylabel("Y[m]");
zlabel("Z[m]");


figure % Throttle
stem3(gpsX(4300:end),gpsY(4300:end),gpsZ(4300:end))
hold on
hSc=scatter3(gpsX(4300:end), gpsY(4300:end), gpsZ(4300:end), 60, throttlePedal(4300:end), 'filled')
title("GPS and Throttle Pedal")
a = colorbar
ylabel(a,"Throttle Pedal[%]")
xlabel("X[m]");
ylabel("Y[m]");
zlabel("Altitude[m]");


figure %Speed
stem3(gpsX(4300:end)./1600,gpsY(4300:end)./1600,gpsZ(4300:end).*3.28)
hold on
hSc=scatter3(gpsX(4300:end)./1600,gpsY(4300:end)./1600,gpsZ(4300:end).*3.28, 60, gSpeed(4300:end).*(18./5./1.6), 'filled')
% title("GPS and Speed")
a = colorbar
ylabel(a,"Ground Speed[m/s]")
xlabel("X position [mi]");
ylabel("Y position [mi]");
zlabel("Z position [ft]");


figure % Grade
stem3(gpsX(4300:end),gpsY(4300:end),gpsZ(4300:end))
hold on
hSc=scatter3(gpsX(4300:end), gpsY(4300:end), gpsZ(4300:end),60, rad2deg(vertGrade(4299:end)), 'filled')
title("GPS and Vertical Grade")
a = colorbar
ylabel(a,"Grade [°]")
xlabel("X distance [km]");
ylabel("Y distance [km]");
zlabel("Altitude [m]");


figure %Braking Pressure front
stem3(gpsX(4300:end),gpsY(4300:end),gpsZ(4300:end))
hold on
hSc=scatter3(gpsX(4300:end), gpsY(4300:end), gpsZ(4300:end), 60, bPressFront(4300:end), 'filled')
title("GPS and Front Braking Pressure")
a = colorbar
ylabel(a,"Braking Pressure [kPa]")
xlabel("X[km]");
ylabel("Y[km]");
zlabel("Altitude[m]");

X = [25170 24442];
Y = [8928 7024];
Z = [578 508]
U = diff(X);
V = diff(Y);
W = diff(Z);
% quiver(X,Y,Z,U,V,W);
i=1;
j=1;
while i <=29567
    max2deploy(j) = trapz(time(i:i+1),posPower(i:i+1));
%     if any(gSpeed(i:i+1)>36)
%         max2deploy(j) =0;      
%     end
    max20deploy(j) = trapz(time(i:i+20),posPower(i:i+20));
%         if any(gSpeed(i:i+20)>36)
%         max20deploy(j) =0;      
%     end
    max5deploy(j) = trapz(time(i:i+5),posPower(i:i+5));
%             if any(gSpeed(i:i+5)>36)
%         max5deploy(j) =0;      
%     end
    max60deploy(j) = trapz(time(i:i+60),posPower(i:i+60));
%             if any(gSpeed(i:i+60)>36)
%         max60deploy(j) =0;      
%     end
    if i<(29687-1800)
    max1800deploy(j) = trapz(time(i:i+1800),posPower(i:i+1800));
%             if any(gSpeed(i:i+1800)>36)
%         max1800deploy(j) =0;      
%     end
    end
    if i<(29687-300)
    max300deploy(j) = trapz(time(i:i+300),posPower(i:i+300));
%             if any(gSpeed(i:i+300)>36)
%         max300deploy(j) =0;      
%     end
    end
    if i<(29687-3602)
    max3600deploy(j) = trapz(time(i:i+3600),posPower(i:i+3600));
%             if any(gSpeed(i:i+3600)>36)
%         max3600deploy(j) =0;      
%     end
    end
    j=j+1;
    i = i+1;
end
[~,max2] = max(max2deploy);
[~,max5] = max(max5deploy);
[~,max20] = max(max20deploy);
[~,max60] = max(max60deploy);
[~,max300] = max(max300deploy);
[~,max1800] = max(max1800deploy);
[~,max3600] = max(max3600deploy);

% set(0,'defaultAxesLineWidth',2.2)
% set(0,'defaultAxesFontSize',14)
% set(0,'defaultLineLinewidth',2.2)
% set(0,'defaultscatterLinewidth',1.5)
% set(0,'DefaultAxesBox','on')
% set(0,'DefaultAxesXGrid','on')
% set(0,'DefaultAxesYGrid','on')

figure % 2 second window
plot((max2:1:max2+1),instPower((max2):max2+1));
hold on
ylabel("Power [kw]");
str1 = sprintf("Average Power = %g kW",mean(instPower((max2):max2+1)));
yline(mean(instPower((max2):max2+1)),'-',str1);
yyaxis right
plot((max2:1:max2+1),gSpeed((max2):max2+1));
xlabel("time [s]")
ylabel("Speed [m/s]")
title("Max Power timestamp")
disp(max2deploy(max2))


figure % 5 second window
plot((max5:1:max5+5),instPower((max5):max5+5));
hold on
ylabel("Power [kw]");
str1 = sprintf("Average Power = %g kW",mean(instPower((max5):max5+5)));
yline(mean(instPower((max5):max5+5)),'-',str1);
yyaxis right
plot((max5:1:max5+5),gSpeed((max5):max5+5));
xlabel("time [s]")
ylabel("Speed [m/s]")

grid on
title("5 Second Power Draw, Power and Speed v Time")

figure % 20 second window
plot((max20:1:max20+20),instPower((max20):max20+20));
hold on
ylabel("Power [kw]");
str1 = sprintf("Average Power = %g kW",mean(instPower((max20):max20+20)));
yline(mean(instPower((max20):max20+20)),'-',str1);
yyaxis right
plot((max20:1:max20+20),gSpeed((max20):max20+20));
xlabel("time [s]")
ylabel("Speed [m/s]")

grid on
title("20 Second Power Draw, Power and Speed v Time")


figure % 60 second window
plot((max60:1:max60+60),instPower((max60):max60+60));
hold on
str1 = sprintf("Average Power = %g kW",mean(instPower((max60):max60+60)));
yline(mean(instPower((max60):max60+60)),'-',str1);
ylabel("Power [kw]");
% yyaxis right
% plot((max60:1:max60+60),gSpeed((max60):max60+60));
xlabel("time [s]")
% ylabel("Speed [m/s]")
grid on
title("60 Second Power Draw, Power and Speed v Time")

% 
% figure % 120 second window
% plot((max120:1:max120+120),instPower((max120):max120+120));
% hold on
% str1 = sprintf("Average Power = %g kW",mean(instPower((max120):max120+120)));
% yline(mean(instPower((max120):max120+120)),'-',str1);
% ylabel("Power [kw]");
% yyaxis right
% plot((max120:1:max120+120),gSpeed((max120):max120+120));
% xlabel("time [s]")
% ylabel("Speed [m/s]")
% grid on
% title("120 Second Power Draw, Power and Speed v Time")
% 
% 
figure % 300 second window
plot((max300:1:max300+300),instPower((max300):max300+300));
hold on
str1 = sprintf("Average Power = %g kW",mean(instPower((max300):max300+300)));
yline(mean(instPower((max300):max300+300)),'-',str1);
ylabel("Power [kw]");
yyaxis right
plot((max300:1:max300+300),gSpeed((max300):max300+300));
xlabel("time [s]")
ylabel("Speed [m/s]")
grid on
title("5min Power Draw, Power and Speed v Time")

figure % 1800 second window
plot((max1800:1:max1800+1800),instPower((max1800):max1800+1800));
hold on
str1 = sprintf("Average Power = %g kW",mean(instPower((max1800):max1800+1800)));
yline(mean(instPower((max1800):max1800+1800)),'-',str1);
ylabel("Power [kw]");
yyaxis right
plot((max1800:1:max1800+1800),gSpeed((max1800):max1800+1800));
xlabel("time [s]")
ylabel("Speed [m/s]")
grid on
title("30mins Second Power Draw, Power and Speed v Time")

figure % 3600 second window
plot((max3600:1:max3600+3600),instPower((max3600):max3600+3600));
hold on
str1 = sprintf("Average Power = %g kW",mean(instPower((max3600):max3600+3600)));
yline(mean(instPower((max3600):max3600+3600)),'-',str1);
ylabel("Power [kw]");
yyaxis right
plot((max3600:1:max3600+3600),gSpeed((max3600):max3600+3600));
xlabel("time [s]")
ylabel("Speed [m/s]")
grid on
title("60mins Second Power Draw, Power and Speed v Time");


%% Power  & Weight Visualizations
% 
% close all
% clear
% clc
i =1;
figure
lowPowLimit =400;
upperPowLimit = 400;
step = 200;
for powLimit = lowPowLimit:step:upperPowLimit
    weighting = 0:0.1:1;
    weighting = 1-weighting;
    motorDense = 6.666;
    dieselDense = 0.772;
    battDense = 0.18382;
    dieselPower(i,:) = flip(linspace(1,powLimit,11));
    dieselWeight = dieselPower(i,:)./dieselDense;
    motorPower = (1-weighting).*powLimit;
    motorWeight = 120;%motorPower./motorDense.*4;
    totalCap = 2975./1600.*powLimit;
    battCapacity(i,:) = dieselPower(i,:).*(1-weighting.*1)%totalCap*(1-(weighting)).^2-[0 3.*ones(1,10)]; %3 to account for regenerative energy
    battCapacity(i,end) = 850;
%     battCapacity = [0 10 45 90 150 200 250 300 350 400 800;0 20 25 37 40 55 100 160 220 400 800 ];
    battWeight(i,:) = battCapacity(i,:)./battDense;
    fuelLiter = 40.*3.78;%(totalCap - battCapacity(i,:))*4*3.6e6./(42e6);
    fuelWeight = fuelLiter.*0.8.*[1.10 ones(1,10)]; %Penalty for ICE engine economy
    totalPtweight = fuelWeight+battWeight(i,:)+motorWeight+dieselWeight;
    powerWeight(i,:) = totalPtweight;
    hold on
    battWeight(:,1) = 0;
    plot((1-weighting),totalPtweight)
    i=i+1;
end




% title("Vehicle Weight v Degree of Hybridness")
% xlabel("Hybridness")
% ylabel("Weight [kg]")
% legend("Power: 100kw","Power: 150kw","Power: 200kw","Power: 250kw","Power: 300kw","Power: 350kw","Power: 400kw","Power: 450kw","Power: 500kw","Power: 550kw","Power: 600kw","location","northwest")
% 
% figure
% surf(lowPowLimit:50:upperPowLimit,(1-weighting)',powerWeight')
% 
% title("Vehicle Weight v Degree of Hybridness")
% xlabel("Power")
% ylabel("Hybridness")
% zlabel("Weight [kg]")
% c=colorbar;
% ylabel(c,"Weight [kg]")
% 
% figure
% histogram(gSpeed);
% title("Velocity Histogram");
% xlabel("Speed [m/s]")
% ylabel("Instances")
% 
% figure
% histogram(posPower);
% title("Positive Power Histogram");
% xlabel("Power [kw]")
% ylabel("Instances")
% ylim([0 2500])
% 




%% Power v Laptime (Mass dependent on power)

% To find the minimum power required we find the addition of grade power
% and rolling resistance, all power levels should be greater than this
% otherwise the vehicle will not accelerate
vehSpeed = gSpeed
distance = vehSpeed.*1 + 0.5.*[acc;0].*1;
timeDelta = ones(1,4192);
% h = 1:length(weighting)
% distance = [0;distance];
instPower;
posPowerRef = posPower;
powerWeight = powerWeight(:,5)
timeDelta = zeros(29688,1)';
newAcc = zeros(1,29688);
fd_ratio =14
% n=1;
for h = 1:size(powerWeight,2)
    n=1;
for j=lowPowLimit./100:step./100:upperPowLimit./100
    mass = 1525 + 500 + powerWeight(n,h);
        n = n+1;
    force  = mass.*(acc) + fr.*mass.*g.*cos(vertGrade) + 0.5.*Cd.*Af.*power(gSpeed(1:end-1),2).*rho_air + mass.*g.*sin(vertGrade);
    instPower = force.*gSpeed(1:end-1)./1000;
    vehSpeed = gSpeed;
    posPower = instPower;
    posPower(find(posPower<0)) = 0;

    rr = fr.*mass.*g.*cos(vertGrade).*gSpeed(1:end-1);
    aero =  0.5.*Cd.*Af.*power(gSpeed(1:end-1),2).*rho_air.*gSpeed(1:end-1);
    grade = mass.*g.*sin(vertGrade).*gSpeed(1:end-1);
    % for powLimit = 400:50:600
    for i = 4192:length(instPower)
        i;
        if posPower(i-1)>j.*110
            disp("rekt")
        end
        if gSpeed(i+1)>(8000.*2.*pi./60./fd_ratio).*0.5
            veh_speed(i+1) = (8000.*2.*pi./60./fd_ratio).*0.5;
            accUpdate = (veh_speed(i+1)-vehSpeed(i))./timeDelta(i);
            posPower(i) = (mass.*(accUpdate) + fr.*mass.*g.*cos(vertGrade(i)) + 0.5.*Cd.*Af.*power(vehSpeed((i)),2).*rho_air + mass.*g.*sin(vertGrade(i))).*veh_speed(i)./1000;
            rr(i) = fr.*mass.*g.*cos(vertGrade(i)).*vehSpeed(i);
            aero(i) =  0.5.*Cd.*Af.*power(vehSpeed(i),2).*rho_air.*vehSpeed(i);
            grade(i) = mass.*g.*sin(vertGrade(i)).*vehSpeed(i);
        end
        if(posPower(i)>j.*100)
            newAcc(i)  = (j.*100000 - rr(i)-aero(i)-grade(i))./(mass.*vehSpeed(i));
            %         disp(newAcc(i))
            %         timeDelta(i) = (vehSpeed(i+1)-vehSpeed(i))./newAcc(i);
            syms t
            eqn = distance(i)==vehSpeed(i).*t + 0.5*newAcc(i).*t.^2;
            soln = double(solve(eqn));
            timeDelta(i) = min(soln(find(soln>0)));
            vcheck = vehSpeed(i) + newAcc(i).*timeDelta(i);
            force1  = mass.*(newAcc(i)) + fr.*mass.*g.*cos(vertGrade(i)) + 0.5.*Cd.*Af.*power(vehSpeed((i)),2).*rho_air + mass.*g.*sin(vertGrade(i));
            posPower(i) = force1.*vehSpeed(i)./1000;
            if vcheck<vehSpeed(i+1)
                vehSpeed(i+1) = vcheck;
%                 posPower(i) = force1.*vehSpeed(i+1)./1000;
                accUpdate = vehSpeed(i+2)-vehSpeed(i+1);
                posPower(i+1) = (mass.*(accUpdate) + fr.*mass.*g.*cos(vertGrade(i+1)) + 0.5.*Cd.*Af.*power(gSpeed((i+1)),2).*rho_air + mass.*g.*sin(vertGrade(i+1))).*gSpeed(i+1)./1000;
                rr(i+1) = fr.*mass.*g.*cos(vertGrade(i+1)).*vehSpeed(i+1);
                aero(i+1) =  0.5.*Cd.*Af.*power(vehSpeed(i+1),2).*rho_air.*vehSpeed(i+1);
                grade(i+1) = mass.*g.*sin(vertGrade(i+1)).*vehSpeed(i+1);
            end
        elseif (posPower(i)<posPowerRef(i)  && instPower(i)>0 && vehSpeed(i)>10)
            if(posPowerRef(i)<j.*100)
                newAcc(i)  = (posPowerRef(i).*1000 - rr(i)-aero(i)-grade(i))./(mass.*vehSpeed(i));
            else
                newAcc(i)  = (j.*100000 - rr(i)-aero(i)-grade(i))./(mass.*vehSpeed(i));
            end
            if newAcc(i)>1.18.*9.8
                newAcc(i) = 1.18.*9.8;
            end
            force1  = mass.*(newAcc(i)) + fr.*mass.*g.*cos(vertGrade(i)) + 0.5.*Cd.*Af.*power(vehSpeed((i+1)),2).*rho_air + mass.*g.*sin(vertGrade(i));
            posPower(i) = force1.*vehSpeed((i))./1000;
            syms t
            eqn = distance(i)==vehSpeed(i).*t + 0.5*newAcc(i).*t.^2;
            soln = double(solve(eqn));
            timeDelta(i) = min(soln(find(soln>0)));
            %         disp(timeDelta(i))
        elseif(vehSpeed(i) ~=gSpeed(i))
            accUpdate2 = vehSpeed(i+1)-vehSpeed(i);
            syms t
            eqn = distance(i)==vehSpeed(i).*t + 0.5*accUpdate2*t.^2;
            soln = double(solve(eqn));
            timeDelta(i) = min(soln(find(soln>0)));
        else
            timeDelta(i) =1;
        end
        % If aero force is greater, decelerate and do distance based
        % calculations
    end
    lapTime(n-1,h) = sum(timeDelta)
    posPower(find(isnan(posPower))) = 0;
    kwh_mi(n-1,h) = (trapz(cumsum(timeDelta(1:end-1)),posPower)./3.6e3)./((sum(distance)./1600))
    posPowerStore(n-1,h,:) = posPower;
    vehSpeedStore(n-1,h,:) = vehSpeed;
    timeStore(n-1,h,:) = timeDelta;
end
end
% figure
% plot(lowPowLimit:50:upperPowLimit,(lapTime(:,[1 2])+4192)./60)
% xlabel("Power [kw]")
% ylabel("Total Race Time [min]")
% title("Total Race Time v Vehicle Power");
% yline(29688./60,'-','Benchmark')
% hold on
% 
% hSc=scatter(lowPowLimit:50:upperPowLimit,(lapTime(:,1)+4192)./60,60, 1525+500+powerWeight(:,1), 'filled')
% hSc=scatter(lowPowLimit:50:upperPowLimit,(lapTime(:,2)+4192)./60,60, 1525+powerWeight(:,2), 'filled')
% % hSc=scatter(lowPowLimit:50:upperPowLimit,(lapTime(:,3)+4192)./60,60, 1525+500+powerWeight(:,3), 'filled')
% % hSc=scatter(lowPowLimit:50:upperPowLimit,(lapTime(:,4)+4192)./60,60, 1525+500+powerWeight(:,4), 'filled')
% % hSc=scatter(lowPowLimit:50:upperPowLimit,(lapTime(:,5)+4192)./60,60, 1525+500+powerWeight(:,5), 'filled')
% % hSc=scatter(lowPowLimit:50:upperPowLimit,(lapTime(:,6)+4192)./60,60, 1525+powerWeight(:,6), 'filled')
% legend("Eng Power only","Hybridised")
% c = colorbar;
% ylabel(c,'Mass [kg]');
% 
figure
hold on
plot(lowPowLimit:100:upperPowLimit,(lapTime(:,[1])-29687+4192))
% hSc=scatter(lowPowLimit:50:upperPowLimit,(lapTime(:,[1])-29687+4192),240, 1525+500+powerWeight(:,1), 'filled')
% hSc=scatter(lowPowLimit:50:upperPowLimit,(lapTime(:,[2])-29687+4192),240, 1525+500+powerWeight(:,2), 'filled')
% yline(0,'-','Benchmark 10th',"LabelHorizontalAlignment","Left")
% ylim([-50 550])
% c = colorbar;
% ylabel(c,'Mass [kg]')
ylim([-10 1000])
title("Drive Time Delta v Vehicle Power")
xlabel("Power [kw]")
ylabel("Time Gained(-)/Time Lost(+)")
ylim([-10 1000])
xline(300,"-",strcat("Mass:",num2str(round(1525+500+powerWeight(2),0))),"FontSize",15)
xline(700,"-",strcat("Mass:",num2str(round(1525+500+powerWeight(end -1),0))),"FontSize",15)
% yline(0-150,'-','9th Position',"LabelHorizontalAlignment","Left")
% yline(0+331,'-','11th Position',"LabelHorizontalAlignment","Left")
% 
% figure
% plot(lowPowLimit:50:upperPowLimit,(kwh_mi(:,1:end)))
% xlabel("Power [kw]")
% ylabel("kwh/mi]")
% title("KWH/mi v Vehicle Power");
% yline(4.83,'-','Benchmark')
% hold on
%% HWFET and UDDS Range 275mi sizing
% 
% HWFET = load('HWFET.mat');
% timeHWFET = HWFET.HWFET(:,1);
% HWFET = HWFET.HWFET(:,2).*1.6./18.*5;
% mass = 2500:50:3200;
% force  = mass.*(diff(HWFET)) + 0.03.*mass.*g.*cos(0) + 0.5.*Cd.*Af.*power(HWFET(1:end-1),2).*rho_air + mass.*g.*sin(0);
% instPower = force.*HWFET(1:end-1)./1000;
% posPower = instPower;
% posPower(find(posPower<0)) = 0;
% posEnergy = (trapz(timeHWFET(1:end-1),posPower)./3.6e3);
% negPower = instPower;
% negPower(find(negPower>0)) = 0;
% negEnergy = (trapz(timeHWFET(1:end-1),negPower)./3.6e3);
% energyHWFET = posEnergy + negEnergy.*0.15; %15% Regen Possible
% kwh_mi_HWFET = energyHWFET./10.26;
% 
% reqFuelHWFET = (275./10.26).*energyHWFET.*4./12.2222./0.7./3.78  %25% Overall Vehicle Efficiency
% 
% UDDS = load('UDDS.mat');
% timeUDDS = UDDS.Drivecycle(:,1);
% UDDS = UDDS.Drivecycle(:,2).*1.6./18.*5;
% 
% force  = mass.*(diff(UDDS)) + 0.03.*mass.*g.*cos(0) + 0.5.*Cd.*Af.*power(UDDS(1:end-1),2).*rho_air + mass.*g.*sin(0);
% instPower = force.*UDDS(1:end-1)./1000;
% posPower = instPower;
% posPower(find(posPower<0)) = 0;
% posEnergy = (trapz(timeUDDS(1:end-1),posPower)./3.6e3);
% negPower = instPower;
% negPower(find(negPower>0)) = 0;
% negEnergy = (trapz(timeUDDS(1:end-1),negPower)./3.6e3);
% energyUDDS = posEnergy + negEnergy.*0.15; %15% Regen Possible
% kwh_mi_UDDS  = energyUDDS./10.26;
% 
% reqFuelUDDS = (275./7.5).*energyUDDS.*4./12.2222./0.7./3.78;  %25% Overall Vehicle Efficiency
% 
% kwh_mi_combined = kwh_mi_UDDS.*(0.55) + 0.45.*kwh_mi_HWFET;
% reqFuel = 0.55*(reqFuelUDDS) + 0.45*(reqFuelHWFET)
% 
% figure % Fuel Required
% plot(mass,reqFuel);
% hold on
% ylabel("Fuel [gal.]");
% xlabel("Vehicle Mass [kg]");
% grid on
% title("Fuel Required for 275mi combined cycle");
% 
% rr = fr.*mass.*g.*cos(vertGrade).*gSpeed(1:end-1);
% aero =  0.5.*Cd.*Af.*power(gSpeed(1:end-1),2).*rho_air.*gSpeed(1:end-1);
% grade = mass.*g.*sin(vertGrade).*gSpeed(1:end-1);
% force  = mass.*(acc) + fr.*mass.*g.*cos(vertGrade) + 0.5.*Cd.*Af.*power(gSpeed(1:end-1),2).*rho_air + mass.*g.*sin(vertGrade);
% instPower = force.*gSpeed(1:end-1)./1000;
% posPower = instPower;
% posPower(find(posPower<0)) = 0;
% posPower(find(isnan(posPower))) = 0;
% negPower = instPower;
% negPower(find(negPower>0)) = 0;
% negPower(find(isnan(negPower))) = 0;
% posEnergy = trapz(time(1:end-1),posPower);
% negEnergy = trapz(time(1:end-1),negPower);
% figure
% plot(instPower)
% hold on
% ylabel("Power[kw]");
% yyaxis right
% plot(gSpeed)
% xlabel("Time[m]");
% ylabel("Speed[m/s]");
% 
% kwh_mi = posEnergy./3.6e3./180
% 
% figure % Fuel Required
% plot(mass,reqFuel);
% hold on
% ylabel("Fuel [gal.]");
% xlabel("Vehicle Mass [kg]");
% grid on

