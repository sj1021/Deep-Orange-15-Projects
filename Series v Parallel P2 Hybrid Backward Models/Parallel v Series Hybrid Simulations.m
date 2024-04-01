% clear
close all
clc


%%
% Assumption list
% All engines have some torque curve, it is just interpolated linearly;
% The motor does not have a end RPM limit
% The best efficiency of the engine lies at the top torque line and is
% constant
% In Series the drive motor has infinite RPM, torque is limited by power
% Two yasa motors as generator in Parallel and 4 yasa motors as drive in
% series

%%
%loading the data for power and velocity
load('2600kg_data_frvar.mat');
% load('Animation for Parallel v Series\Data.mat')
% load('rpmx_torqy_yasaeffmap.mat');
time=timeStore(:);
veh_vel=vehSpeedStore(:);
veh_pow = posPowerStore(:);
%%

%Engine Data taken from
%\53_powertrain\12_specsheet\Ecodiesel\torquecurve.png
eng_rpm = [0 500 1000 1500 2000 2500 3000 3500 4000 4500 5000];
eng_norm_pow = [10 37.2 37.2 97 126 157 185 193 190 162 140]./193;
eng_bsfc_lookup = interp2([1000 1500 2e+3 2.5e3 3e+3 3.5e3 4e+3 4.5e3],[0 50 100 150 200 250 300]*2,[320 208 191 190 191 190 190;325 200 189 186 184 187 186;330 208 190 186 183 184 183;335 209 192 187 183 185 182;340 209 193 188 187 187 184;345 210 195 193 192 192 188;350 211 203 195 197 195 193;355 213 208 200 197 200 194]',eng_rpm.*ones(11,1),linspace(0,600,11)'.*ones(1,11));
eng_bsfc_func = griddedInterpolant((eng_rpm.*ones(11,1))',(linspace(0,600,11)'.*ones(1,11))',(eng_bsfc_lookup)');

%Setting the efficiencies of each component through the powertrain
batt_eff_inv = 1./0.96;

%Motor Efficiency Map
parallel_mot_eng_ratio = 8000./5000;  %To match the max motor RPM to engine RPM
mgu_eff_series = 0.94; % Yasa Datasheet single RPM (combined motor and controller efficiency)

%Below data has been extracted from YASA P400R datasheet
mgu_rpm = [0 1000 2000 3000 4000 5000 6000 7000 8000];
mgu_torq = [0 50 100 150 200 250 300 350 370];
% mgu_eff_lookup = yasaEffMap./100;
% mgu_eff_lookup = interp2(mgu_rpm,mgu_torq,mgu_eff_lookup,mgu_rpm.*ones(9,1),linspace(0,370,9)'.*ones(1,9));
mgu_torq = linspace(0,370,9);
% mgu_eff_function = griddedInterpolant((mgu_rpm.*ones(9,1))',(mgu_torq'.*ones(1,9))',(mgu_eff_lookup)');
mgu_eff_parallel = 0.9;
mgu_eff_series = 0.94;
mgu_eff_series_drive = 0.9;
%Driveline data
global gear_nos;
gear_nos = 6;
driveline_eff_parallel = 0.97.*0.94.*0.97*0.97; %Transfer case-differential-cvaxles-gearbox  https://huei.engin.umich.edu/wp-content/uploads/sites/186/2015/02/SAE_2006-01-0443.pdf
driveline_eff_series = 0.98; %cv axles
fdr_ratio = 14.5;

%Defining the control strategy
%Series: Engine is run at single point that has maximum power and once
%battery reaches xx% soc it shuts down and switches back on when soc is at
%yy%
eng_on_soc_series = 0.98; %Engine switches on at 95%
eng_off_soc_series = 0.98; %Engine switches off at 98%

%Parallel: Engine is running at max torque at each RPM, once the battery
%reaches xx% the engine load follows and electric motor only assists
eng_loadfollow_soc_parallel = 1; %Engine switches on at 95%
eng_maxtorq_soc_parallel = 1; %Engine switches off at 98%

%Running the loop for the following engine powers and battery sizes
eng_maxPow = 130:2:132;
batt_cap = (30:2:32).*(3.6e6);


%initializing variables
derate_parallel=zeros(length(eng_maxPow),length(batt_cap));
derate_series=zeros(length(eng_maxPow),length(batt_cap));

%function optimization
% Precompute constants
ratios = [4.0000 7.4000 10.8000 14.2000 17.6000 21.0000];
upper_lim_eng_spd = 5000;
lower_lim_eng_spd = 700;
conversion_constant = 60 / (0.5 * 2 * pi);





for i = 1:length(eng_maxPow)

    eng_pow = eng_norm_pow.*eng_maxPow(i);
    pow_interp = griddedInterpolant(eng_rpm, eng_pow);
    % Compute the gridded interpolant
    eng_powers_func = griddedInterpolant(eng_rpm,eng_pow);
    %Once a previous row is zero stop the for loop
    if i~=1
        if all(derate_series(i-1,:)==0) && all(derate_parallel(i-1,:)==0)
            break;
        end
    end
    for j = 1:length(batt_cap)
        %once a previous cell in the same row is zero, no need to calculate
        %the next cell in the same row
        if i~=1 && j~=1
            if all(derate_series(i,j-1)==0) && all(derate_parallel(i,j-1)==0)
                continue;
            end
        end
        %initializing variables

        batt_energy_series = zeros(length(veh_pow),1);
        batt_energy_parallel = zeros(length(veh_pow),1);
        temp_mot_torq_parallel = zeros(length(veh_pow),1);
        temp_mot_rpm_parallel = zeros(length(veh_pow),1);
        temp_mot_torq_series_drive = zeros(length(veh_pow),1);
        temp_mot_rpm_series_drive = zeros(length(veh_pow),1);
        batt_energy_series(4192) = batt_cap(j);
        batt_energy_parallel(4192) = batt_cap(j);
        soc_series = zeros(1,length(veh_pow)+1);
        soc_parallel = zeros(1,length(veh_pow)+1);
        eng_pow_series = zeros(1,length(veh_pow)+1);
        eng_pow_parallel = zeros(1,length(veh_pow)+1);
        eng_rpm_inst = zeros(1,length(veh_pow));
        eng_torq_inst = zeros(1,length(veh_pow));
        eng_fuel_consumed_parallel =0;
        eng_fuel_consumed_series = 0;
        tic
        for k = 4192:length(veh_pow)

            if k == 16491
                k;
            end
            %Calculating Energy flow in series
            %first calculating the drive motors rpm
            %             temp_mot_pow_drive = veh_pow(k)./driveline_eff_series;
            %             temp_mot_rads_series_drive = veh_vel(k).*fdr_ratio.*1./(0.5.*1);
            %             temp_mot_rpm_series_drive(k) = veh_vel(k).*fdr_ratio.*60./(0.5.*2.*pi);
            %             temp_mot_torq_series_drive(k) = temp_mot_pow_drive./temp_mot_rads_series_drive.*1000./(4);
            %             if temp_mot_rads_series_drive>=8000*(2.*pi./60)
            %             mgu_eff_series_drive = 0.9;
            %             else
            %                 mgu_eff_series_drive =mgu_eff_function(temp_mot_rpm_series_drive(k),min(abs(temp_mot_torq_series_drive(k)),370));
            %             end
            %             if isnan(mgu_eff_series_drive)
            %                 disp(temp_mot_torq_series_drive);
            %             end

            %             soc_series(k) = batt_energy_series(k)./batt_energy_series(4192);
            %             if soc_series(k)>=eng_off_soc_series
            %                 eng_pow_series(k) = 0;
            %             else
            %                 eng_pow_series(k) =eng_maxPow(i);
            %             end
            eng_pow_series(k) =eng_maxPow(i);
            batt_energy_series(k+1) = min(max(batt_energy_series(k) - (max(((veh_pow(k)./(driveline_eff_series.*mgu_eff_series_drive))-eng_pow_series(k).*mgu_eff_series)).*1000.*time(k)).*batt_eff_inv,0),batt_energy_series(4192));
            %             eng_fuel_consumed_series = eng_fuel_consumed_series + (eng_pow_series(k).*eng_bsfc_func(3600,600).*time(k))./3600;

            %Calculating Energy flow in parallel
            %             gear = gearSelector(veh_vel(k),eng_rpm,eng_pow);
            gear = gearSelector(veh_vel(k),eng_rpm,eng_pow, ratios, upper_lim_eng_spd, lower_lim_eng_spd, eng_powers_func, conversion_constant);
    
            soc_parallel(k) = batt_energy_parallel(k)./batt_energy_parallel(4192);
            eng_rpm_inst(k) = veh_vel(k).*gear.*60./(0.5.*2.*pi);
            ref_power = max(pow_interp(eng_rpm_inst(k)),0);
            if soc_parallel(k)<eng_loadfollow_soc_parallel
                eng_pow_parallel(k) = ref_power;

            else
                eng_pow_parallel(k) =ref_power;
            end

            %             eng_torq_inst(k) = eng_pow_parallel(k).*1000./(eng_rpm_inst(k).*(2.*pi./60));
            temp_mot_pow_p2 = ((veh_pow(k)./driveline_eff_parallel)-eng_pow_parallel(k));
            para_pow(k) = ((veh_pow(k)./driveline_eff_parallel)-eng_pow_parallel(k));
            seri_pow(k) = ((veh_pow(k)./(driveline_eff_series.*mgu_eff_series_drive))-eng_pow_series(k));
            %             temp_mot_rads_parallel = eng_rpm_inst(k).*parallel_mot_eng_ratio.*2.*pi./60;
            %             temp_mot_rpm_parallel(k) = eng_rpm_inst(k).*parallel_mot_eng_ratio;
            %             if temp_mot_rpm_parallel(k)<1000
            %             temp_mot_rpm_parallel(k)
            %             end
            %             temp_mot_torq_parallel(k) = temp_mot_pow_p2./temp_mot_rads_parallel.*1000./(2);
            % %             if temp_mot_rads_parallel>=8000*(2.*pi./60)
            % %                 mgu_eff_parallel = 0.9;
            % %             else
            % %                 mgu_eff_parallel = mgu_eff_function(temp_mot_rpm_parallel(k),min(abs(temp_mot_torq_parallel(k)),370));
            % %             end
            %             if isnan(mgu_eff_parallel)
            %                 disp(temp_mot_torq_parallel);
            %             end

            batt_energy_parallel(k+1) = min(max(batt_energy_parallel(k) - temp_mot_pow_p2.*1000.*time(k).*batt_eff_inv.*(1./mgu_eff_parallel),0),batt_energy_parallel(4192));
            %             eng_fuel_consumed_parallel = eng_fuel_consumed_parallel + (eng_pow_parallel(k).*min(220,eng_bsfc_func(max(eng_rpm_inst(k),1000),min(eng_torq_inst(k),600))).*time(k)./3600);
            if batt_energy_parallel(k) < temp_mot_pow_p2.*1000.*time(k).*batt_eff_inv.*(1./mgu_eff_parallel)
                derate_parallel(i,j) = derate_parallel(i,j)+1;
                %                 max_c_parallel(i,j) =
            end
            if batt_energy_series(k) < (((veh_pow(k)./(driveline_eff_series.*mgu_eff_series_drive))-eng_pow_series(k).*mgu_eff_series).*1000.*time(k)).*batt_eff_inv
                derate_series(i,j) = derate_series(i,j)+1;
            end

        end
         toc
    end
end
%%
figure
contour(batt_cap./3.6e6,eng_maxPow,derate_series,[0 1],'ShowText','on')
hold on
contour(batt_cap./3.6e6,eng_maxPow,derate_parallel,[0 1],'ShowText','on')

xlabel("Battery Capacity [kwh]")
ylabel("Engine Power [kw]")
title("Derate Counts v (battery capacity,engine power)")

% Series drive motors operation points
figure
hold on
contourf([0 1000 2000 3000 4000 5000 6000 7000 8000],[0 50 100 150 200 250 300 350 370],yasaEffMap./100);
scatter(temp_mot_rpm_series_drive, temp_mot_torq_series_drive,"r");
title("Series Drive Motor Operating Points")
xlabel("Motor RPM");
ylabel("Motor Torque");

% Parallel Generator Operating Points\
figure
hold on
contourf([0 1000 2000 3000 4000 5000 6000 7000 8000],[0 50 100 150 200 250 300 350 370],yasaEffMap./100);
contourf([0 1000 2000 3000 4000 5000 6000 7000 8000],-[0 50 100 150 200 250 300 350 370],yasaEffMap./100)
scatter(temp_mot_rpm_parallel, temp_mot_torq_parallel,"r");
title("Parallel Motor Generator Operating Points")
xlabel("Motor RPM");
ylabel("Motor Torque");
ylim([-400 400])

% Parallel Engine Operating Points
figure
hold on
contourf([1000 1500 2e+3 2.5e3 3e+3 3.5e3 4e+3 4.5e3],[0 50 100 150 200 250 300]*2,[320 208 191 190 191 190 190;325 200 189 186 184 187 186;330 208 190 186 183 184 183;335 209 192 187 183 185 182;340 209 193 188 187 187 184;345 210 195 193 192 192 188;350 211 203 195 197 195 193;355 213 208 200 197 200 194]',[180 185 190 200 210 220 250 300],'ShowText','on')
scatter(eng_rpm_inst(4192:end),eng_torq_inst(4192:end),"r")
xlabel("Engine RPM")
ylabel("Engine Torque")
title("Parallel Engine Operating Points 1000 speed gearbox")
ylim([0 650])
xlim([0 4500])

%%
[a,b] = find(derate_parallel==0);
i=6;
while i<=length(batt_cap)
    if isempty(min(a(find(b==i))))
        i=i+1;
        continue;
    end
    req_eng_parallel(i) = eng_maxPow(min(a(find(b==i))))
    req_batt_parallel(i) = batt_cap(i);
    i=i+1;
end

[a,b] = find(derate_series==0);
i=6;
while (i)<=length(batt_cap)
    if isempty(min(a(find(b==i))))
        i=i+1;
        continue;
    end
    req_eng_series(i) = eng_maxPow(min(a(find(b==i))))
    req_batt_series(i) = batt_cap(i);
    i=i+1;
end
req_batt_series = req_batt_series./3.6e6;
req_batt_parallel = req_batt_parallel./3.6e6;

engDens = 0.85;
battDens = 0.18;
engPow = 000:400;
battCap = 00:100;
for i = 1:length(engPow)
    for j = 1:length(battCap)
        ptMass(i,j) = engPow(i)./engDens + battCap(j)./battDens + 158 + 120 + 60 + 2 + 11 +60+ 20 + 5.9 + 25.8 + 70 + 20 + 50 +50 +20 +50 ; %43 is auxiliary %120kg fuel tank %20 is for motor mass, 50 is radiator
    end
end

figure
contour(battCap,engPow,ptMass,[0 1000 1250 1500],"ShowText","on")
hold on

% plot(req_batt_parallel,req_eng_parallel,"LineWidth", 3);
plot(req_batt_series,req_eng_series,"LineWidth",3);
plot(req_batt_series,req_eng_series,"LineWidth",3);
xlabel("Battery Capacity [kwh]")
ylabel("Engine Power at Crank [kw]")
title("Engine Battery Tradespace")
legend("Constant PT Mass (kg)","","Series Powertrain")

ylim([50 200])
xlim([24 80])

%%
close all
f1 = figure('Color', [1 1 1]);

paren = @(x, varargin) x(varargin{:});
timeRange = 16502:17319
distMax = max(cumsum(dist(timeRange))./1600);

%Car path
subplot(2,3,1)
PlotPosition1 = [0.1, 0.6, 0.55, 0.15];
plot(cumsum(dist(timeRange))./1600,gpsZ(timeRange))
ylim([-105 100])
xlim([0 distMax])
title("Car Path")
ylabel("Car Elevation [m]")
xlabel("Distance Travelled [mi]")
ax1 = gca;

%SoC Series
subplot(2,3,2)
PlotPosition2 = [0.75, 0.1, 0.05, 0.65];
ax2 = gca;
title("Series")
ylabel("Percentage (%)")

%Driver Power Demand
subplot(2,3,4)
PlotPosition3 = [0.1, 0.1, 0.55, 0.35];
ax3 = gca;
plot(cumsum(dist(timeRange))./1600,movmean(reshape(posPowerStore(timeRange),[(length(timeRange)) 1]),20))
xlim([0 distMax ])
xlabel("Distance Travelled [mi]")
ylabel("Driver Power Demand [kW]")
title("Driver Power Demand")

%SoC Parallel
subplot(2,3,3)
PlotPosition4 = [0.9, 0.1, 0.05, 0.65];
ax4 = gca;
rectangle(ax4,"Position",[0 0 0.2 1],"FaceColor","Blue")
text(0.1,0.5,"SoC(%)","HorizontalAlignment",'center',"FontSize",5)

title("Parallel")
set(ax1,'Position',PlotPosition1);
set(ax2,'Position',PlotPosition2);
set(ax3, 'Position', PlotPosition3);
set(ax4, 'Position', PlotPosition4);
hold(ax1,'on')
p1 = plot(ax1,max(cumsum(dist(timeRange((1)))))./1600,gpsZ(timeRange(1)),"rO");
p1_1 =  plot(ax1,max(cumsum(dist(timeRange((1)))))./1600,gpsZ(timeRange(1)),"b","Marker","square")
hold(ax3,'on')
p3 = plot(ax3,cumsum(dist(timeRange(1)))./1600,reshape(posPowerStore(timeRange(1)),[(length(timeRange(1))) 1]),"r*");

F(length(timeRange)) = struct('cdata',[],'colormap',[]);
filename = 'animation.gif'
n=1;
for i = min(timeRange):3:max(timeRange)
    % Delete points on distance and power plots
    delete(p1)
    delete(p1_1)
    delete(p3)
    %Create empty box for SoC axes
    hold(ax4,'off')
    hold(ax2,'off')
    rectangle(ax4,"Position",[0 0 0.2 100],"FaceColor","white","EdgeColor","white")
    rectangle(ax2,"Position",[0 0 0.2 100],"FaceColor","white","EdgeColor","white")
    hold(ax4,'on')
    hold(ax2,'on')
    %SoC Animation for Parallel
    if  (batt_energy_parallel(i) - batt_energy_parallel(i-1) )>0
        rectangle(ax4,"Position",[0 0 0.2 batt_energy_parallel(i)./batt_energy_parallel(4192).*100],"FaceColor","Green","EdgeColor","white")
    else
        rectangle(ax4,"Position",[0 0 0.2 batt_energy_parallel(i)./batt_energy_parallel(4192).*100],"FaceColor","red","EdgeColor","white")
    end
    %SoC Animation for series
    if  (batt_energy_series(i) - batt_energy_series(i-1))>0
        rectangle(ax2,"Position",[0 0 0.2 batt_energy_series(i)./batt_energy_series(4192).*100],"FaceColor","Green","EdgeColor","white")
    else
        rectangle(ax2,"Position",[0 0 0.2 batt_energy_series(i)./batt_energy_series(4192).*100],"FaceColor","red","EdgeColor","white")
    end


    hold(ax1,'on')
    p1 = plot(ax1,max(cumsum(dist((timeRange(1):i))))./1600,gpsZ((i)),"rO");
    if (batt_energy_parallel(i)./batt_energy_parallel(4192).*100)<0.05
        p1_1 = plot(ax1,max(cumsum(dist((timeRange(1):i-n))))./1600,gpsZ((i-n)),"b","Marker","square");
        n=n+2;
    else
    p1_1 = plot(ax1,max(cumsum(dist((timeRange(1):i-n+1))))./1600,gpsZ((i-n+1)),"b","Marker","square");
    end
    legend(ax1,"","Series","Parallel","location","southeast")
    p3 = plot(ax3,max(cumsum(dist(timeRange(1):i)))./1600,paren(movmean(reshape(posPowerStore(timeRange),[(length(timeRange)) 1]),20), i-timeRange(1)+1, 1),"rO");
    pause(0.0001)
    % Set the filename for the GIF
    frame = getframe(f1);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);

    % Write the frame to the GIF file
    if i == timeRange(1)
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.00001);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.00001);

    end
end




%%

function [gearRatio] = gearSelector(vel, rpmband, powerband, ratios, upper_lim_eng_spd, lower_lim_eng_spd, eng_powers_func, conversion_constant)

%objective of this function is to find the gear ratio which will get the
%highest poweroutput from the engine
global gear_nos;
upper_lim_eng_spd = 5000;
lower_lim_eng_spd = 700;

%gear ratios
if gear_nos==1
    ratios =4;
else
    ratios = [4.0000    7.4000   10.8000   14.2000   17.6000   21.0000];
end


eng_speeds = vel.*ratios.*60./(0.5.*2.*pi);
% eng_speeds((eng_speeds>upper_lim_eng_spd)) = 0;
eng_powers = eng_powers_func(eng_speeds);
eng_powers((eng_speeds>upper_lim_eng_spd | eng_speeds<lower_lim_eng_spd)) = 0;
[~,idx] = max(eng_powers);
gearRatio = ratios(idx);
end


