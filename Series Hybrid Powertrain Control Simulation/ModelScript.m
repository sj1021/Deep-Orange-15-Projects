
clear 
close all
clc

%% !!!!!!!!!!! PLEASE FILL THIS BEFORE YOU START !!!!!!!!!!!!!!!!!!!!!!!!!
%Model Parameters 

% Set the start and end times for the simulation
start_time = 4192; % Set your desired start time here

SerialNo = "10_1";
motorName = "Yasa P400";  % Emrax 268/Yasa P400R/Emrax 348/Omni
generatorName = "Yasa P400"; %Emrax 268/Yasa P400R
engineName = "Cummins Tuned"; %Cummins/Cummins Tuned/Stellantis/Duramax 2.8/Duramax 3.0
batteryCap = "64"; % "Enter the battery Capacity; 

SourceFolder = "Lap time simulator\";
ResultsFolder = "Results\";


vehMass =2512;
ratio_fd = 9.5;

%% Variable Initialization

stringConcat = strcat(SerialNo,"_",engineName,"_",generatorName,"_",motorName,"_",batteryCap,"kWh",".mat");
load(fullfile(SourceFolder,stringConcat));
ratio_fd = ratio_fd.*0.95;
%Vehicle


time = cumsum(timeDelta);
end_time = max(time);  % Set your desired end time here
grade_TS = [time ;[vertGrade 0]]';
fr_TS = [time ; fr_new]';
vehSpeed(end) = 0;
vehSpeed_TS = [time ; vehSpeed]';
vehMaxPower = 400e3; %Power in watts
vehMaxBrakePower = 800e3; %Power in wattsa
vehTireRadius = 0.5; % Tire Radius in meter
 %kg

g = 9.81; %m/s2
Cd = 0.4;
rho_air = 1.225;
vehArea = 3.5; %m2


%Motor
plant.eMot.moi = 0.09;

eff_trans = 0.96;
mass = vehMass;
Af = vehArea;
frr = 0.1;
r_tire = vehTireRadius;
max_tract_force_brake = -20e5;

max_tract_force_acc = 20e5;


simData = sim("VehicleLongitudnalModel.slx",'StartTime', num2str(start_time), 'StopTime', num2str(end_time));
simData.logsout;


%% Data Processing 

%Vehicle Data 
vehicle_speed = getElement(simData.logsout,'CurSpd').Values;
reference_speed = getElement(simData.logsout,'RefSpd').Values;

Vehicle_States = getElement(simData.logsout,'Vehicle_States').Values;
Vehicle_Acceleration = Vehicle_States.Vehicle_Acceleration;
Vehicle_Distance_miles = Vehicle_States.Vehicle_Distance_miles;
Vehicle_Power = Vehicle_States.Vehicle_Power;

Vehicle_DesiredStates =  getElement(simData.logsout,'Vehicle_DesiredStates').Values;
Vehicle_PowerDemand = Vehicle_DesiredStates.veh_PowerDemand;

%Energy Storage Data
Battery_States = getElement(simData.logsout,'Battery_States').Values;
Battery_SOC = Battery_States.Battery_SOC;
Battery_Temp = Battery_States.Battery_Temperature;
Battery_Power_Limit = Battery_States.Battery_Power_Limit;
Battery_Power = getElement(simData.logsout,'BatteryPower');
Battery_Current = Battery_States.Battery_Current;

%Motor Data 
Motor_States = getElement(simData.logsout,'Motor_States').Values;
Motor_Torque_FL = getElement(simData.logsout,'Motor_Torque_FL').Values;
Motor_Torque_FR = getElement(simData.logsout,'Motor_Torque_FR').Values;
Motor_Torque_RL = getElement(simData.logsout,'Motor_Torque_RL').Values;
Motor_Torque_RR = getElement(simData.logsout,'Motor_Torque_RR').Values;

% Motor_Torque_FL = Motor_States.Driveshaft_Torque_FL./ratio_fd;
% Motor_Torque_FR = Motor_States.Driveshaft_Torque_FR./ratio_fd;
% Motor_Torque_RL = Motor_States.Driveshaft_Torque_RL./ratio_fd;
% Motor_Torque_RR = Motor_States.Driveshaft_Torque_RR./ratio_fd;
Motor_w_FL = Vehicle_DesiredStates.FL_RPM.*ratio_fd;
Motor_w_FR = Vehicle_DesiredStates.FR_RPM.*ratio_fd;
Motor_w_RL = Vehicle_DesiredStates.RL_RPM.*ratio_fd;
Motor_w_RR = Vehicle_DesiredStates.RR_RPM.*ratio_fd;
Motor_HeatGenerated = Motor_States.Motor_HeatGenerated;
Motor_Power = Motor_States.Motor_ElectricPowerDraw;


%EnergyConverter Data 

EnergyConverter_States = getElement(simData.logsout,7).Values;
EnergyConverter_Engine_HeatLoad = EnergyConverter_States.EnergyConverter_Engine_HeatLoad;
EnergyConverter_ePower = EnergyConverter_States.EnergyConverter_ePower;
EnergyConverter_FuelConsumed = EnergyConverter_States.EnergyConverter_FuelConsumed;
EnergyConverter_Gen_HeatLoad = EnergyConverter_States.EnergyConverter_Gen_HeatLoad;
EnergyConverter_Power_max = EnergyConverter_States.EnergyConverter_Power_max;
EnergyConverter_Power_max = EnergyConverter_States.EnergyConverter_Power_max;
EnergyConverter_Engine_Torque = EnergyConverter_States.EnergyConverter_Engine_Torque;
EnergyConverter_Engine_CurrentRPM = EnergyConverter_States.EnergyConverter_Engine_CurrentRPM;

EnergyConverter_DesiredStates = getElement(simData.logsout,'EnergyConverter_DesiredStates').Values;

EnergyConverter_Power_Watt = getElement(simData.logsout,'EnergyConverter_Power_Watt').Values;



%% Vehicle Plots 

subText = strcat(SerialNo,". ",engineName,", (G) ",generatorName,", (M) ",motorName," ,",batteryCap,"kWh");

% Road load equations to calculate power 
vertGrade = [vertGrade 0];
acc = diff(vehSpeed);
acc = [acc 0];
rr = fr_new.*mass.*g.*cos(vertGrade).*vehSpeed;
aero =  0.5.*Cd.*Af.*power(vehSpeed,2).*rho_air.*vehSpeed;
grade = mass.*g.*sin(vertGrade).*vehSpeed;
force  = mass.*(acc) + fr_new.*mass.*g.*cos(vertGrade) + 0.5.*Cd.*Af.*power(vehSpeed,2).*rho_air + mass.*g.*sin(vertGrade);
instPower = force.*vehSpeed./1000;
subtitle(subText)

figure
plot(time,instPower);
hold on 
plot(Vehicle_Power.Time,Vehicle_Power.Data./1000)
xlabel("Time [s]")
ylabel("Power [kW] ")
title("Vehicle Power v time");
legend("Road load power calculated from velocity","Simulink Model Power")
xlim([4192 end_time])
grid on
subtitle(subText)

figure
plot(vehicle_speed.time,vehicle_speed.data)
hold on
plot(reference_speed.time,reference_speed.data)
xlabel("Time [s]");
ylabel("Vehicle Speed [m/s]");
title("Vehicle Speed v time");
legend("Vehicle Speed","Reference Speed")
xlim([4192 end_time])
grid on
subtitle(subText)


%% Energy Converter Plots 

%added
%Code goes here

figure
plot(EnergyConverter_ePower.Time,-reshape(EnergyConverter_ePower.Data,[max(size(EnergyConverter_ePower.Data)) 1])./1000)
hold on
plot(EnergyConverter_Power_Watt/1000)
title("Energy Converter Power Response")
xlabel("Time [s]")
ylabel("Power [kW]")
legend("Generator Power Output","Energy Converter Power Demand");
subtitle(subText)

figure()
hold on
contourf(simData.Engine_RPM_Map,simData.Engine_Torque_Map,simData.Engine_Efficiency_Map,[180 200 220 240 260 280 300 320 1000 4000])
scatter(reshape(EnergyConverter_Engine_CurrentRPM.Data,[max(size(EnergyConverter_Engine_Torque.Data)) 1]),reshape(EnergyConverter_Engine_Torque.Data,[max(size(EnergyConverter_Engine_Torque.Data)) 1]),"r");
RPM_max = linspace(simData.Engine_RPM_Map(1),simData.Engine_RPM_Map(end),length(simData.EnergyConvertor_Engine_Torque_Map));
T_max = max(simData.EnergyConvertor_Engine_Torque_Map);
plot(RPM_max,T_max,'LineWidth',2.5,'Color','k')
xlabel("Engine Speed [RPM]")
ylabel("Engine Torque [Nm]")
title(sprintf(strcat("Operating Points of Engine on BSFC Map")))
xlim([min(RPM_max) max(RPM_max)])

subtitle(subText)
 
%Engigine Desired vs Current RPM
figure
plot(simData.tout,simData.EnergyConverter_Engine_desiredRPM,'k')
hold on
plot(simData.tout,simData.EnergyConverter_Engine_CurrentRPM,'r')
xlabel('Time [s]')
ylabel('Desired/Current RPM')
title('Desired vs Current RPM')
legend('Desired RPM','Actual RPM')
grid on
subtitle(subText)

%Engine On Off
figure
plot(simData.tout,simData.EnergyConverter_Engine_OnOff,"k")
title("Engine On-Off States")
xlabel("Time [s]")
ylabel('Engine On/Off')
grid on 
subtitle(subText)
%

%Heat Output 
figure
plot(simData.tout,simData.EnergyConverter_Gen_HeatLoad+(reshape(EnergyConverter_Engine_HeatLoad.Data,[max(size(EnergyConverter_Engine_HeatLoad.Data)) 1])),'r',LineWidth=1.5)
title("Energy Convertor Heat Load")
xlabel("Time [s]")
ylabel('Heat Load (kW)')
grid on 
subtitle(subText)

%Fuel Consumption 
figure
plot (EnergyConverter_FuelConsumed)
title("Fuel Consumption vs time")
xlabel("Time [s]")
ylabel('Fuel Consumption (gallons)')
grid on 
subtitle(subText)
%generator efficiency plots 

figure()
contourf(simData.Gen_RPM.Data',-simData.Gen_Torque.Data',simData.Gen_eff.Data')
hold on
contourf(simData.Gen_RPM.Data',simData.Gen_Torque.Data',simData.Gen_eff.Data')
scatter(simData.Gen_operating_RPM,simData.Gen_operating_Torque,"r");
xlabel('Generator Speed (RPM)')
ylabel('Generator Torque (Nm)')
title('Generator Operating Points')
legend('Gen Efficiency Map','','Operating Points')
subtitle(subText)
%generator heat generated
figure()
plot(simData.tout,simData.EnergyConverter_Gen_HeatLoad/1000)
xlabel('Drivecycle time (sec)')
ylabel('Heat Generated (kW)')
title('Generator Heat Generated Vs Time')
grid on
subtitle(subText)


% Whatever you think will help justify your sub system choice

%% Energy Storage Plots 

%Code goes here
figure()
a = reshape(simData.Battery_SOC,[max(size(simData.Battery_SOC)) 1])
plot(simData.tout,a)
xlabel("Time [s]")
ylabel('SOC')
title('Battery SOC vs time')
subtitle(subText)

figure()
plot(simData.tout,reshape(simData.OCV,[max(size(simData.OCV)) 1]))
xlabel("Time [s]")
ylabel('OCV')
title('Battery OCV vs time')
subtitle(subText)

figure()
plot(simData.tout,reshape(simData.Current,[max(size(simData.Current)) 1]))
xlabel("Time [s]")
ylabel('Current)')
title('Battery Current vs time')
subtitle(subText)


% Whatever you think will help justify your sub system choice

%% Motor Plots

%Code goes here

% Motor Efficiency Map with operating points %

figure()
hold on
contourf(simData.Motor_1_RPM.Data,simData.Motor_1_Tor.Data,simData.Motor_1_eff.Data)
contourf(simData.Motor_1_RPM.Data,-simData.Motor_1_Tor.Data,simData.Motor_1_eff.Data)
scatter(Motor_w_FL.Data,Motor_Torque_FL.Data,"yo")
plot(simData.Motor_Max_Torque_Line,simData.Motor_RPM_Max_Torque_Line,'LineWidth',5,'Color',"r");
plot((simData.Motor_Max_Torque_Line),-simData.Motor_RPM_Max_Torque_Line,'LineWidth',5,'Color',"r");
xlabel('Traction Motor Speed (RPM)')
ylabel('Traction Motor Torque (Nm)')
title('Motor Operating Points')
legend('Motor Efficiency Map','','Operating Points','Max Torque Line','')
subtitle(subText)

% Motor heat generated vs time %

figure()
plot(Motor_HeatGenerated/1000)
xlabel('Drivecycle time (sec)')
ylabel('Heat Generated (kW)')
title('Motor Heat Generated Vs Time')
grid on
subtitle(subText)

% Motor heat generated vs Motor torque %

figure()
plot(Motor_Torque_FL.Data,Motor_HeatGenerated.Data/1000)
xlabel('Traction Motor Torque (Nm)')
ylabel('Motor Heat Generated (kW)')
title('Motor Heat Generated Vs Traction Motor Torque')
grid on
subtitle(subText)

% Motor heat generated vs Motor power %

figure()
plot(Motor_Power.Data/1000,Motor_HeatGenerated.Data/1000)
xlabel('Traction Motor Power (kW)')
ylabel('Motor Heat Generated (kW)')
title('Motor Heat Generated Vs Traction Motor Power')
grid on
subtitle(subText)

% Whatever you think will help justify your sub system choice

%Saving the workspace variable for future reference

save(fullfile(ResultsFolder,stringConcat));