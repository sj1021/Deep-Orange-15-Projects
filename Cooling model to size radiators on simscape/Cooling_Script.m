% Cooling model script 
clc
clear 
close all

load HeatLoadsFull.mat

%%Initial Values 

coolant_T_init = 303; %Kelvin
coolant_p_init = 101325; %Pascal



%% Engine Radiator

coolant_channel_engine = 0.02; %meter
engine_hydraulic_channel = 0.005;

%  Radiator tubes
n_tubes      = 36;                % Number of tubes
w_radiator  = 2.6*25.4;           % Radiator width [m]
h_tube        = 0.0015;         % Tube height [m]
L_tube        = 46*25.4./1000;             % Tube length [m]

% Radiator fins
delta_fin     = 0.81e-4;           % Fin thickness [m]
w_fin           = 0.00055;        %Unknwn  % Fin width [m]
h_fin           = 0.009;            % Fin height

% Radiator fins
n_fin_rows  = n_tubes - 1;   % Number of fin rows
n_fins         = n_fin_rows  * (L_tube / w_fin);   % Total number of fins

% Heat transfer surface area
area_primary = 2 * L_tube * w_radiator * (n_fin_rows) - 2 * delta_fin * w_radiator * n_fins + 2 * h_fin * w_radiator * n_fin_rows + 2 * h_tube * n_tubes * L_tube;
area_fins        = n_fins * (h_fin-delta_fin) * w_radiator * 2;

% Air
area_air = n_fins * h_fin * w_fin;      % Cross-sectional area [m^2]
fan_dia = 0.380  %Fan diameter  [m] 
fan_qty = 2;
area_air_fan = (fan_qty*pi.*fan_dia^2)./4'%Cross sectional area [m^2]

% Fluid jacket
Re_TLU   = [6, 8, 12, 14, 16]' .* 1e4;  % Reynolds number vector
Pr_TLU   = [2, 4, 6, 8, 10];                  % Prandtl number vector

% Fluid jacket
Nu_TLU  = 0.3.* Re_TLU .^ 0.8 * Pr_TLU .* 0.33;  %  Nusselt number table

% Air
rho_air          = 1.2;       % Density [kg/m^3]
cp_air            = 1004;   % Specific heat [J/kg/K]
vel_air_nom = 1;         % Nominal velocity [m/s]
HC_nom       = 100;     % Nominal heat transfer coefficient [W/(K*m^2)]
HC_min        = 10;       % Minimum heat transfer coefficient [W/(K*m^2)]

%% Inverter Radiator

coolant_channel_engine = 0.02; %meter
engine_hydraulic_channel = 0.005;

%  Radiator tubes
n_tubes_inverter      = 36;                % Number of tubes
w_radiator_inverter  = 2.6*25.4;           % Radiator width [m]
h_tube_inverter        = 0.0015;         % Tube height [m]
L_tube_inverter        = 46*25.4./1000;             % Tube length [m]

% Radiator fins
delta_fin_inverter     = 1e-4;           % Fin thickness [m]
w_fin_inverter           = 0.81e-4;        %Unknwn  % Fin width [m]
h_fin_inverter           = 0.009;            % Fin height

% Radiator fins
n_fin_rows_inverter  = n_tubes_inverter - 1;   % Number of fin rows
n_fins_inverter         = n_fin_rows_inverter  * (L_tube / w_fin);   % Total number of fins

% Heat transfer surface area
area_primary_inverter = 2 * L_tube_inverter * w_radiator_inverter * (n_fin_rows_inverter) - 2 * delta_fin_inverter * w_radiator_inverter * n_fins_inverter + 2 * h_fin_inverter * w_radiator_inverter * n_fin_rows_inverter + 2 * h_tube_inverter * n_tubes_inverter * L_tube_inverter;
area_fins_inverter        = n_fins_inverter * (h_fin_inverter-delta_fin_inverter) * w_radiator_inverter * 2;

% Air
area_air_inverter = n_fins_inverter * h_fin_inverter * w_fin_inverter;      % Cross-sectional area [m^2]
fan_dia_inverter = 0.38  %Fan diameter  [m] 
fan_qty_inverter = 2
area_air_fan_inverter = (fan_qty_inverter*pi.*fan_dia_inverter^2)./4'%Cross sectional area [m^2]

% Fluid jacket
Re_TLU_inverter   = [6, 8, 12, 14, 16]' .* 1e4;  % Reynolds number vector
Pr_TLU_inverter   = [2, 4, 6, 8, 10];                  % Prandtl number vector

% Fluid jacket
Nu_TLU_inverter  = 0.3.* Re_TLU_inverter .^ 0.8 * Pr_TLU_inverter .* 0.33;  %  Nusselt number table

% Air
rho_air          = 1.2;       % Density [kg/m^3]
cp_air            = 1004;   % Specific heat [J/kg/K]
vel_air_nom = 1;         % Nominal velocity [m/s]
HC_nom       = 100;     % Nominal heat transfer coefficient [W/(K*m^2)]
HC_min        = 10;       % Minimum heat transfer coefficient [W/(K*m^2)]


%% motor Radiator

coolant_channel_engine = 0.02; %meter
engine_hydraulic_channel = 0.005;

%  Radiator tubes
n_tubes_motor      = 36;                % Number of tubes
w_radiator_motor  = 2.6*25.4;           % Radiator width [m]
h_tube_motor        = 0.0015;         % Tube height [m]
L_tube_motor        = 46*25.4./1000;             % Tube length [m]

% Radiator fins
delta_fin_motor     = 1e-4;           % Fin thickness [m]
w_fin_motor           = 0.81e-4;        %Unknwn  % Fin width [m]
h_fin_motor           = 0.009;            % Fin height

% Radiator fins
n_fin_rows_motor  = n_tubes_motor - 1;   % Number of fin rows
n_fins_motor         = n_fin_rows_motor  * (L_tube / w_fin);   % Total number of fins

% Heat transfer surface area
area_primary_motor = 2 * L_tube_motor * w_radiator_motor * (n_fin_rows_motor) - 2 * delta_fin_motor * w_radiator_motor * n_fins_motor + 2 * h_fin_motor * w_radiator_motor * n_fin_rows_motor + 2 * h_tube_motor * n_tubes_motor * L_tube_motor;
area_fins_motor        = n_fins_motor * (h_fin_motor-delta_fin_motor) * w_radiator_motor * 2;

% Air
area_air_motor = n_fins_motor * h_fin_motor * w_fin_motor;      % Cross-sectional area [m^2]
fan_dia_motor = 0.38  %Fan diameter  [m] 
fan_qty_motor = 2
area_air_fan_motor = fan_qty_motor*pi.*fan_dia_motor^2./4'%Cross sectional area [m^2]

% Fluid jacket
Re_TLU_motor   = [6, 8, 12, 14, 16]' .* 1e4;  % Reynolds number vector
Pr_TLU_motor   = [2, 4, 6, 8, 10];                  % Prandtl number vector

% Fluid jacket
Nu_TLU_motor  = 0.3.* Re_TLU_motor .^ 0.8 * Pr_TLU_motor .* 0.33;  %  Nusselt number table

% Air
rho_air          = 1.2;       % Density [kg/m^3]
cp_air            = 1004;   % Specific heat [J/kg/K]
vel_air_nom = 1;         % Nominal velocity [m/s]
HC_nom       = 100;     % Nominal heat transfer coefficient [W/(K*m^2)]
HC_min        = 10;       % Minimum heat transfer coefficient [W/(K*m^2)]
