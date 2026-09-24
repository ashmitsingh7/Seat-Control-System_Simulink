function build_motor_gearleadscrew_model()
%BUILD_MOTOR_GEAR_LEADSCREW_MODEL
% Phase 3, Milestone A, Step A3: Motor Electrical + Mechanical + Gearbox + Lead Screw
% Subsystems ONLY. No seat mechanics, Stateflow, CAN, diagnostics yet.
%
% This model extends the validated A1/A2 motor-alone subsystem by adding:
%   - 5:1 gearbox with 0.85 efficiency
%   - Lead screw with 4 mm/rev pitch
%
% The A1/A2 motor subsystem remains unchanged and validated.
%
% Equations implemented:
%   Electrical:  L*di/dt = V - i*R - Ke*wm          [A1 - VALIDATED]
%   Mechanical:  J*dwm/dt = Kt*i - T_load - B*wm   [A1/A2 - VALIDATED]
%   Gearbox:     ω_out = ω_m / 5                    [A3]
%                T_out = T_m × 5 × 0.85 (for power flow motor→output) [A3]
%   Lead Screw:  v = (n_out × 4) / 60               [A3]
%                where v = linear velocity [mm/s], n_out = gearbox output [RPM]
%
% T_load here is a raw constant test torque applied directly at the
% motor shaft (no gearbox/lead-screw reflection yet -- that convention
% with the eta*N reflection is introduced only once the gearbox is
% added).

modelName = 'SCS_MotorGearLeadScrew';

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end

new_system(modelName);
open_system(modelName);

%% ---- Parameters (project-defined, Milestone A) ----
R  = 2;        % Ohm
L  = 0.005;    % H
Ke = 0.05;     % V/(rad/s)
Kt = 0.05;     % N*m/A
J  = 0.0005;   % kg*m^2
B  = 0.001;    % N*m*s/rad

% A3 Parameters
gearRatio = 5;         % 5:1
gearEfficiency = 0.85; % 85%
screwPitch = 4;        % mm/rev

assignin('base','R',R);
assignin('base','L',L);
assignin('base','Ke',Ke);
assignin('base','Kt',Kt);
assignin('base','J',J);
assignin('base','B',B);
assignin('base','gearRatio',gearRatio);
assignin('base','gearEfficiency',gearEfficiency);
assignin('base','screwPitch',screwPitch);

%% ---- Sources ----
% Default target: no-load 1000 RPM voltage per corrected sanity check.
add_block('simulink/Sources/Step', [modelName '/V_applied'], ...
    'Time','1','Before','0','After','9.42');

add_block('simulink/Sources/Constant', [modelName '/T_load'], ...
    'Value','0');

fprintf('Sources added\n');

%% ---- A1: Motor Electrical subsystem ----
% Sum_V computes V - i*R - Ke*wm
add_block('simulink/Math Operations/Sum', [modelName '/Sum_V'], ...
    'Inputs','+--');

add_block('simulink/Math Operations/Gain', [modelName '/Gain_1_L'], ...
    'Gain','1/L');

add_block('simulink/Continuous/Integrator', [modelName '/Integrator_i'], ...
    'InitialCondition','0');

add_block('simulink/Math Operations/Gain', [modelName '/Gain_R'], 'Gain','R');
add_block('simulink/Math Operations/Gain', [modelName '/Gain_Ke'], 'Gain','Ke');
add_block('simulink/Math Operations/Gain', [modelName '/Gain_Kt'], 'Gain','Kt');

fprintf('A1 subsystem added\n');

%% ---- A2: Motor Mechanical subsystem ----
% Sum_T computes Kt*i - T_load - B*wm
add_block('simulink/Math Operations/Sum', [modelName '/Sum_T'], ...
    'Inputs','+--');

add_block('simulink/Math Operations/Gain', [modelName '/Gain_1_J'], ...
    'Gain','1/J');

add_block('simulink/Continuous/Integrator', [modelName '/Integrator_w'], ...
    'InitialCondition','0');

add_block('simulink/Math Operations/Gain', [modelName '/Gain_B'], 'Gain','B');

fprintf('A2 subsystem added\n');

%% ---- A3: Gearbox subsystem -%
% Gearbox speed reduction: ω_out = ω_in / ratio
add_block('simulink/Math Operations/Gain', [modelName '/Gain_Gearbox_Speed'], ...
    'Gain','1/gearRatio');

% Gearbox torque multiplication with efficiency: T_out = T_in × ratio × efficiency
add_block('simulink/Math Operations/Gain', [modelName '/Gain_Gearbox_Torque'], ...
    'Gain','gearRatio*gearEfficiency');

fprintf('A3 Gearbox subsystem added\n');

%% ---- A3: Lead screw subsystem ----
% Convert rotational speed to linear velocity: v = (n_out × pitch) / 60
% where v [mm/s], n_out = gearbox output [RPM], pitch [mm/rev]
add_block('simulink/Math Operations/Gain', [modelName '/Gain_LeadScrew'], ...
    'Gain','screwPitch/60');

%% ---- RPM conversion for instrumentation ----
add_block('simulink/Math Operations/Gain', [modelName '/RPM_conv'], ...
    'Gain','60/(2*pi)');
add_block('simulink/Math Operations/Gain', [modelName '/Radps_to_RPM'], ...
    'Gain','30/pi');  % rad/s to RPM: multiply by 30/π

fprintf('A3 Lead screw subsystem added\n');

%% ---- Logging (To Workspace, structure with time) ----
signals = {'V_applied','i','di_dt','e_back','Tm','wm','RPM','T_load', ...
           'gearbox_omega_out','gearbox_torque_out','lead_screw_linear_velocity'};
fprintf('Creating %d logging blocks...\n', numel(signals));
for k = 1:numel(signals)
    signalName = signals{k};
    blockName = [modelName '/' signalName '_log'];
    fprintf('  Creating block #%d: %s (variable: %s)\n', k, blockName, signalName);
    try
        add_block('simulink/Sinks/To Workspace', blockName, ...
            'VariableName', signalName, 'SaveFormat','Structure With Time');
        fprintf('    SUCCESS: Created %s\n', blockName);
    catch ME
        fprintf('    FAILED to create %s: %s\n', blockName, ME.message);
    end
end

fprintf('Logging blocks added\n');

%% ---- Wiring ----
try
    fprintf('Starting wiring...\n');

    % Electrical subsystem
    add_line(modelName, 'V_applied/1', 'Sum_V/1');
    add_line(modelName, 'Gain_R/1',    'Sum_V/2');
    add_line(modelName, 'Gain_Ke/1',   'Sum_V/3');

    add_line(modelName, 'Sum_V/1',       'Gain_1_L/1');
    add_line(modelName, 'Gain_1_L/1',    'Integrator_i/1');
    add_line(modelName, 'Gain_1_L/1',    'di_dt_log/1');

    add_line(modelName, 'Integrator_i/1', 'Gain_R/1');
    add_line(modelName, 'Integrator_i/1', 'Gain_Kt/1');
    add_line(modelName, 'Integrator_i/1', 'i_log/1');

    fprintf('Electrical subsystem wired\n');

    % Mechanical subsystem
    add_line(modelName, 'Gain_Kt/1', 'Sum_T/1');
    add_line(modelName, 'T_load/1',  'Sum_T/2');
    add_line(modelName, 'Gain_B/1',  'Sum_T/3');

    add_line(modelName, 'Sum_T/1',    'Gain_1_J/1');
    add_line(modelName, 'Gain_1_J/1', 'Integrator_w/1');

    fprintf('Mechanical subsystem wired\n');

    % Sensor outputs from motor
    add_line(modelName, 'Integrator_w/1', 'Gain_Ke/1');     % Back EMF
    add_line(modelName, 'Integrator_w/1', 'Gain_B/1');      % Damping torque
    add_line(modelName, 'Integrator_w/1', 'RPM_conv/1');    % Motor RPM
    add_line(modelName, 'Integrator_w/1', 'wm_log/1');      % Motor angular velocity

    fprintf('Motor sensor outputs wired\n');

    % Gearbox connections
    fprintf('About to wire gearbox connections...\n');
    add_line(modelName, 'Integrator_w/1', 'Gain_Gearbox_Speed/1');    % ω_in to gearbox
    fprintf('  ω_in connection successful\n');
    add_line(modelName, 'Gain_Gearbox_Speed/1', 'gearbox_omega_out_log/1');   % ω_out TO LOGGING
    fprintf('  ω_out connection successful\n');
    add_line(modelName, 'Gain_Kt/1', 'Gain_Gearbox_Torque/1');        % T_m to gearbox torque input
    fprintf('  T_in connection successful\n');
    add_line(modelName, 'Gain_Gearbox_Torque/1', 'gearbox_torque_out_log/1'); % T_out TO LOGGING
    fprintf('  T_out connection successful\n');

    fprintf('Gearbox connections wired\n');

    % Lead screw connections - Tap OFF the signals BEFORE they go to logging
    fprintf('About to wire lead screw connections...\n');
    % We need to tap the ω_out signal BEFORE it goes to gearbox_omega_out_log
    add_line(modelName, 'Gain_Gearbox_Speed/1', 'Radps_to_RPM/1');  % ω_out [rad/s] to RPM conversion
    fprintf('  ω_out to Radps_to_RPM connection successful\n');
    add_line(modelName, 'Radps_to_RPM/1', 'Gain_LeadScrew/1');        % n_out [RPM] to lead screw
    fprintf('  Radps_to_RPM to Gain_LeadScrew connection successful\n');
    add_line(modelName, 'Gain_LeadScrew/1', 'lead_screw_linear_velocity_log/1');  % v [mm/s] TO LOGGING
    fprintf('  Lead screw output to logging connection successful\n');

    fprintf('Lead screw connections wired\n');

    % Additional logging
    add_line(modelName, 'Integrator_w/1', 'Tm_log/1');      % Motor torque
    add_line(modelName, 'V_applied/1','V_applied_log/1');
    add_line(modelName, 'Gain_Ke/1',  'e_back_log/1');
    add_line(modelName, 'T_load/1',   'T_load_log/1');

    fprintf('All wiring completed\n');

catch ME
    fprintf('Wiring failed at some point: %s\n', ME.message);
    fprintf('Rethrowing error...\n');
    rethrow(ME);
end

%% ---- Layout ----
try
    Simulink.BlockDiagram.arrangeSystem(modelName);
catch
    % arrangeSystem is best-effort; skip silently if unavailable
end

%% ---- Solver config ----
% tau_e = L/R = 2.5 ms -> Ts = 0.5 ms gives 5 samples/time-constant.
set_param(modelName, 'SolverType', 'Fixed-step');
set_param(modelName, 'Solver', 'ode4');
set_param(modelName, 'FixedStep', '0.0005');
set_param(modelName, 'StopTime', '80');  % Increased to capture ~75s travel time

save_system(modelName);
fprintf('Built and saved %s.slx\n', modelName);

end