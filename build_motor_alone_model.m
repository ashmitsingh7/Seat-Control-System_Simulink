function build_motor_alone_model()
%BUILD_MOTOR_ALONE_MODEL
% Phase 3, Milestone A, Steps A1-A2: Motor Electrical + Motor Mechanical
% subsystems ONLY. No gearbox, lead screw, seat mechanics, friction, or
% limits yet -- per the incremental build rule, this is the first
% debugging boundary and must pass its acceptance tests before anything
% else is added.
%
% Equations implemented (frozen conventions from Phase 3 corrections):
%   Electrical:  L*di/dt = V - i*R - Ke*wm
%   Mechanical:  J*dwm/dt = Kt*i - T_load - B*wm
%
% T_load here is a raw constant test torque applied directly at the
% motor shaft (no gearbox/lead-screw reflection yet -- that convention
% with the eta*N reflection is introduced only once the gearbox is
% added in the next build step).

modelName = 'SCS_MotorAlone';

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

assignin('base','R',R);
assignin('base','L',L);
assignin('base','Ke',Ke);
assignin('base','Kt',Kt);
assignin('base','J',J);
assignin('base','B',B);

%% ---- Sources ----
% Default target: no-load 1000 RPM voltage per corrected sanity check.
add_block('simulink/Sources/Step', [modelName '/V_applied'], ...
    'Time','1','Before','0','After','9.42');

add_block('simulink/Sources/Constant', [modelName '/T_load'], ...
    'Value','0');

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

%% ---- A2: Motor Mechanical subsystem ----
% Sum_T computes Kt*i - T_load - B*wm
add_block('simulink/Math Operations/Sum', [modelName '/Sum_T'], ...
    'Inputs','+--');

add_block('simulink/Math Operations/Gain', [modelName '/Gain_1_J'], ...
    'Gain','1/J');

add_block('simulink/Continuous/Integrator', [modelName '/Integrator_w'], ...
    'InitialCondition','0');

add_block('simulink/Math Operations/Gain', [modelName '/Gain_B'], 'Gain','B');

%% ---- RPM conversion for instrumentation ----
add_block('simulink/Math Operations/Gain', [modelName '/RPM_conv'], ...
    'Gain','60/(2*pi)');

%% ---- Logging (To Workspace, structure with time) ----
sigs = {'V_applied','i','di_dt','e_back','Tm','wm','RPM','T_load'};
for k = 1:numel(sigs)
    add_block('simulink/Sinks/To Workspace', [modelName '/' sigs{k} '_log'], ...
        'VariableName', sigs{k}, 'SaveFormat','Structure With Time');
end

%% ---- Wiring ----
add_line(modelName, 'V_applied/1', 'Sum_V/1');
add_line(modelName, 'Gain_R/1',    'Sum_V/2');
add_line(modelName, 'Gain_Ke/1',   'Sum_V/3');

add_line(modelName, 'Sum_V/1',       'Gain_1_L/1');
add_line(modelName, 'Gain_1_L/1',    'Integrator_i/1');
add_line(modelName, 'Gain_1_L/1',    'di_dt_log/1');

add_line(modelName, 'Integrator_i/1', 'Gain_R/1');
add_line(modelName, 'Integrator_i/1', 'Gain_Kt/1');
add_line(modelName, 'Integrator_i/1', 'i_log/1');

add_line(modelName, 'Gain_Kt/1', 'Sum_T/1');
add_line(modelName, 'T_load/1',  'Sum_T/2');
add_line(modelName, 'Gain_B/1',  'Sum_T/3');

add_line(modelName, 'Sum_T/1',    'Gain_1_J/1');
add_line(modelName, 'Gain_1_J/1', 'Integrator_w/1');

add_line(modelName, 'Integrator_w/1', 'Gain_Ke/1');
add_line(modelName, 'Integrator_w/1', 'Gain_B/1');
add_line(modelName, 'Integrator_w/1', 'RPM_conv/1');
add_line(modelName, 'Integrator_w/1', 'wm_log/1');

add_line(modelName, 'RPM_conv/1', 'RPM_log/1');
add_line(modelName, 'Gain_Kt/1',  'Tm_log/1');
add_line(modelName, 'V_applied/1','V_applied_log/1');
add_line(modelName, 'Gain_Ke/1',  'e_back_log/1');
add_line(modelName, 'T_load/1',   'T_load_log/1');

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
set_param(modelName, 'StopTime', '2');

save_system(modelName);
fprintf('Built and saved %s.slx\n', modelName);

end
