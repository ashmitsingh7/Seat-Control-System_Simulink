function results = run_motor_alone_tests()
%RUN_MOTOR_ALONE_TESTS
% Executes the four Milestone A motor-alone acceptance tests:
%   Test 1 - Zero voltage            -> i -> 0, w -> 0
%   Test 2 - Small voltage step      -> di/dt matches V/L
%   Test 3 - No-load 1000 RPM        -> w ~104.7 rad/s, i ~2.09 A @ V~9.42
%   Test 4 - Load response           -> w down, i up under constant load
%
% Run build_motor_alone_model.m first.

modelName = 'SCS_MotorAlone';
loadedHere = false;
if ~bdIsLoaded(modelName)
    if exist([modelName '.slx'], 'file')
        load_system(modelName);
        loadedHere = true;
    else
        error('Model not found. Run build_motor_alone_model.m first.');
    end
end

%% Parameters required by the model Gain blocks.
% matlab -batch starts a fresh MATLAB process, so these must be restored
% before sim() evaluates the block parameters saved as R, L, Ke, Kt, J, B.
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

results = struct();

%% Test 1 - Zero voltage
	set_param([modelName '/V_applied'], 'Before','0','After','0');
	set_param([modelName '/T_load'], 'Value','0');
	set_param(modelName, 'StopTime','1');
	out1 = sim(modelName);
	results.test1.i_final = out1.i.signals.values(end);
	results.test1.w_final = out1.wm.signals.values(end);
	results.test1.pass = abs(results.test1.i_final) < 1e-9 && abs(results.test1.w_final) < 1e-9;
	fprintf('Test 1 (Zero V): i_final=%.4f A (expect ~0), w_final=%.4f rad/s (expect ~0) -> %s\n', ...
	    results.test1.i_final, results.test1.w_final, passfail(results.test1.pass));

%% Test 2 - Small voltage step, check INSTANTANEOUS initial di/dt
	set_param([modelName '/V_applied'], 'Time','0','Before','0','After','1');
	set_param([modelName '/T_load'], 'Value','0');
	set_param(modelName, 'StopTime','0.01');
	out2 = sim(modelName);

	t2 = out2.di_dt.time;
	di_dt2 = out2.di_dt.signals.values;
	i2 = out2.i.signals.values;

	% Find exact t=0 point
	idx0 = find(t2 == 0, 1, 'first');
	if isempty(idx0)
	    idx0 = 1;  % fallback to first point
	end

	% Diagnostic finite difference (NOT for acceptance)
	idx_fd = 1;
	didt_fd = (i2(idx_fd+1) - i2(idx_fd)) / ...
	    (out2.i.time(idx_fd+1) - out2.i.time(idx_fd));

	% Instantaneous derivative at t=0 (THIS IS THE ACCEPTANCE CRITERION)
	didt_initial = di_dt2(idx0);
	didt_expected = (1 - R*0 - Ke*0)/L;  % = 200 A/s

	results.test2.didt_initial = didt_initial;
	results.test2.didt_expected = didt_expected;
	results.test2.didt_finite_difference = didt_fd;
	results.test2.pass = abs(didt_initial - didt_expected) < 1e-9;
	fprintf('Test 2 (di/dt check): initial=%.2f A/s, expected=%.2f A/s, fd=%.2f A/s -> %s\n', ...
	    didt_initial, didt_expected, didt_fd, passfail(results.test2.pass));

%% Test 3 - No-load 1000 RPM
	set_param([modelName '/V_applied'], 'Before','0','After','9.42');
	set_param([modelName '/T_load'], 'Value','0');
	set_param(modelName, 'StopTime','2');
	out3 = sim(modelName);
	results.test3.w_final   = out3.wm.signals.values(end);
	results.test3.rpm_final = out3.RPM.signals.values(end);
	results.test3.i_final   = out3.i.signals.values(end);
	results.test3.pass = abs(results.test3.w_final - 104.7) < 0.1 && abs(results.test3.i_final - 2.09) < 0.1;
	fprintf(['Test 3 (No-load 1000 RPM): w=%.2f rad/s (expect ~104.7), ' ...
	    'RPM=%.1f (expect ~1000), i=%.3f A (expect ~2.09) -> %s\n'], ...
	    results.test3.w_final, results.test3.rpm_final, results.test3.i_final, passfail(results.test3.pass));

%% Test 4 - Load response
	set_param([modelName '/V_applied'], 'Before','0','After','9.42');
	set_param([modelName '/T_load'], 'Value','0.02'); % N*m step load
	set_param(modelName, 'StopTime','2');
	out4 = sim(modelName);
	results.test4.w_final = out4.wm.signals.values(end);
	results.test4.i_final = out4.i.signals.values(end);
	% Loaded speed should be less than no-load, loaded current should be greater than no-load
	results.test4.pass = results.test4.w_final < results.test3.w_final && results.test4.i_final > results.test3.i_final;
	fprintf('Test 4 (Loaded): w=%.2f rad/s (expect < no-load), i=%.3f A (expect > no-load) -> %s\n', ...
	    results.test4.w_final, results.test4.i_final, passfail(results.test4.pass));

%% Plots
	figure('Name','Motor-Alone Milestone A Tests');
	subplot(2,2,1);
	plot(out3.i.time, out3.i.signals.values); grid on;
	xlabel('t [s]'); ylabel('i [A]'); title('Test 3: Current, no-load 1000 RPM target');

	subplot(2,2,2);
	plot(out3.RPM.time, out3.RPM.signals.values); grid on;
	xlabel('t [s]'); ylabel('RPM'); title('Test 3: Speed, no-load 1000 RPM target');

	subplot(2,2,3);
	plot(out4.i.time, out4.i.signals.values, 'r'); grid on;
	xlabel('t [s]'); ylabel('i [A]'); title('Test 4: Current under load');

	subplot(2,2,4);
	plot(out4.wm.time, out4.wm.signals.values, 'r'); grid on;
	xlabel('t [s]'); ylabel('\omega_m [rad/s]'); title('Test 4: Speed under load');

	save('motor_alone_test_results.mat','results');
	fprintf('\nResults saved to motor_alone_test_results.mat\n');

	if loadedHere
	    close_system(modelName, 0);
	end

end

function text = passfail(tf)
	if tf
	    text = 'PASS';
	else
	    text = 'FAIL';
	end
	end