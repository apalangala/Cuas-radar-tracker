function run_tests()
%RUN_TESTS  Run the unit/integration test suite.  Errors if any test fails
%   (so CI marks the build red).  Works in MATLAB and GNU Octave.
%
%   >> setup_paths; run_tests

tests = {
    'test_radar_params'
    'test_cfar_false_alarm_rate'
    'test_single_target_detection'
    'test_mti_removes_clutter'
    'test_meas_model_jacobian'
    'test_tracker_consistency'
    'test_notch_aware_coasting'
};
n_fail = 0;
fprintf('Running %d tests\n', numel(tests));
for i = 1:numel(tests)
    t0 = tic;
    try
        feval(tests{i});
        fprintf('  PASS  %-32s (%.1f s)\n', tests{i}, toc(t0));
    catch err
        n_fail = n_fail + 1;
        fprintf('  FAIL  %-32s %s\n', tests{i}, err.message);
    end
end
if n_fail > 0
    error('run_tests:failed', '%d of %d tests failed', n_fail, numel(tests));
end
fprintf('All %d tests passed\n', numel(tests));
end
