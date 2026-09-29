function test_meas_model_jacobian()
%TEST_MEAS_MODEL_JACOBIAN  Analytic EKF Jacobian matches finite differences.
states = [ -60 5 420 -15; 120 0 150 20; 30 -12 80 3 ].';
for s = 1:size(states, 2)
    x = states(:, s);
    [~, H] = meas_model(x);
    Hn = zeros(3, 4); h = 1e-6;
    for i = 1:4
        dx = zeros(4, 1); dx(i) = h;
        Hn(:, i) = (meas_model(x + dx) - meas_model(x - dx)) / (2*h);
    end
    err = max(abs(H(:) - Hn(:))) / max(abs(Hn(:)));
    assert(err < 1e-6, sprintf('Jacobian mismatch %.1e at state %d', err, s));
end
end
