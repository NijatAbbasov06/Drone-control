function result = simulate(controllers, desired_state, init_state, tstart, tend, dt)
    % Physical constants
    g = 9.81;
    m = 0.468;
    L = 0.225;
    k = 2.98e-6; % lift/thrust constant
    b = 1.14e-7; % drag constant
    I = diag([4.856e-3, 4.856e-3, 8.801e-3]);
    kd = 0.25;
    
    ts = tstart:dt:tend-dt;

    % Number of points in the simulation.
    N = numel(ts);

    % Output values, recorded as the simulation runs.
    xout = zeros(3, N);
    xdotout = zeros(3, N);
    thetaout = zeros(3, N);
    thetadotout = zeros(3, N);
    inputout = zeros(4, N);

    % Struct given to the controller to run simulations
    controller_params = struct('dt', dt, 'I', I, 'k', k, 'L', L, 'b', b, 'm', m, 'g', g);

    % Initial system state.
    x = init_state.x;
    xdot = init_state.xdot;
    theta = init_state.theta;
    thetadot = init_state.thetadot;

    ind = 0;
    for t = ts
        ind = ind + 1;     

        [rotor_state, controller_params] = resolve_control_signals(controllers, controller_params, desired_state, theta, thetadot, x(3), xdot(3));
        


        % Compute forces, torques, and accelerations
        omega = thetadot2omega(thetadot, theta); %labframe to body frame
        a = acceleration(rotor_state, theta, xdot, m, g, k, kd);
        omegadot = angular_acceleration(rotor_state, omega, I, L, b, k);

        % Progress the system state
        omega = omega + dt * omegadot; %in body frame
        thetadot = omega2thetadot(omega, theta); 
        theta = theta + dt * thetadot;
        xdot = xdot + dt * a;
        x = x + dt * xdot;

        % Store simulation state for output
        xout(:, ind) = x;
        xdotout(:, ind) = xdot;
        thetaout(:, ind) = theta;
        thetadotout(:, ind) = thetadot;
        inputout(:, ind) = rotor_state;
    end

    % Put all simulation variables into an output struct
    result = struct('x', xout, 'theta', thetaout, 'vel', xdotout, ...
                    'angvel', thetadotout, 't', ts, 'dt', dt, 'input', inputout);
end

% ===========================DYNAMICS==================================== %

% Compute Thrust given current inputs (thrust force on each motor) and thrust coefficient
function T = thrust(inputs, k)
    T = [0; 0; k * sum(inputs)];
end

% Compute torques given current inputs, length of the drone arm, drag coefficient and thrust coefficient
function tau = torques(rotor_states, L, b, k)
    tau = [
        L * k * (rotor_states(4) - rotor_states(2)) % roll
        L * k * (rotor_states(3) - rotor_states(1)) % pitch
        b * (rotor_states(4) - rotor_states(3) + rotor_states(2) - rotor_states(1)) % yaw
    ];
end
        
function a = acceleration(rotor_state, theta, xdot, m, g, k, kd)
    gravity = [0; 0; -g];
    R = rotation(theta);
    T = R * thrust(rotor_state, k);
    Fd = -kd * xdot; % aerodynamic drag
    a = gravity + (1 / m) * T + Fd;
end

function omegadot = angular_acceleration(rotor_state, omega, I, L, b, k)
    tau = torques(rotor_state, L, b, k);
    inv_I = diag([1/(4.856e-3), 1/(4.856e-3), 1/(8.801e-3)]);
    omegadot = inv_I * (tau - cross(omega, I * omega));
end

% Convert derivatives of roll, pitch, yaw to omega.
function omega = thetadot2omega(thetadot, angles)
    phi = angles(1);
    theta = angles(2);
    
    W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];

    omega = W * thetadot;
end

% Convert omega to roll, pitch, yaw derivatives
function thetadot = omega2thetadot(omega, theta)
    phi = theta(1);
    theta = theta(2);
    
    inv_W = [
        1, sin(phi)*tan(theta), cos(phi)*tan(theta)
        0, cos(phi), -sin(phi)
        0, -sin(phi)/cos(theta), cos(phi)/cos(theta)
    ];

    thetadot = inv_W * omega;
end

