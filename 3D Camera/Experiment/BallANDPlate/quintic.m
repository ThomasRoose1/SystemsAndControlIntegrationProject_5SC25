function path = quintic(previous_value_reference_start, end_time, reference_end, Ts)
    t0 = 0;                                 %Starting time
    q0 = previous_value_reference_start;    %Starting position
    v0 = 0;                                 %Starting velocity
    ac0 = 0;                                %Starting acceleration
    tf = end_time;                          %End time
    qf = reference_end;                     %End position
    vf = 0;                                 %End velocity
    acf = 0;                                %End acceleration

    M = [1 t0 t0^2 t0^3 t0^4 t0^5;
    0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
    0 0 2 6*t0 12*t0^2 20*t0^3;
    1 tf tf^2 tf^3 tf^4 tf^5;
    0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
    0 0 2 6*tf 12*tf^2 20*tf^3];

    b = [q0; v0; ac0; qf; vf; acf];

    x = M \ b;

    t = t0:Ts:tf;

    path = x(1) + x(2).*t + x(3).*t.^2 + x(4).*t.^3 + +x(5).*t.^4 + x(6).*t.^5;