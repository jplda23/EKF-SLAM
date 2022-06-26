function [data] = read_data_sim(filename1)

    input = load(filename1);

    data = struct;
    data.timestep.sensor = struct('time', cell(1,1), 'id', cell(1,1), 'range', cell(1,1), 'bearing', cell(1,1));
    data.timestep.odometry = struct('time', cell(1,1), 'x', cell(1,1), 'y', cell(1,1), 'theta', cell(1,1));
    data.timestep.real = struct('time', cell(1,1), 'x', cell(1,1), 'y', cell(1,1), 'theta', cell(1,1));
    
    first = 1;


    odom = struct;
    real = struct;
    
    data.timestep(1).odometry.time = 0;
    data.timestep(1).odometry.r1 = 0;
    data.timestep(1).odometry.t = 0;
    data.timestep(1).odometry.r2 = 0;
    data.timestep(1).odometry.x = 0.5;
    data.timestep(1).odometry.y = 0.5;
    data.timestep(1).odometry.theta = 0;

    data.timestep(1).real.time = 0;
    data.timestep(1).real.x = 0.5;
    data.timestep(1).real.y = 0.5;
    data.timestep(1).real.theta = 0;

    j = 1;

    for i = 2:size(input.Odometria)
        if(first == 0)
            data.timestep(i-1).odometry = odom;
            odom = struct;
        end

        first = 0;

        odom.time = input.Odometria(i, 1);
        dx = input.Odometria(i, 2) - input.Odometria(i-1, 2);
        dy = input.Odometria(i, 3) - input.Odometria(i-1, 3);

        odom.r1  = wrapToPi(atan2(dy, dx) - input.Odometria(i-1, 4));
        odom.t = sqrt(dx^2 + dy^2);
        odom.r2 = wrapToPi(input.Odometria(i, 4) - atan2(dy, dx));

        odom.x = input.Odometria(i-1, 2);
        odom.y = input.Odometria(i-1, 3);
        odom.theta = input.Odometria(i-1, 4);
        
    end

    data.timestep(i).odometry = odom;

    first = 1;

    for i = 2:size(input.Odometria)
        if(first == 0)
            data.timestep(i-1).real = real;
            real = struct;
        end
        first = 0;
        real.time = input.Real(i,1);
        real.x = input.Real(i, 2);
        real.y = input.Real(i, 3);
        real.theta = input.Real(i, 4);
    end
    
    data.timestep(i).real = real;
    data.timestep = data.timestep(2:end);


end
