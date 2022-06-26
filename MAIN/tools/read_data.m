function [data, timestep, sensors] = read_data(camera_tf, filename1, filename2, N)

    input = load(filename1);
    sensors = load(filename2);

    for q = 2:size(sensors.array(:, 1))-1
        if ~isnan(sensors.array(q, 2)) && ~isnan(sensors.array(q+1, 2)) && ~isnan(sensors.array(q-1, 2))         
            if sensors.array(q, 1) ~= sensors.array(q+1, 1) && sensors.array(q, 1) ~= sensors.array(q-1, 1)
                if sensors.array(q, 2) ~= sensors.array(q+1, 2) && sensors.array(q, 2) ~= sensors.array(q-1, 2)
                    sensors.array(q, 2) = NaN;
                    sensors.array(q, 3) = NaN;
                    sensors.array(q, 4) = NaN;
                end
            end
        end
    end

    for q = 1:size(sensors.array(:, 1))
        if sensors.array(q, 2) > N-1
            sensors.array(q, 2) = NaN;
            sensors.array(q, 3) = NaN;
            sensors.array(q, 4) = NaN;
        end
    end

    data = struct;
    data.timestep.sensor = struct('time', cell(1,1), 'id', cell(1,1), 'range', cell(1,1), 'bearing', cell(1,1));
    data.timestep.odometry = struct('time', cell(1,1), 'x', cell(1,1), 'y', cell(1,1), 'theta', cell(1,1));
    data.timestep.real = struct('time', cell(1,1), 'x', cell(1,1), 'y', cell(1,1), 'theta', cell(1,1));
    
    first = 1;

    timestep.sensor_struct = struct;

    i = 1;
    j = 1;
    k = 1;

    while k < size(sensors.array,1)
        time = sensors.array(k,1);
        time2 = sensors.array(k+1,1);

        timestep(i).sensor_struct(j).time = time;
        timestep(i).sensor_struct(j).id = sensors.array(k,2) + 1;
        timestep(i).sensor_struct(j).range = sqrt(sensors.array(k,3)^2 + (sensors.array(k,4) + camera_tf)^2);
        timestep(i).sensor_struct(j).bearing = wrapToPi(atan2(-sensors.array(k,3), (sensors.array(k,4) + camera_tf)));

        if time ~= time2
            i = i+1;
            j = 1;
        else
            j = j+1;
        end
        k = k+1;
    end

    odom = struct;
    real = struct;
    sensor = struct;
    
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

    for i = 2:size(input.Odometria)-100
        if(first == 0)
            data.timestep(i-1).odometry = odom;
            odom = struct;
        end

        first = 0;
        time = timestep(j).sensor_struct.time;

        odom.time = input.Odometria(i, 1);
        dx = input.Odometria(i, 2) - input.Odometria(i-1, 2);
        dy = input.Odometria(i, 3) - input.Odometria(i-1, 3);

        odom.r1  = wrapToPi(atan2(dy, dx) - input.Odometria(i-1, 4));
        odom.t = sqrt(dx^2 + dy^2);
        odom.r2 = wrapToPi(input.Odometria(i, 4) - atan2(dy, dx));

        odom.x = input.Odometria(i-1, 2)+0.5;
        odom.y = input.Odometria(i-1, 3)+0.5;
        odom.theta = input.Odometria(i-1, 4);

        if abs(odom.time - time) < 0.0224
            data.timestep(i).sensor = timestep(j).sensor_struct;
            if j < size(timestep, 2)
                j = j + 1;
            end
        else
            data.timestep(i).sensor.time = timestep(j).sensor_struct.time;
            data.timestep(i).sensor.id = NaN;
            data.timestep(i).sensor.range = NaN;
            data.timestep(i).sensor.bearing = NaN;
        end
    end

    data.timestep(i).odometry = odom;

    first = 1;

    for i = 2:size(input.Odometria)-100
        if(first == 0)
            data.timestep(i-1).real = real;
            real = struct;
        end
        first = 0;
        real.x = input.Real(i, 2);
        real.y = input.Real(i, 3);
        real.theta = input.Real(i, 4);
        real.time = input.Real(i,1);
    end
    
    data.timestep(i).real = real;

end
