function data = read_data(filename1, filename2)
    % Reads the odometry and sensor readings from a file.
    %
    % filename: path to the file to parse
    % data: structure containing the parsed information
    %
    % The data is returned in a structure where the u_t and z_t are stored
    % within a single entry. A z_t can contain observations of multiple
    % landmarks.
    %
    % Usage:
    % - access the readings for timestep i:
    %   data.timestep(i)
    %   this returns a structure containing the odometry reading and all
    %   landmark obsevations, which can be accessed as follows
    % - odometry reading at timestep i:
    %   data.timestep(i).odometry
    % - senor reading at timestep i:
    %   data.timestep(i).sensor
    %
    % Odometry readings have the following fields:
    % - r1 : rotation 1
    % - t  : translation
    % - r2 : rotation 2
    % which correspond to the identically labeled variables in the motion
    % mode.
    %
    % Sensor readings can again be indexed and each of the entris has the
    % following fields:
    % - id      : id of the observed landmark
    % - range   : measured range to the landmark
    % - bearing : measured angle to the landmark (you can ignore this)
    %
    % Examples:
    % - Translational component of the odometry reading at timestep 10
    %   data.timestep(10).odometry.t
    % - Measured range to the second landmark observed at timestep 4
    %   data.timestep(4).sensor(2).range
    input = load(filename1);
    sensors = load(filename2);

    data = struct;
    data.timestep.sensor = struct('time', cell(1,0), 'id', cell(1,0), 'range', cell(1,0), 'bearing', cell(1,0));
    first = 1;

    odom = struct;
    real = struct;
    sensor = struct;

    j = 1;

    for i = 1:size(input.Odometria)-1
        if(first == 0)
            data.timestep(end+1).odometry = odom;
            data.timestep(i).sensor = reading;
            %data.timestep(end).real = real;
            %data.timestep(end).sensor = sensor;
            odom = struct;
            reading = struct;
            %sensor = struct('id',cell(1,0),'range',cell(1,0),'bearing',cell(1,0));
        end
        first = 0;
        reading.time = sensors.array(j, 1)*10^-9;
        odom.time = input.Odometria(i, 1);
        dx = input.Odometria(i+1, 2) - input.Odometria(i, 2);
        dy = input.Odometria(i+1, 3) - input.Odometria(i, 3);
        odom.r1  = wrapToPi(atan2(dy, dx) - input.Odometria(i, 4));
        odom.t = sqrt(dx^2 + dy^2);
        odom.r2 = wrapToPi(input.Odometria(i+1, 4) - atan2(dy, dx));
        odom.x = input.Odometria(i, 2)+0.5;
        odom.y = input.Odometria(i, 3)+0.5;
        odom.theta = input.Odometria(i, 4);

        if abs(odom.time - reading.time) < 0.02
            reading.id = sensors.array(j, 2)+1;
            reading.range = sensors.array(j, 3);
            reading.bearing = sensors.array(j, 4);
            j = j + 1;
        else
            reading.id = NaN;
            reading.range = NaN;
            reading.bearing = NaN;
        end
    end

%     first = 1;
%     j = 1;
%     for i = 1:size(input.Odometria)-1
%         if(first == 0)
%             %data.timestep(end+1).odometry = odom;
%             data.timestep(i+1).sensor = reading;
%             %odom = struct;
%             %sensor = struct('time', cell(1,0), 'id', cell(1,0), 'range', cell(1,0), 'bearing', cell(1,0));
%         end
%         first = 0;
%         reading = struct;
%         reading.time = sensors.array(j, 1);
%         time1 = reading.time;
%         time2 = data.timestep(i).odometry.time;
%         if abs(time2 - time1) < 0.02
%             reading.time = sensors.array(j, 1);
%             reading.id = sensors.array(j, 2)+1;
%             reading.range = sensors.array(j, 3);
%             reading.bearing = sensors.array(j, 4);
%             j = j + 1;
%         else
%             reading.time = 'NaN';
%             reading.id = 'NaN';
%             reading.range = 'NaN';
%             reading.bearing = 'NaN';
%         end
%     end

    first = 1;
    for i = 1:size(input.Odometria)-1
        if(first == 0)
            data.timestep(i).real = real;
            real = struct;
        end
        first = 0;
        real.x = input.Real(i, 2);
        real.y = input.Real(i, 3);
        real.theta = input.Real(i, 4);
    end

    data.timestep = data.timestep(2:end);

%     while true
%         line = fgetl(input);
%         if ~ischar(line); break; end
%         arr = strsplit(line, ' ');
%         type = deblank(arr{1});
% 
%         if(strcmp(type, 'ODOMETRY') == 1)
%             if(first == 0)
%                 data.timestep(end+1).odometry = odom;
%                 data.timestep(end).sensor = sensor;
%                 odom = struct;
%                 sensor = struct('id',cell(1,0),'range',cell(1,0),'bearing',cell(1,0));
%             end
%             first = 0;
%             %odom.r1 = str2double(arr{2});
%             %odom.t  = str2double(arr{3});
%             %odom.r2 = str2double(arr{4});
%             % ADD EXTRA NOISE
%             odom.r1 = str2double(arr{2}) + rand(1)*0.02 - 0.01;
%             odom.t  = str2double(arr{3}) + rand(1)*0.2 - 0.1;
%             odom.r2 = str2double(arr{4}) + rand(1)*0.02 - 0.01;
%         elseif(strcmp(type, 'SENSOR') == 1)
%             reading = struct;
%             reading.id      = str2double(arr{2});
%             reading.range   = str2double(arr{3});
%             reading.bearing = str2double(arr{4});
%             % ADD EXTRA NOISE
%             %reading.range   = str2double(arr{3}) + rand(1)*0.25 - 0.125;
%             %reading.bearing = str2double(arr{4}) + rand(1)*0.2 - 0.1;
%             sensor(end+1) = reading;
%         end
%     end
% 
%     data.timestep = data.timestep(2:end);
% 
%     fclose(input);
end
