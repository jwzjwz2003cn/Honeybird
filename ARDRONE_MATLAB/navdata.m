classdef navdata < handle
    %NAVDATA Summary of this class goes here
    %   Detailed explanation goes here
    % class provide utility functions to decode the navdata array
    
    properties
        ARcontrol
        ARn
        nav_data
        file_index;
       % t = timer;
    end
    
    methods
        function obj = navdata(nav_prt, ctrl) %constructor
            obj.ARn = nav_prt;
            obj.ARcontrol = ctrl;
            fopen(obj.ARn);
            fwrite(obj.ARn, 1); % convention to write a byte first b4 receiving data
            obj.config_nav();
            obj.file_index = 1;
           % set(obj.t, 'ExecutionMode','FixedRate', 'TimerFcn',@obj.get_navdata,'period',0.01);
           % start(obj.t);
            
        end
        
        function hex_string = fread2hex(obj, matrix ) %reorder convert the 4 bytes matrix into python support string

            byte(1) = matrix(4);
            byte(2)= matrix(3);
            byte(3) = matrix(2);
            byte(4) = matrix(1);   
            hex = cell(1,4);
    
            for i = 1:4
                hex{i} = dec2hex(byte(i),2);  
            end
       
            hex_string = sprintf('%s',hex{:});
        end

        function config_nav(obj)
            obj.ARcontrol.at_navconfig();
            
        end
        
        %function get_navdata(obj, hObject, evt)
        function get_navdata(obj)

            fwrite(obj.ARn, 1);
            obj.ARcontrol.at_navconfig();            
            obj.nav_data = fread(obj.ARn);
            
        end
        
        
        
        function output_navdata(obj)
            fwrite(obj.ARn, 1);
            obj.ARcontrol.at_comwdg();
            obj.nav_data = fread(obj.ARn);
            
            file_name = sprintf('capture%u.bin', obj.file_index);
            fid = fopen(file_name, 'w');
            fwrite(fid, obj.nav_data,'uint8');
            %disp('writing nav_data file');
            obj.file_index = obj.file_index+1;
            fclose(fid);            
            
        end
        
        function reset_port(obj)
            fclose(obj.ARn);
            fopen(obj.ARn);
        end
        
        function battery = decode_battery(obj)
            obj.get_navdata;
            battery = obj.nav_data(25);

        end
        
        function pitch = decode_pitch(obj)
            obj.get_navdata;
            pitch_hex = obj.fread2hex(obj.nav_data(29:32));
            pitch = str2double(python('hex2float.py', pitch_hex))/1000;
        end
        
        function roll = decode_roll(obj)
            obj.get_navdata;
            roll_hex = obj.fread2hex(obj.nav_data(33:36));
            roll = str2double(python('hex2float.py', roll_hex))/1000;
        end
        
        function yaw = decode_yaw(obj)
            obj.get_navdata;
            yaw_hex = obj.fread2hex(obj.nav_data(37:40));
            yaw = str2double(python('hex2float.py', yaw_hex))/1000;
        end
        
        function altitude = decode_altitude(obj)
            obj.get_navdata;
            altitude_hex = obj.fread2hex(obj.nav_data(41:44));
            altitude = str2double(python('hex2int32.py', altitude_hex));
        end
        
        function vx = decode_vx(obj)
            obj.get_navdata;
            vx_hex = obj.fread2hex(obj.nav_data(45:48));
            vx = str2double(python('hex2float.py', vx_hex));
        end
        
        function vy = decode_vy(obj)
            obj.get_navdata;
            vy_hex = obj.fread2hex(obj.nav_data(49:52));
            vy = str2double(python('hex2float.py', vy_hex));
        end
        
        function vz = decode_vz(obj)
            obj.get_navdata;
            vz_hex = obj.fread2hex(obj.nav_data(53:56));
            vz = str2double(python('hex2float.py', vz_hex));          
        end
        
        function alt_vec = calibrate_navdata(obj) %seems not used
            obj.get_navdata;
            i = 1;
            j = 1;
            while(i<=509)
                altitude_hex = obj.fread2hex(obj.nav_data(i:i+3));
                altitude = str2double(python('hex2int32.py', altitude_hex));
                
               % if (altitude > 500 && altitude < 3000)                    
                   % altitude
                   % i
                %end
                i = i+4;
            end
        end
        
        
        
        function terminate(obj)
            fclose(obj.ARn);            
        end
        
    end
    
end

