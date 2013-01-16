classdef control < handle

    
    properties
        ARc    %control port  (udp)   
        ARcc   %control port (tcp/ip)
        seq    %swq num
        config
    end
    
    methods
        function obj = control(ctrl_prt)
            obj.ARc = ctrl_prt;
            obj.ARcc = tcpip('192.168.1.1', 5559, 'LocalPort', 5559);
            obj.seq = 1; 
            %echotcpip('on', 5559)
            fopen(obj.ARc); 
            %set(obj.ARcc, 'LocalHost', '192.168.1.2');

            
        end
        
        function at_ref(obj, ref_arg)
            for i = 1 : 5
                REF = sprintf('AT*REF=%u,%d\r',obj.seq, ref_arg); % Creating REF command(Take off, landing, reset,etc)
                fprintf(obj.ARc, REF); % Sending REF command
                pause(0.01)   %to ensure successive at_ref receives
            end   
            obj.seq = obj.seq+1;
        end
        
        
        function at_pcmd(obj, progressive, lr, fb, ud, rot) %(left/right, front/back, up/down, rotation)
            
            PCMD = sprintf('AT*PCMD=%u,%d,%d,%d,%d,%d\r',obj.seq, progressive,lr,fb,ud,rot); 
            obj.sendcmd(PCMD);
            obj.seq = obj.seq+1;
        end
        
        function at_ftrim(obj)
            FTRIM = sprintf('AT*PCMD=%d,\r',obj.seq);
            obj.sendcmd(FTRIM);
            obj.seq = obj.seq+1;
        end
        
        function at_comwdg(obj)
            COMWDG = sprintf('AT*COMWDG=1\r');
            obj.sendcmd(COMWDG);

        end
        
        function pause_wdg(obj, pause_time)
            pause(pause_time);
            obj.at_comwdg;          
        end
        
        function at_navconfig(obj)
            
            AR_NAV_CONFIG = sprintf('AT*CONFIG=1,\"general:navdata_demo\",\"FALSE\"\r');
            obj.pause_wdg(0.01);
            obj.sendcmd(AR_NAV_CONFIG);
            %obj.seq = obj.seq+1;
        end
        
        function at_flymode(obj)
            fly_mode = 3;
            AR_FMODE_CONFIG = sprintf('AT*CONFIG=605,\"control:flying_mode\",\"%u\"\r',fly_mode);
            obj.pause_wdg(0.01);
            obj.sendcmd(AR_FMODE_CONFIG);
        end
        
        function at_ctrl(obj)
            AR_CONFIG = sprintf('AT*CTRL=1,4,0\r');
            obj.pause_wdg(0.01);
            obj.sendcmd(AR_CONFIG);
            
        end
        
        function get_config(obj)
            fopen(obj.ARcc);
            fwrite(obj.ARcc, 1);
            obj.at_ctrl;
            obj.config = fread(obj.ARcc);
            fclose(obj.ARcc);            
        end
        
        function sendcmd(obj, cmd_arg)
             for i = 1:3 %send 3 times with 0.02 sec interval to ensure data integrity
                fprintf(obj.ARc, cmd_arg); 
                pause(0.02);
             end            
        end
        
        function reset_port(obj)
            fclose(obj.ARc);
            fopen(obj.ARc);
        end
        
        function terminate(obj)
            fclose(obj.ARcc);
            fclose(obj.ARc);
        end
    end
    
end

