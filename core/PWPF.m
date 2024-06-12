classdef PWPF
    %PWPF
    
    properties
        Km (1,1) double
        Taum (1,1) double
        Uon (1,1) double
        Uoff (1,1) double
        Um (1,1) double
        Kp (1,1) double
        dt (1,1) double
        Phi (1,1) double
        Gamma (1,1) double
    end
    
    methods
        function obj = PWPF(K_m,tau_m,U_on,U_off,U_m,K_p,dt)
            %PWPF Construct an instance of this class
            obj.Km = K_m;
            obj.Taum = tau_m;
            obj.Uon = U_on;
            obj.Uoff = U_off;
            obj.Um = U_m;
            obj.Kp = K_p;
            obj.dt = dt;

            % Create the discrete time transfer function
            num = K_m;
            den = [tau_m 1];
            [af,bf,~,~] = tf2ss(num,den);
            obj.Phi, obj.Gamma = c2d(af,bf,dt);
        end

        function ufilt = update_lag_filter(obj, ufilt_prev, u_des, u_prev)
            arguments (Input)
                obj PWPF
                ufilt_prev (1,:) double
                u_des (1,:) double
                u_prev (1,:) double
            end
            arguments (Output)
                ufilt (1,:) double
            end
            u_error = obj.Kp*u_des - u_prev;
            ufilt = obj.Phi*ufilt_prev + obj.Gamma*u_error; 
        end

        function u = compute_control_output(obj, ufilt)
            arguments (Input)
                obj PWPF
                ufilt (1,:) double
            end
            arguments (Output)
                u (1,:) double
            end
            u = zeros(size(ufilt));
            for i=1:length(u)
                if abs(ufilt(i)) < obj.Uoff; u(i)=0;end
                if ufilt(i) > obj.Uon, u(i)=obj.Um;end
                if ufilt(i) < -obj.Uon, u(i)=-obj.Um;end
            end
        end

        function [u, ufilt] = run(obj, ufilt_prev, u_des, u_prev)
            arguments (Input)
                obj PWPF
                ufilt_prev (1,:) double
                u_des (1,:) double
                u_prev (1,:) double
            end
            arguments (Output)
                u (1,:) double
                ufilt (1,:) double
            end
            ufilt = obj.update_lag_filter(ufilt_prev,u_des,u_prev);
            u = obj.compute_control_output(ufilt);
        end
    end
end

