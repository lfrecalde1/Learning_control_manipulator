classdef controller < matlab.mixin.SetGet
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Controller gains
        K1
        K2
        l1_memories
        e1_memories
        l2_memories
        e2_memories
        A
        B
        % Pendulum system
        pendulum
    end
    
    methods
        function obj = controller(K1, K2, num, den, pendulum)
            %UNTITLED3 Construct an instance of this class
            %   Detailed explanation goes here
            obj.K1 = K1;
            obj.K2 = K2;
            obj.pendulum = pendulum;
            
            % Learning parameters
            obj.B = den(2:end);
            obj.A = num(2:end);
            
            % memories values
            
            obj.l1_memories = zeros(length(obj.B), 1);
            obj.e1_memories = zeros(length(obj.A), 1);
            
            obj.l2_memories = zeros(length(obj.B), 1);
            obj.e2_memories = zeros(length(obj.A), 1);
        end
        
        function xe = error(obj,xd)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            % Position error definition
            x = obj.pendulum.get_positions();
            xe = xd - x;
        end
        
        function xep = error_p(obj, xdp)
            % Velocity Error Definition
            xp = obj.pendulum.get_velocities();
            xep = xdp -xp;
        end
        
        function u = get_control_PD(obj, xd, xdp)
            % Get Error Value
            
            xe = obj.error(xd);
            
            % Get Error dot Value
            xep = obj.error_p(xdp);
            
            % Control Law
            u = obj.K1*xe + obj.K2*xep;
        end
        function u = get_control_PD_Gravity(obj, xd, xdp)
            % Get Error Value
            
            xe = obj.error(xd);
            
            % Get Error dot Value
            xep = obj.error_p(xdp);
            
            % Control Law
            u = obj.K1*xe + obj.K2*xep + obj.pendulum.G_matrix();
        end
        
        function u = get_control_inverse_full(obj, xd, xdp, xdpp)
           
            % Auxiliar operator
            v = obj.operator_control(xd, xdp, xdpp);
            
            % Matrices of the system
            M = obj.pendulum.M_matrix();
            C = obj.pendulum.C_matrix();
            G = obj.pendulum.G_matrix();
            F = obj.pendulum.F_matrix();
            
            % Error In matrices
            
            % Get system states 
            x = obj.pendulum.get_positions();
            xp = obj.pendulum.get_velocities();
            
            u = M*v + C*xp + G + F*xp;
            
        end
        
        function v = operator_control(obj, xd, xdp, xdpp)
            % Get control error value
            xe = obj.error(xd);
            
            % Get Error dot Value
            xep = obj.error_p(xdp);
            
            % Control operator
            v = xdpp + obj.K1*xe + obj.K2*xep;
        end
        
        function learning = learning_control(obj, xd)
            xe = obj.error(xd);
            % Learning algorithm
            l1_k = -obj.B*obj.l1_memories + obj.A*obj.e1_memories;
            l2_k = -obj.B*obj.l2_memories + obj.A*obj.e2_memories;
            
            % Vector of learning parameters
            learning = [l1_k;...
                        l2_k];
             
            % Update values memories learning 1
            obj.update_l_memories(learning);
            obj.update_e_memories(xe)
             
        end
        
        function update_l_memories(obj, l)
            for k = length(obj.l1_memories):-1:2
                obj.l1_memories(k) = obj.l1_memories(k-1);
                obj.l2_memories(k) = obj.l2_memories(k-1);
            end
            obj.l1_memories(1) = l(1);
            obj.l2_memories(1) = l(2);
        end
        
        function update_e_memories(obj,xe)
            for k = length(obj.e1_memories):-1:2
               obj.e1_memories(k) = obj.e1_memories(k-1); 
               obj.e2_memories(k) = obj.e2_memories(k-1);
            end
            obj.e1_memories(1) = xe(1);
            obj.e2_memories(1) = xe(2);
        end
    end
end

