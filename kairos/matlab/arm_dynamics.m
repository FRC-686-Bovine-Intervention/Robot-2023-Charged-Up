classdef arm_dynamics < handle
    properties (Constant)
        g = 9.80665;
        SHOULDER = 1;
        ELBOW = 2;
    end
    
    properties
        shoulder
        elbow
    end

    methods
        function obj = arm_dynamics(shoulder, elbow, wrist)
            obj.shoulder = shoulder;

            elbow_cgRadius = (elbow.cgRadius*elbow.mass + (elbow.length + wrist.cgRadius)*wrist.mass) / (elbow.mass + wrist.mass);
            elbow_moi = elbow.mass*(elbow.cgRadius - elbow_cgRadius)^2 + wrist.mass*(elbow.length + wrist.cgRadius - wrist.cgRadius)^2;

            obj.elbow = joint_config(elbow.mass + wrist.mass, ...
                elbow.length + wrist.length,...
                elbow_moi, ...
                elbow_cgRadius, ...
                elbow.motor);
        end

        function [torque, voltage, current] = feedforward(obj, position, velocity, acceleration)
            [M, C, Tg] = get_M_C_Tg(obj, position, velocity);
            torque = M*acceleration + C*velocity + Tg;
            voltage(obj.SHOULDER,:) = obj.shoulder.motor.getVoltage(torque(obj.SHOULDER,:), velocity(obj.SHOULDER,:));
            voltage(obj.ELBOW,:) = obj.elbow.motor.getVoltage(torque(obj.ELBOW,:), velocity(obj.ELBOW,:));
            current(obj.SHOULDER,:) = obj.shoulder.motor.getCurrent(velocity(obj.SHOULDER,:), voltage(obj.SHOULDER,:));
            current(obj.ELBOW,:) = obj.elbow.motor.getCurrent(velocity(obj.ELBOW,:), voltage(obj.ELBOW,:));          
        end

        function [M, C, Tg] = get_M_C_Tg(obj, position, velocity)
            m1 = obj.shoulder.mass;
            l1 = obj.shoulder.length;
            r1 = obj.shoulder.cgRadius;
            I1 = obj.shoulder.moi;
            c1 = cos(position(obj.SHOULDER));

            m2 = obj.elbow.mass;
            r2 = obj.elbow.cgRadius;
            I2 = obj.elbow.moi;
            c2 = cos(position(obj.ELBOW));
            
            cx = cos(position(obj.SHOULDER) - position(obj.ELBOW));
            sx = sin(position(obj.SHOULDER) - position(obj.ELBOW));
            
            g = obj.g; %#ok<*PROPLC> 

            M = zeros(2,2);
            M(1,1) = m1*r1^2 + m2*l1^2 + I1;
            M(1,2) = m2*l1*r2 + I2 + m2*l1*r2*cx;
            M(2,1) = M(1,2);
            M(2,2) = m2*r2^2 + I2;

            C = zeros(2,2);
            C(1,1) = +m2*l1*r2*sx*velocity(obj.ELBOW);
            C(1,2) = +m2*l1*r2*sx*(velocity(obj.SHOULDER) - velocity(obj.ELBOW));
            C(2,1) = C(1,2);
            C(2,2) = +m2*l1*r2*sx*velocity(obj.SHOULDER);

            Tg(1,1) = (m1*r1 + m2*l1)*g*c1 + m2*r2*g*c2;
            Tg(2,1) = m2*r2*g*c2;
        end

    end
end