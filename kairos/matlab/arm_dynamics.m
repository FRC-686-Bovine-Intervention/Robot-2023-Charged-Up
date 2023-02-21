classdef arm_dynamics < handle
    properties
        proximal
        distal
        g = 9.80665
    end

    methods
        function obj = arm_dynamics(proximal, distal, grabber)
            obj.proximal = proximal;
            obj.distal = distal;

            obj.distal.cgRadius = (distal.cgRadius * distal.mass + (distal.length + grabber.cgRadius) * grabber.mass) / (distal.mass + grabber.mass);
            obj.distal.moi = distal.mass * (distal.cgRadius - obj.distal.cgRadius)^2 + grabber.mass * (distal.length + grabber.cgRadius - obj.distal.cgRadius)^2;            
            obj.distal.mass = distal.mass + grabber.mass;
            obj.distal.length = distal.length + grabber.length;
        end

        function [torque, voltage, current] = calculate(obj, position, velocity, acceleration)
            M = zeros(2,2);
            C = zeros(2,2);
            Tg = zeros(2,1);

            m1 = obj.proximal.mass;
            l1 = obj.proximal.length;
            r1 = obj.proximal.cgRadius;
            I1 = obj.proximal.moi;
            c1 = cos(position(1));
            s1 = sin(position(1));

            m2 = obj.proximal.mass;
            l2 = obj.proximal.length;
            r2 = obj.proximal.cgRadius;
            I2 = obj.proximal.moi;
            c2 = cos(position(2));
            s2 = sin(position(2));
            
            c12 = cos(position(1) + position(2));
            g = obj.g; %#ok<*PROPLC> 

            M(1,1) = m1*r1^2 + m2*(l1^2 + l2^2) + I1 + I2 + 2*m2*l1*r2*c2;
            M(1,2) = m2*r2^2 + I2 + m2*l1*r2*c2;
            M(2,1) = M(1,2);
            M(2,2) = m2*r2^2 + I2;

            C(1,1) = -m2*I1*r2*s2*velocity(2);
            C(1,2) = -m2*I1*r2*s2*(velocity(1) + velocity(2));
            C(2,1) = m2*l1*r2*s2*velocity(1);
            C(2,2) = 0;

            Tg(1) = (m1*r1 + m2*l2)*g*c1 + m2*r2*g*c12;
            Tg(2) = m2*r2*g*c12;

            torque = M*acceleration + C*velocity + Tg;

            voltage(1,:) = obj.proximal.motor.get_voltage(torque(1,:), velocity(1,:));
            voltage(2,:) = obj.distal.motor.get_voltage(torque(2,:), velocity(2,:));

            current(1,:) = obj.proximal.motor.get_current(velocity(1,:), voltage(1,:));
            current(2,:) = obj.distal.motor.get_current(velocity(2,:), voltage(2,:));
        end
    end
end