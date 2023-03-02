classdef joint_config < handle
    properties
        mass
        length
        moi
        cgRadius
        minAngle
        maxAngle
        motor
    end

    methods
        function obj = joint_config(mass, length, moi, cgRadius, motor)
            obj.mass = mass;
            obj.length = length;
            obj.moi = moi;
            obj.cgRadius = cgRadius;
            obj.motor = motor;
        end
    end
end