classdef joint_config < handle
    properties
        mass
        length
        moi
        cgRadius
        minAngle
        maxAngle
        reduction
        motor
    end

    methods
        function obj = joint_config(mass, length, moi, cgRadius, reduction, motor)
            obj.mass = mass;
            obj.length = length;
            obj.moi = moi;
            obj.cgRadius = cgRadius;
            obj.reduction = reduction;
            obj.motor = motor;
        end
    end
end