classdef falcon500 < dc_motor
    methods
        function obj = falcon500(numMotors, gearReduction)
            obj@dc_motor(12, 4.69, 257, 1.5, dc_motor.rotationsPerMinuteToRadiansPerSecond(6380.0), numMotors, gearReduction);
        end

        function voltage = getVoltage(obj, torqueNm, speedRadiansPerSec) 
            voltage = getVoltage@dc_motor(obj, torqueNm, speedRadiansPerSec);
        end

        function current = getCurrent(obj, speedRadiansPerSec, voltageInputVolts)
            current = getCurrent@dc_motor(obj, speedRadiansPerSec, voltageInputVolts);        
        end
    end
end
