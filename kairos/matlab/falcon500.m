classdef falcon500 < dc_motor
    methods
        function obj = falcon500(numMotors)
            obj@dc_motor(12, 4.69, 257, 1.5, dc_motor.rotationsPerMinuteToRadiansPerSecond(6380.0), numMotors);
        end

        function voltage = get_voltage(obj, torqueNm, speedRadiansPerSec) 
            voltage = get_voltage@dc_motor(obj, torqueNm, speedRadiansPerSec);
        end

        function current = get_current(obj, speedRadiansPerSec, voltageInputVolts)
            current = get_current@dc_motor(obj, speedRadiansPerSec, voltageInputVolts);        
        end
    end
end
