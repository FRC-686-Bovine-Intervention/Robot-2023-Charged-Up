classdef dc_motor < handle
    properties
        nominalVoltageVolts;
        stallTorqueNewtonMeters;
        stallCurrentAmps;
        freeCurrentAmps;
        freeSpeedRadPerSec;
        rOhms;
        KvRadPerSecPerVolt;
        KtNMPerAmp;
    end

    methods
        function this = dc_motor(nominalVoltageVolts, stallTorqueNewtonMeters, stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, numMotors)
            this.nominalVoltageVolts = nominalVoltageVolts;
            this.stallTorqueNewtonMeters = stallTorqueNewtonMeters * numMotors;
            this.stallCurrentAmps = stallCurrentAmps * numMotors;
            this.freeCurrentAmps = freeCurrentAmps * numMotors;
            this.freeSpeedRadPerSec = freeSpeedRadPerSec;

            this.rOhms = nominalVoltageVolts / this.stallCurrentAmps;
            this.KvRadPerSecPerVolt = freeSpeedRadPerSec / (nominalVoltageVolts - this.rOhms * this.freeCurrentAmps);
            this.KtNMPerAmp = this.stallTorqueNewtonMeters / this.stallCurrentAmps;
        end
    
        function voltage = get_voltage(this, torqueNm, speedRadiansPerSec) 
            voltage = 1.0 ./ this.KvRadPerSecPerVolt .* speedRadiansPerSec + 1.0 ./ this.KtNMPerAmp .* this.rOhms .* torqueNm;
        end

        function current = get_current(this, speedRadiansPerSec, voltageInputVolts)
            current = -1.0 ./ this.KvRadPerSecPerVolt ./ this.rOhms .* speedRadiansPerSec + 1.0 ./ this.rOhms .* voltageInputVolts;        
        end
    end

    methods (Static)
        function radPerSec = rotationsPerMinuteToRadiansPerSecond(rpm)
            radPerSec = rpm * pi / (60 / 2);
        end
    end        
end
