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
        gearboxReduction;
    end

    methods
        function this = dc_motor(nominalVoltageVolts, stallTorqueNewtonMeters, stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, numMotors, gearboxReduction)
            this.nominalVoltageVolts = nominalVoltageVolts;
            this.stallTorqueNewtonMeters = stallTorqueNewtonMeters * numMotors * gearboxReduction;
            this.stallCurrentAmps = stallCurrentAmps * numMotors;
            this.freeCurrentAmps = freeCurrentAmps * numMotors;
            this.freeSpeedRadPerSec = freeSpeedRadPerSec / gearboxReduction;

            this.rOhms = nominalVoltageVolts / this.stallCurrentAmps;
            this.KvRadPerSecPerVolt = freeSpeedRadPerSec / (nominalVoltageVolts - this.rOhms * this.freeCurrentAmps);
            this.KtNMPerAmp = this.stallTorqueNewtonMeters / this.stallCurrentAmps;

            this.gearboxReduction = gearboxReduction;   % just for reference
        end
    
        function voltage = getVoltage(this, torqueNm, speedRadiansPerSec) 
            voltage = 1.0 ./ this.KvRadPerSecPerVolt .* speedRadiansPerSec + 1.0 ./ this.KtNMPerAmp .* this.rOhms .* torqueNm;
        end

        function current = getCurrent(this, speedRadiansPerSec, voltageInputVolts)
            current = -1.0 ./ this.KvRadPerSecPerVolt ./ this.rOhms .* speedRadiansPerSec + 1.0 ./ this.rOhms .* voltageInputVolts;        
        end
    end

    methods (Static)
        function radPerSec = rotationsPerMinuteToRadiansPerSecond(rpm)
            radPerSec = rpm * pi / (60 / 2);
        end
    end        
end
