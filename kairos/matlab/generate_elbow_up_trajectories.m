presetEnum = get_preset_enum;

s = jsondecode(fileread('..\..\src\main\deploy\arm_preset_poses.json'));

preset(presetEnum.AUTO_START).info = s.auto_start;              preset(presetEnum.AUTO_START).name = "auto_start";
preset(presetEnum.DEFENSE).info = s.defense;                    preset(presetEnum.DEFENSE).name = "defense";
preset(presetEnum.DOUBLE_SUBSTATION).info = s.double_substation;          preset(presetEnum.DOUBLE_SUBSTATION).name = "double_substation";
preset(presetEnum.SCORE_MID_CUBE).info = s.score_mid_cube;      preset(presetEnum.SCORE_MID_CUBE).name = "score_mid_cube";
preset(presetEnum.SCORE_HIGH_CUBE).info = s.score_high_cube;    preset(presetEnum.SCORE_HIGH_CUBE).name = "score_high_cube";    
preset(presetEnum.SCORE_MID_CONE).info = s.score_mid_cone;      preset(presetEnum.SCORE_MID_CONE).name = "score_mid_cone";
preset(presetEnum.SCORE_HIGH_CONE).info = s.score_high_cone;    preset(presetEnum.SCORE_HIGH_CONE).name = "score_high_cone";    

for startPreset = [presetEnum.AUTO_START presetEnum.DEFENSE]
    for finalPreset = [presetEnum.DOUBLE_SUBSTATION (presetEnum.SCORE_MID_CUBE : presetEnum.SCORE_HIGH_CONE)]
    
        [theta1, theta2] = get_elbow_up_path(preset(startPreset).info.theta1, preset(startPreset).info.theta2, ...
                                             preset(finalPreset).info.theta1, preset(finalPreset).info.theta2);
        
        points = [theta1; theta2];
        
        % Make Trajectory ====================================
        
        [points, totalTime] = make_spline_trajectory2(points, strrep(sprintf('%s to %s', preset(startPreset).name, preset(finalPreset).name), '_','\_'));
        
        % write new spline trajectory
        sout.startPos = preset(startPreset).name;
        sout.finalPos = preset(finalPreset).name;
        sout.totalTime = totalTime;
        sout.grannyFactor = 1.25;
        sout.theta1 = points(1,:);
        sout.theta2 = points(2,:);

        
        write_spline_trajectory(preset(startPreset).info.fileIdx, preset(finalPreset).info.fileIdx, sout);
    end
end