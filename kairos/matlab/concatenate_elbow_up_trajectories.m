for start = 3:7
    for final = 3:7
        if start ~= final

            start_filename  = fullfile('..\..\src\main\deploy\paths\', sprintf('arm_path_%d_%d.json', start, 0));
            final_filename  = fullfile('..\..\src\main\deploy\paths\', sprintf('arm_path_%d_%d.json', 0, final));
            concat_filename = fullfile('..\..\src\main\deploy\paths\', sprintf('arm_path_%d_%d.json', start, final));

            start_traj = jsondecode(fileread(start_filename));
            final_traj = jsondecode(fileread(final_filename));

            concat_json.startPos = start_traj.startPos;
            concat_json.finalPos = start_traj.finalPos;
            concat_json.totalTime = start_traj.totalTime + final_traj.totalTime;
            theta1 = [start_traj.theta1; final_traj.theta1];
            theta2 = [start_traj.theta2; final_traj.theta2];           

            % look for areas to trim
            removed_theta1 = [];
            removed_len = 0;
            for j=length(start_traj.theta1):-1:1
                d = sqrt((final_traj.theta1 - start_traj.theta1(j)).^2 + (final_traj.theta2 - start_traj.theta2(j)).^2);
                idx = find(d < 0.05, 1, 'last');

                if isempty(idx)
                    continue
                end

                removed_theta1 = [start_traj.theta1(j+1:end); final_traj.theta1(1:idx)];
                removed_theta2 = [start_traj.theta2(j+1:end); final_traj.theta2(1:idx)];

                if length(removed_theta1) > removed_len
                    removed_len = length(removed_theta1);
                    theta1 = [start_traj.theta1(1:j); final_traj.theta1(idx+1:end)];
                    theta2 = [start_traj.theta2(1:j); final_traj.theta2(idx+1:end)];
                end
            end

            concat_json.theta1 = theta1;
            concat_json.theta2 = theta2;
            concat_json.totalTime = concat_json.totalTime * (length(theta1) / (length(theta1) + length(removed_theta1)));

            fid = fopen(concat_filename,'w');
            fprintf(fid, "%s", jsonencode(concat_json, PrettyPrint=true));
            fclose(fid);
        end
    end
end
