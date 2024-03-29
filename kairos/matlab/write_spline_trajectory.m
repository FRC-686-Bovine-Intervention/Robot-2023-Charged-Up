function write_spline_trajectory(startIdx, finalIdx, s, varargin)

writeReversePath = true;
if nargin > 3
    writeReversePath = varargin{1};
end

% Write files ====================================

base_filename = fullfile('..\..\src\main\deploy\paths\', sprintf('arm_path_%d_%d', startIdx, finalIdx));
traj_filename = [base_filename '.json'];
orig_filename = [base_filename '_orig.json'];
spline_filename = [base_filename '_spline.json'];

% copy original trajectory to _orig
if ~exist(orig_filename, 'file')
    copyfile(traj_filename, orig_filename);
end

fid = fopen(spline_filename,'w');
fprintf(fid, "%s", jsonencode(s, PrettyPrint=true));
fclose(fid);

% overwrite original
copyfile(spline_filename, traj_filename);
    

if writeReversePath
    if (finalIdx >= 4) && (finalIdx <= 7) && (startIdx == 0)
        % go faster when returning from a scoring position
        s.grannyFactor = 1.0;
    end

    % write reverse path
    base_filename = fullfile('..\..\src\main\deploy\paths\', sprintf('arm_path_%d_%d', finalIdx, startIdx));
    traj_filename = [base_filename '.json'];
    orig_filename = [base_filename '_orig.json'];
    spline_filename = [base_filename '_spline.json'];
    
    % copy original trajectory to _orig
    if ~exist(orig_filename, 'file')
        copyfile(traj_filename, orig_filename);
    end

    % flip directions and write return path
    temp = s.startPos;
    s.startPos = s.finalPos;
    s.finalPos = temp;
    s.theta1 = fliplr(s.theta1);
    s.theta2 = fliplr(s.theta2);
    
    fid = fopen(spline_filename,'w');
    fprintf(fid, "%s", jsonencode(s, PrettyPrint=true));
    fclose(fid);
    
    % overwrite original
    copyfile(spline_filename, traj_filename);
end 