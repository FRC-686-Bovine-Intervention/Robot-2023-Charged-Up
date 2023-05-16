P = 111;    % points needed for 95% chance to CMP
R = 20;     % rank through qualifications
A = 8;      % draft selection (captain # or pick # in draft)


N = 60;     % number of teams in competition
a = 1.07;

% qualification points
qual_pts = ceil(erfinv((N-2*R+2)/(a*N))*(10/(erfinv(1/a)))+12);
draft_pts = 17 - A;

% necessary playoff points
playoff_pts = ceil((P-3*qual_pts-3*draft_pts)/3);

if playoff_pts >= 7
    fprintf('Need to place 4th (loser match 12)\n')
elseif playoff_pts >= 13
    fprintf('Need to place 3rd (loser match 13)\n')
elseif playoff_pts >= 20
    fprintf('Need to place 2nd (finalist)\n')
elseif playoff_pts >= 30
    fprintf('Need to place 1st (winner)\n')
else
    fprintf('Can''t qualify');
end