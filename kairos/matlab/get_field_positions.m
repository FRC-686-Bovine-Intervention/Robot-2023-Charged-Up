function field = get_field_positions(bumper_width_inches)

%=====================================
% Field positions 
% relative to center of turret rotation
%=====================================

field.pole_dia = 1.66;
field.low_pole.x  = bumper_width_inches/2 + 14.25/2;
field.low_pole.y  = 0.0;
field.mid_pole.x  = bumper_width_inches/2 + 22.75;
field.mid_pole.y  = 34.0;
field.high_pole.x = bumper_width_inches/2 + 39.75;
field.high_pole.y = 46.0;
field.dbl_substation.x      = bumper_width_inches/2;
field.dbl_substation.y      = 37.3;

field.shelf_depth = 17.0;
field.dbl_substation_shelf_depth = 14.0;

field.low_shelf_front_corner.x    = bumper_width_inches/2;
field.SCORE_HYBRID.x         = field.low_shelf_front_corner.x + field.shelf_depth/2;
field.low_shelf.y                 = 0.0;
field.mid_shelf_front_corner.x    = bumper_width_inches/2 + 14.25;
field.SCORE_MID_CUBE.x         = field.mid_shelf_front_corner.x + field.shelf_depth/2;
field.mid_shelf.y                 = 23.5;
field.high_shelf_front_corner.x   = bumper_width_inches/2 + 31.625;
field.SCORE_HIGH_CUBE.x        = field.high_shelf_front_corner.x + field.shelf_depth/2;
field.high_shelf.y                = 35.5;
field.dbl_substation_front_corner.x   = field.dbl_substation.x;
field.dbl_substation_front_corner.y   = field.dbl_substation.y + 4.0;
field.DOUBLE_SUBTATION.x         = field.dbl_substation_front_corner.x + field.dbl_substation_shelf_depth/2;
field.DOUBLE_SUBTATION.y         = field.dbl_substation_front_corner.y;

field.back_wall.x = bumper_width_inches/2 + 48;