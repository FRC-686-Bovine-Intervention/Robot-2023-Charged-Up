classdef arm_kinematics < handle
    
    properties
        x1;
        y1;
        arm_length;
    end
    
    methods
        function obj = arm_kinematics(x1, y1, arm_length)
            obj.x1 = x1;
            obj.y1 = y1;
            obj.arm_length = arm_length;
        end
        


        
        function [x2, y2, x3, y3] = forward_kinematics(obj, theta1, theta2)
           x2 = obj.x1 + obj.arm_length(1)*cos(deg2rad(theta1));
           y2 = obj.y1 + obj.arm_length(1)*sin(deg2rad(theta1));

           x3 = x2 + obj.arm_length(2)*cos(deg2rad(theta2));
           y3 = y2 + obj.arm_length(2)*sin(deg2rad(theta2));
        end


        
        function [theta1, theta2, x2, y2, valid] = inverse_kinematics(obj, x3, y3, varargin)
            % find where pivot point between 2 joints could be
            % (finding intersection of 2 circles centered at the base pivot and [x,z]
            % https://math.stackexchange.com/questions/256100/how-can-i-find-the-points-at-which-two-circles-intersect

            x1 = obj.x1;
            y1 = obj.y1;
            r1 = obj.arm_length(1); % PROXIMAL
            r2 = obj.arm_length(2); % DISTAL

            D = sqrt((x3-x1).^2 + (y3-y1).^2);
            J = (r1^2 - r2^2)./(2*D.^2);
            K = 1/2*sqrt(2*(r1^2+r2^2)./D.^2 - (r1^2-r2^2)^2./D.^4 - 1);
            K(abs(imag(K)) > 10*eps) = NaN; % NaN when no solution is possible

            x2_a = (x1+x3)/2 + J.*(x3-x1) +  K.*(y3-y1);
            y2_a = (y1+y3)/2 + J.*(y3-y1) +  K.*(x1-x3);

            x2_b = (x1+x3)/2 + J.*(x3-x1) -  K.*(y3-y1);
            y2_b = (y1+y3)/2 + J.*(y3-y1) -  K.*(x1-x3);
            
            theta1_a = angle(complex(x2_a,y2_a) - complex(x1,y1));
            theta1_b = angle(complex(x2_b,y2_b) - complex(x1,y1));

            x2 = x2_a;
            y2 = y2_a;
            if nargin<=3
                % pick elbow back intersection point (lowest theta1)
                idx = find(theta1_b < theta1_a);
                x2(idx) = x2_b(idx);
                y2(idx) = y2_b(idx);
            else
                % alternate
                idx = find(theta1_b >= theta1_a);
                x2(idx) = x2_b(idx);
                y2(idx) = y2_b(idx);
            end

            % calculate joint angles
            theta1 = rad2deg(angle(complex(x2,y2) - complex(x1,y1)));
            theta2 = rad2deg(angle(complex(x3,y3) - complex(x2,y2)));
            valid = true(size(theta1));

            % for points we can't reach, return an angle pointing in the
            % right angle
            idx = isnan(K);
            theta1(idx) = rad2deg(angle(complex(x3(idx),y3(idx)) - complex(x1,y1)));
            theta2(idx) = theta1(idx);
            valid(idx) = false;
        end


       
    end
end
    
    