classdef robot
    properties
        L1
        L2
        L3
    end
    methods
        function obj = robot(a1, a2, a3)
            obj.L1 = a1;
            obj.L2 = a2;
            obj.L3 = a3;
        end
        function ret = IK(obj,pWx, pWy, pWz)
            ret = [-1 -1 -1]; % assigning invalid value

            % Moving pWz from frame 1 to frame 2
            pWz = pWz - obj.L1;

            % Calculating cos(theta3)
            c3 = (pWx^2 + pWy^2 + pWz^2 - obj.L2^2 - obj.L3^2) / (2 * obj.L2 * obj.L3);
            
            % Check for the validity of the solution
            if abs(c3) > 1
                disp("Point outside workspace");
                return;
            end
            
            % Calculating possible values for sin(theta3)
            s3_pos = sqrt(1 - c3^2);  % sin(theta_3) for the first solution
            s3_neg = -s3_pos;         % sin(theta_3) for the second solution
    
            % Calculating possible values for theta3
            theta3_1 = atan2(s3_pos, c3);  % First solution for theta_3
            theta3_2 = atan2(s3_neg, c3);  % Second solution for theta_3
            
            % Calculating possible values for theta2
            theta2_1 = atan2(pWz,sqrt(pWx^2 + pWy^2)) - atan2((obj.L3*sin(theta3_1)),(obj.L2 + obj.L3*cos(theta3_1)));
            theta2_2 = atan2(pWz,sqrt(pWx^2 + pWy^2)) - atan2((obj.L3*sin(theta3_2)),(obj.L2 + obj.L3*cos(theta3_2)));

            % Calculating possible values for theta1
            theta1_1 = atan2(pWy, pWx);  % First solution for theta_1
            theta1_2 = atan2(-pWy, -pWx);  % Second solution for theta_1
            
            % Choosing the combination of theta2 and thata3 values in respective permissible ranges
            if theta2_1 >= 0 && theta2_1 <= pi && theta3_1 >= -pi/2 && theta3_1 <= pi/2
                theta2 = theta2_1;
                theta3 = theta3_1;
            elseif theta2_2 >= 0 && theta2_2 <= pi && theta3_2 >= -pi/2 && theta3_2 <= pi/2
                theta2 = theta2_2;
                theta3 = theta3_2;
            else
                disp("No unique combination of theta 2 and theta 3 within constraints")
                return
            end

            % Choosing the theta1 value in permissible range and making changes to theta2 and theta3 to accomodate for it if necessary
            if theta1_1 >= 0 && theta1_1 <= pi
                theta1 = theta1_1;
            elseif theta1_2 >= 0 && theta1_2 <= pi
                theta1 = theta1_2;
                theta2 = pi-theta2;
                theta3 = -theta3;
            else
                disp("No unique theta 1 within constraints")
                return;
            end
            ret = [rad2deg(theta1), rad2deg(theta2), rad2deg(theta3)];
            return
        end
    end
end