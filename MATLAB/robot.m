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
            % Step 1: Calculate cos(theta_3) and sin(theta_3)
            dist = (pWx^2 + pWy^2 + pWz^2 - obj.L2^2 - obj.L3^2) / (2 * obj.L2 * obj.L3);
            
            % Check for the validity of the solution
            if abs(dist) > 1
                disp("Point outside workspace");
                return;
            end
            s3_pos = sqrt(1 - c3^2);  % sin(theta_3) for the first solution
            s3_neg = -s3_pos;         % sin(theta_3) for the second solution
    
            % Step 2: Calculate theta_3 (select the one with smallest angular displacement)
            theta3_1 = atan2(s3_pos, c3);  % First solution for theta_3
            theta3_2 = atan2(s3_neg, c3);  % Second solution for theta_3
    
            % Choose the smallest angular displacement solution for theta_3
            if theta3_1 >= 0 && theta3_1 <= pi
                theta3 = theta3_1;
            elseif theta3_2 >= 0 && theta3_2 <= pi
                theta3 = theta3_2;
            else
                disp("No unique theta 3 within constraints")
                return;
            end
            % Step 3: Calculate c2 and s2 for theta_2
            c2_pos = (pWx^2 + pWy^2) * (obj.L2 + obj.L3 * c3) + pWz * obj.L3 * s3_pos;
            s2_pos = pWz * (obj.L2 + obj.L3 * c3) - sqrt(pWx^2 + pWy^2) * obj.L3 * s3_pos;
            c2_neg = (pWx^2 + pWy^2) * (obj.L2 + obj.L3 * c3) + pWz * obj.L3 * s3_neg;
            s2_neg = pWz * (obj.L2 + obj.L3 * c3) - sqrt(pWx^2 + pWy^2) * obj.L3 * s3_neg;
            
            % Calculate theta_2 (choose the solution with smallest angular displacement)
            theta2_1 = atan2(s2_pos, c2_pos);
            theta2_2 = atan2(s2_neg, c2_neg);
            theta2_3 = atan2(s2_pos, c2_neg);
            theta2_4 = atan2(s2_neg, c2_pos);
            
            % Choose the solution for theta_2 that minimizes joint displacement (smallest theta_2)
            if theta2_1 >= 0 && theta2_1 <= pi
                theta2 = theta2_1;
    
            elseif theta2_2 >= 0 && theta2_2 <= pi
                theta2 = theta2_2;
    
            elseif theta2_3 >= 0 && theta2_3 <= pi
                theta2 = theta2_3;

            elseif theta2_4 >= 0 && theta2_4 <= np.pi
                theta2 = theta2_4;
            
            else
                disp("No unique theta 2 within constraints")
                return;
            end
    
            % Step 4: Calculate theta_1 (choose the solution with smallest angular displacement)
            theta1_1 = atan2(pWy, pWx);  % First solution for theta_1
            theta1_2 = atan2(-pWy, -pWx);  % Second solution for theta_1
            % Choose the smallest angular displacement solution for theta_1
    
            if theta1_1 >= 0 && theta1_1 <= pi
                theta1 = theta1_1;
    
            elseif theta1_2 >= 0 && theta1_2 <= pi
                theta1 = theta1_2;
    
            else
                disp("No unique theta 1 within constraints")
                return;
            end

            ret = [theta1, theta2, theta3];
            return
        end
    end
end