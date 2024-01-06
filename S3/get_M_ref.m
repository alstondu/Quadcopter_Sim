% Function to generate reference matrix
function M_r = get_M_ref(circle_points)
    % 4 target points plus points on the circle
    M_r = zeros(12,circle_points+4);
    % Assign first 2 target points
    M_r(1:3,1:2) = [0,    0; 
                    0,   2.5;
                    5,    5];
    % Assign points on the circle
    M_r(1:3, 3:circle_points+2) = ref_circle(circle_points);
    % Assign final 2 reference points
    M_r(1:6,circle_points+3:circle_points+4) = [2.5,   2.5; 
                                                2.5,   2.5;
                                                2.5,     0;
                                                  0,     0;
                                                  0,     0;
                                                  0,  -0.1];
end

% Function to generate points on the circle trajectory
function rc = ref_circle(circle_points)
% Parameters for the circular trajectory
    r = 2.5;  % radius of the circle
    center = [0, 0, 5];  % center of the circle
    
    % Calculate the angle step
    angleStep = 2 * pi / circle_points;
    rc = zeros(3, circle_points);  % 3 position states
    % Generate the points
    for i = 1:circle_points
        % Calculate the angle for this point, shifting by one step
        angle = angleStep * (i); 

        % Calculate the coordinates
        py = center(2) + r * cos(angle);
        pz = center(3) + r * sin(angle);

        % Assign the coordinates to the points matrix
        rc(2, i) = py;  % y position
        rc(3, i) = pz;  % z position
    end
end