% Kompiliere alle Funktionen, die für diese Toolbox notwendig sind.

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-07
% (c) Institut für Regelungstechnik, Universität Hannover


matlabfcn2mex({ 'find_intersection_line_box', ...
                'find_intersection_line_cylinder', ...
                'box_random_surface_point', ...
                'box_random_surface_point_equal', ...
                'cylinder_random_surface_point', ...
                'cylinder_random_surface_point_equal', ...
                'atlas5_wbody_intersect_collbodies', ...
                'atlas5_wbody_collision_detection', ...
                'atlas5_wbody_collision_isolation', ...
                'atlas5_wbody_collision_isolation_fullchain', ...
                'atlas5_wbody_collision_identification', ...
                'atlas5_wbody_collision_isolation_fullchain_multi_ic_in', ...
                'atlas5_wbody_collision_isolation_fullchain_multi', ...
                'atlas5_wbody_collision_isolation_forcesensor', ...
                'atlas5_wbody_collision_identification_multi'});

              