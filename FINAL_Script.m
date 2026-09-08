 % =========================================================
% AUTONOMOUS INDOOR ROUTE INSPECTION ROBOT SIMULATION
% Full Integrated System Script
% Matlab R2016a
% =========================================================

clear all;
close all;
clc;

% ROBOT PARAMETERS
r  = 0.05;
L  = 0.2;
dt = 0.05;
T  = 150; % raised from initial 90 for accomodation of more complex layouts
N  = T / dt;

% FLOOR PLAN OPTIONS (letter):
%   'A'  = Scattered open-plan office (dense furniture clusters)
%   'B2' = Balanced symmetric layout, RANDOMISED
%   'C'  = Walled multi-room office - Conference Room, Manager's Office,
%          Break Room
%   'D'  = ICB Lab - real measured room (8.8m x 10.4m), 6 lab desks in
%          a 3x2 grid, structural pillar, stool clusters, wastebins
%   'E'  = EEE Lab 1 - real measured room (10.8m x 9.2m), mixed desk
%          types, L-shaped structural pillar, two usable entrances
%
% SCENARIO OPTIONS (number), independent of floor plan:
%   1 = Fixed layout, no dynamic obstacles by default
%   2 = Fixed layout + moving people (always includes people)
%   3 = Random placement in floor plan boundary, no people by default
%
% ADD_PEOPLE (true/false): layers moving people onto whichever scenario
% is active, e.g. scenario=3 + add_people=true = random layout AND people

% ---------------------------------------------------------
% INTERACTIVE SETUP WIZARD
% Set interactive_mode = setting to false skips it 
interactive_mode = true;

% Defaults - used as-is if interactive_mode is false,
floor_plan       = 'A';
scenario         = 1;
add_people       = false;
add_extrapeople  = 0;

if interactive_mode

    % --- Floor plan ---
    fp_options = {'A  -  Scattered open-plan office', ...
                  'B2 -  Balanced symmetric layout (randomised)', ...
                  'C  -  Walled multi-room office', ...
                  'D  -  ICB Lab (measured)', ...
                  'E  -  EEE Lab 1 (measured)'};
    fp_codes = {'A', 'B2', 'C', 'D', 'E'};
    [sel, ok] = listdlg('ListString', fp_options, ...
                         'SelectionMode', 'single', ...
                         'Name', 'Floor Plan', ...
                         'PromptString', 'Choose a floor plan:', ...
                         'ListSize', [360 140]);
    if ok
        floor_plan = fp_codes{sel};
    end

    % --- Scenario ---
    sc_options = {'1  -  Fixed layout', ...
                  '2  -  Fixed layout + moving people', ...
                  '3  -  Random placement'};
    [sel, ok] = listdlg('ListString', sc_options, ...
                         'SelectionMode', 'single', ...
                         'Name', 'Scenario', ...
                         'PromptString', 'Choose a scenario:', ...
                         'ListSize', [360 100]);
    if ok
        scenario = sel;   % list order matches scenario numbers directly
    end

    % --- Add people (only asked if the scenario doesn't already force it) ---
    if scenario ~= 2
        choice = questdlg('Add moving people to this scenario too?', ...
                           'Dynamic People', 'Yes', 'No', 'No');
        add_people = strcmp(choice, 'Yes');
    end

    % --- Extra people count (only asked if people will actually be active) ---
    if scenario == 2 || add_people
        answer = inputdlg('Extra people on top of the default 4 (can be negative):', ...
                           'Extra People', [1 55], {'0'});
        if ~isempty(answer)
            val = str2double(answer{1});
            if ~isnan(val)
                add_extrapeople = round(val);
            end
        end
    end

    fprintf('Running: Floor Plan %s, Scenario %d', upper(floor_plan), scenario);
    if scenario == 2 || add_people
        fprintf(', %d people', max(0, 4+add_extrapeople));
    end
    fprintf('\n');

end

rng('shuffle');

if ~strcmpi(floor_plan,'A') && ~strcmpi(floor_plan,'B2') && ~strcmpi(floor_plan,'C') && ~strcmpi(floor_plan,'D') && ~strcmpi(floor_plan,'E')
    fprintf('NOTE: Floor plan %s not recognised -- using Floor Plan A.\n', floor_plan);
    floor_plan = 'A';
end

if strcmpi(floor_plan, 'A')
    % =====================================================
    % FLOOR PLAN A - Scattered open-plan office
    % =====================================================
    floor_plan_name = 'Scattered open-plan office';

    room_W = 9.0;
    room_H = 9.0;
    door_left  = 4.0;
    door_right = 5.0;

    start_x    = (door_left + door_right) / 2;
    start_y    = 0.2;
    theta_init = pi/2;

    rect_obs = [
        3.6, 5.4, 1.4, 2.1;   % O1  Reception desk
        0.9, 2.1, 2.9, 3.7;   % O2  Desk pod A - desk 1
        0.9, 1.7, 3.7, 4.9;   % O3  Desk pod A - desk 2
        2.1, 3.3, 3.1, 3.9;   % O4  Desk pod A - desk 3
        6.9, 8.1, 2.9, 3.7;   % O5  Desk pod B - desk 1
        7.3, 8.1, 3.7, 4.9;   % O6  Desk pod B - desk 2
        5.7, 6.9, 3.1, 3.9;   % O7  Desk pod B - desk 3
        6.6, 8.2, 5.6, 6.3;   % O8  L-shaped desk - long arm
        6.6, 7.3, 6.3, 7.3;   % O9  L-shaped desk - short arm
        3.7, 5.3, 7.2, 8.2;   % O10 Meeting pod
        1.0, 1.8, 7.8, 8.6;   % O11 Phone booth
        7.2, 8.2, 7.4, 8.6;   % O12 Locker bank
    ];

    rect_colors = [
        0.85 0.55 0.15;
        0.65 0.45 0.25;
        0.65 0.45 0.25;
        0.65 0.45 0.25;
        0.65 0.45 0.25;
        0.65 0.45 0.25;
        0.65 0.45 0.25;
        0.55 0.35 0.65;
        0.55 0.35 0.65;
        0.30 0.55 0.75;
        0.75 0.65 0.20;
        0.45 0.45 0.50;
    ];

    rect_names = {
        'Reception desk'; 'Desk pod'; 'Desk pod'; 'Desk pod'; ...
        'Desk pod'; 'Desk pod'; 'Desk pod'; 'L-shaped desk'; ...
        'L-shaped desk'; 'Meeting pod'; 'Phone booth'; 'Locker bank'
    };

    circ_obs = [
        3.1, 6.1, 0.5;
        4.4, 6.0, 0.5;
        1.4, 6.6, 0.65;
    ];

    circ_colors = [
        0.20 0.55 0.55;
        0.20 0.55 0.55;
        0.70 0.35 0.55;
    ];

    circ_names = {'Round table'; 'Round table'; 'Lounge seat'};

    % Collision insets: 0 = fully solid (default). A positive value means
    % the robot can pass within that distance of the object's centre -
    % an honest 2D approximation of "there's clearance underneath" (e.g.
    % a round table's slender pedestal vs its wide tabletop), not true
    % 3D height simulation. Shrinking-only, so this can never invalidate
    % an already-working route - it can only open up MORE space.
    rect_inset = zeros(size(rect_obs,1), 1);
    circ_inset = [0.35; 0.35; 0];   % both round tables passable, lounge stays solid

    waypoints = [
        6.2, 1.8;
        4.5, 3.5;
        5.3, 6.3;
        5.9, 7.6;
        4.5, 8.7;
    ];

elseif strcmpi(floor_plan, 'B2')
    % =====================================================
    % FLOOR PLAN B2 - Randomised symmetric layout
    % =====================================================
    floor_plan_name = 'Balanced symmetric layout (randomised)';

    room_W = 9.0;
    room_H = 9.0;
    door_left  = 4.0;
    door_right = 5.0;

    start_x    = (door_left + door_right) / 2;
    start_y    = 0.2;
    theta_init = pi/2;

    fprintf('Generating randomised symmetric layout (Floor Plan B2)...\n');

    num_pairs         = 4;
    wall_margin_b2     = 0.6;
    aisle_limit        = 3.2;   % left-half objects stay left of this x,
                                % guaranteeing a >=2.6m aisle once mirrored
    obj_clearance_b2   = 0.35;
    door_clearance_b2  = 0.8;
    max_attempts_b2    = 300;

    rect_list_b2 = zeros(0,4);
    circ_list_b2 = zeros(0,3);

    for pair_i = 1:num_pairs
        is_rect  = rand() < 0.6;
        placed   = false;
        attempts = 0;

        while ~placed && attempts < max_attempts_b2
            attempts = attempts + 1;

            if is_rect
                ow = 0.6 + 1.0*rand();
                oh = 0.6 + 1.0*rand();
                ocx = wall_margin_b2 + ow/2 + (aisle_limit - wall_margin_b2 - ow) * rand();
                ocy = wall_margin_b2 + oh/2 + (room_H - 2*wall_margin_b2 - oh) * rand();
                oxmin = ocx - ow/2; oxmax = ocx + ow/2;
                oymin = ocy - oh/2; oymax = ocy + oh/2;
            else
                orad = 0.3 + 0.35*rand();
                ocx = wall_margin_b2 + orad + (aisle_limit - wall_margin_b2 - 2*orad) * rand();
                ocy = wall_margin_b2 + orad + (room_H - 2*wall_margin_b2 - 2*orad) * rand();
            end

            ok = true;

            if is_rect
                if oymin < door_clearance_b2 && oxmax > (door_left-door_clearance_b2)
                    ok = false;
                end
            else
                if (ocy-orad) < door_clearance_b2 && (ocx+orad) > (door_left-door_clearance_b2)
                    ok = false;
                end
            end

            if ok
                for rk = 1:size(rect_list_b2,1)
                    rxmin=rect_list_b2(rk,1); rxmax=rect_list_b2(rk,2);
                    rymin=rect_list_b2(rk,3); rymax=rect_list_b2(rk,4);
                    if is_rect
                        if (oxmin-obj_clearance_b2) < rxmax && (oxmax+obj_clearance_b2) > rxmin && ...
                           (oymin-obj_clearance_b2) < rymax && (oymax+obj_clearance_b2) > rymin
                            ok = false; break;
                        end
                    else
                        cxn = min(max(ocx,rxmin),rxmax);
                        cyn = min(max(ocy,rymin),rymax);
                        dd  = sqrt((ocx-cxn)^2+(ocy-cyn)^2);
                        if dd < (orad+obj_clearance_b2)
                            ok = false; break;
                        end
                    end
                end
            end

            if ok
                for ck = 1:size(circ_list_b2,1)
                    ccx=circ_list_b2(ck,1); ccy=circ_list_b2(ck,2); crad=circ_list_b2(ck,3);
                    if is_rect
                        cxn = min(max(ccx,oxmin),oxmax);
                        cyn = min(max(ccy,oymin),oymax);
                        dd  = sqrt((ccx-cxn)^2+(ccy-cyn)^2);
                        if dd < (crad+obj_clearance_b2)
                            ok = false; break;
                        end
                    else
                        dd = sqrt((ocx-ccx)^2+(ocy-ccy)^2);
                        if dd < (orad+crad+obj_clearance_b2)
                            ok = false; break;
                        end
                    end
                end
            end

            if ok
                placed = true;
            end
        end

        if attempts >= max_attempts_b2
            fprintf('WARNING: B2 pair %d could not find a fully clear spot -- using best available.\n', pair_i);
        end

        if is_rect
            rect_list_b2 = [rect_list_b2; oxmin, oxmax, oymin, oymax];
        else
            circ_list_b2 = [circ_list_b2; ocx, ocy, orad];
        end
    end

    % Mirror every left-half object across the centreline (x = room_W/2)
    num_left_rect = size(rect_list_b2,1);
    num_left_circ = size(circ_list_b2,1);

    rect_obs = rect_list_b2;
    for rk = 1:num_left_rect
        oxmin = rect_list_b2(rk,1); oxmax = rect_list_b2(rk,2);
        oymin = rect_list_b2(rk,3); oymax = rect_list_b2(rk,4);
        rect_obs = [rect_obs; room_W-oxmax, room_W-oxmin, oymin, oymax];
    end

    circ_obs = circ_list_b2;
    for ck = 1:num_left_circ
        circ_obs = [circ_obs; room_W-circ_list_b2(ck,1), circ_list_b2(ck,2), circ_list_b2(ck,3)];
    end

    % One centrepiece exactly on the mirror axis - geometrically
    % guaranteed clear of every mirrored pair given the aisle_limit
    % construction above, no extra rejection sampling needed
    centre_r = 0.35 + 0.15*rand();
    centre_y = 4.0 + 3.0*rand();
    circ_obs = [circ_obs; room_W/2, centre_y, centre_r];

    % Colours reuse Floor Plan B's original palette for visual continuity
    rect_colors = repmat([0.45 0.60 0.35], size(rect_obs,1), 1);
    circ_colors = repmat([0.75 0.40 0.30], size(circ_obs,1), 1);
    circ_colors(end,:) = [0.55 0.50 0.20];   % centrepiece gets its own accent

    rect_names = repmat({'Furniture (random)'}, size(rect_obs,1), 1);
    circ_names = repmat({'Furniture (random)'}, size(circ_obs,1), 1);
    circ_names{end} = 'Centrepiece';

    rect_inset = zeros(size(rect_obs,1), 1);
    circ_inset = zeros(size(circ_obs,1), 1);   % fully solid for now - same
                                               % scope boundary as Scenario 3

    % --- Random route, validated against this run's mirrored layout ---
    num_wp_b2      = 5;
    wp_list_b2     = zeros(num_wp_b2, 2);
    min_spacing_b2 = 2.2;
    placed_x_b2    = start_x;
    placed_y_b2    = start_y;
    prev_wx_b2     = start_x;
    prev_wy_b2     = start_y;

    for wp_i = 1:num_wp_b2
        wvalid    = false;
        wattempts = 0;

        while ~wvalid && wattempts < max_attempts_b2
            wattempts = wattempts + 1;

            cwx = wall_margin_b2 + (room_W - 2*wall_margin_b2) * rand();
            cwy = wall_margin_b2 + (room_H - 2*wall_margin_b2) * rand();

            cxn = min(max(cwx, rect_obs(:,1)), rect_obs(:,2));
            cyn = min(max(cwy, rect_obs(:,3)), rect_obs(:,4));
            dd_r = sqrt((cwx-cxn).^2 + (cwy-cyn).^2);
            dd_c = sqrt((cwx-circ_obs(:,1)).^2 + (cwy-circ_obs(:,2)).^2) - circ_obs(:,3);
            wvalid = all(dd_r >= 0.5) && all(dd_c >= 0.5);

            if wvalid
                dd_p = sqrt((cwx-placed_x_b2).^2 + (cwy-placed_y_b2).^2);
                wvalid = all(dd_p >= min_spacing_b2);
            end

            if wvalid
                for seg_t = 0:0.1:1
                    sx = prev_wx_b2 + seg_t*(cwx-prev_wx_b2);
                    sy = prev_wy_b2 + seg_t*(cwy-prev_wy_b2);
                    cxn = min(max(sx, rect_obs(:,1)), rect_obs(:,2));
                    cyn = min(max(sy, rect_obs(:,3)), rect_obs(:,4));
                    dd_r = sqrt((sx-cxn).^2 + (sy-cyn).^2);
                    dd_c = sqrt((sx-circ_obs(:,1)).^2 + (sy-circ_obs(:,2)).^2) - circ_obs(:,3);
                    if any(dd_r < 0.35) || any(dd_c < 0.35)
                        wvalid = false; break;
                    end
                end
            end
        end

        if wattempts >= max_attempts_b2
            fprintf('WARNING: B2 waypoint %d could not find a fully clear path -- using best available.\n', wp_i);
        end

        wp_list_b2(wp_i, :) = [cwx, cwy];
        prev_wx_b2  = cwx;
        prev_wy_b2  = cwy;
        placed_x_b2 = [placed_x_b2, cwx];
        placed_y_b2 = [placed_y_b2, cwy];
    end

    waypoints = wp_list_b2;

elseif strcmpi(floor_plan, 'C')
    % =====================================================
    % FLOOR PLAN C - Walled multi-room office
    % Three enclosed rooms (Conference Room, Manager's Office,
    % Break Room), each reachable only through its own doorway
    % off a central corridor. Walls are just more rect_obs
    % entries (thin, elongated) - the exact same collision
    % engine as furniture, no new physics needed.
    % =====================================================
    floor_plan_name = 'Walled multi-room office';

    room_W = 9.0;
    room_H = 9.0;
    door_left  = 4.0;
    door_right = 5.0;

    start_x    = (door_left + door_right) / 2;
    start_y    = 0.2;
    theta_init = pi/2;

    % Rectangles: walls (thin segments with door gaps) + rectangular
    % furniture, including a reception desk in the entrance lobby.
    % [xmin, xmax, ymin, ymax]
    rect_obs = [
        0.40, 3.60, 0.90, 1.05;   % O1  Conference Room - bottom wall
        0.40, 3.60, 4.35, 4.50;   % O2  Conference Room - top wall
        0.40, 0.55, 0.90, 4.50;   % O3  Conference Room - left wall
        3.45, 3.60, 0.90, 2.20;   % O4  Conference Room - right wall (below door)
        3.45, 3.60, 3.00, 4.50;   % O5  Conference Room - right wall (above door)
        0.80, 2.30, 2.00, 3.60;   % O6  Conference table
        5.40, 8.60, 0.90, 1.05;   % O7  Manager's Office - bottom wall
        5.40, 8.60, 3.65, 3.80;   % O8  Manager's Office - top wall
        8.45, 8.60, 0.90, 3.80;   % O9  Manager's Office - right wall
        5.40, 5.55, 0.90, 2.00;   % O10 Manager's Office - left wall (below door)
        5.40, 5.55, 2.80, 3.80;   % O11 Manager's Office - left wall (above door)
        6.80, 8.20, 2.80, 3.40;   % O12 Manager's desk
        0.40, 3.60, 5.30, 5.45;   % O13 Break Room - bottom wall
        0.40, 3.60, 8.45, 8.60;   % O14 Break Room - top wall
        0.40, 0.55, 5.30, 8.60;   % O15 Break Room - left wall
        3.45, 3.60, 5.30, 6.50;   % O16 Break Room - right wall (below door)
        3.45, 3.60, 7.30, 8.60;   % O17 Break Room - right wall (above door)
        6.00, 7.50, 0.30, 0.70;   % O18 Reception desk (entrance lobby, outside all rooms)
        7.10, 8.30, 5.20, 5.90;   % O19 Workspace desk 1 (open corridor area)
        7.00, 8.20, 6.30, 7.00;   % O20 Workspace desk 2
        5.70, 6.90, 7.30, 8.00;   % O21 Workspace desk 3
    ];

    rect_colors = [
        0.55 0.25 0.30;   % Conference Room walls (x5)
        0.55 0.25 0.30;
        0.55 0.25 0.30;
        0.55 0.25 0.30;
        0.55 0.25 0.30;
        0.55 0.25 0.30;   % Conference table
        0.25 0.30 0.45;   % Manager's Office walls (x5)
        0.25 0.30 0.45;
        0.25 0.30 0.45;
        0.25 0.30 0.45;
        0.25 0.30 0.45;
        0.25 0.30 0.45;   % Manager's desk
        0.80 0.50 0.25;   % Break Room walls (x5)
        0.80 0.50 0.25;
        0.80 0.50 0.25;
        0.80 0.50 0.25;
        0.80 0.50 0.25;
        0.45 0.55 0.45;   % Reception desk - neutral, shared-space colour
        0.45 0.55 0.45;   % Workspace desk 1
        0.45 0.55 0.45;   % Workspace desk 2
        0.45 0.55 0.45;   % Workspace desk 3
    ];

    % Names grouped by colour, not by object - the walls and furniture
    % inside each room deliberately SHARE a colour (room-coded, not
    % type-coded), so the legend groups them the same way the drawing does
    rect_names = {
        'Conference Room (wall/table)'; 'Conference Room (wall/table)'; ...
        'Conference Room (wall/table)'; 'Conference Room (wall/table)'; ...
        'Conference Room (wall/table)'; 'Conference Room (wall/table)'; ...
        'Manager''s Office (wall/desk)'; 'Manager''s Office (wall/desk)'; ...
        'Manager''s Office (wall/desk)'; 'Manager''s Office (wall/desk)'; ...
        'Manager''s Office (wall/desk)'; 'Manager''s Office (wall/desk)'; ...
        'Break Room (wall)'; 'Break Room (wall)'; 'Break Room (wall)'; ...
        'Break Room (wall)'; 'Break Room (wall)'; ...
        'Reception / Workspace desk'; 'Reception / Workspace desk'; ...
        'Reception / Workspace desk'; 'Reception / Workspace desk'
    };

    % Circles: [centre_x, centre_y, radius]
    circ_obs = [
        2.70, 1.50, 0.22;   % O22 Conference chair
        7.50, 1.50, 0.22;   % O23 Manager's side chair
        1.50, 7.80, 0.40;   % O24 Break Room round table
        1.50, 6.20, 0.22;   % O25 Break Room chair
        6.50, 5.50, 0.35;   % O26 Lobby plant (corridor, outside all rooms)
        8.00, 7.60, 0.35;   % O27 Workspace shared round table
    ];

    circ_colors = [
        0.55 0.25 0.30;
        0.25 0.30 0.45;
        0.80 0.50 0.25;
        0.80 0.50 0.25;
        0.45 0.55 0.45;   % neutral, shared-space colour
        0.45 0.55 0.45;
    ];

    circ_names = {
        'Conference Room (wall/table)'; 'Manager''s Office (wall/desk)'; ...
        'Break Room (wall)'; 'Break Room (wall)'; ...
        'Reception / Workspace desk'; 'Reception / Workspace desk'
    };

    % Walls stay fully solid; the Break Room's round table is marked
    % passable (same reasoning as Floor Plans A and B)
    rect_inset = zeros(size(rect_obs,1), 1);
    circ_inset = [0; 0; 0.35; 0; 0; 0];

    % Route: door -> conference room (past the table) -> manager's
    % office (past the desk/chair) -> the open workspace cluster ->
    % break room (past the table/chair) -> goal.
    %

    waypoints = [
        4.5, 2.6;    % corridor, level with conference room door
        2.4, 2.6;    % just past the doorway, same height - clean crossing
        0.9, 3.9;    % NOW redirect: deep in conference room, past the table
        4.5, 2.4;    % corridor, level with manager's office door
        7.3, 2.3;    % deep in manager's office, past the desk and chair
        4.5, 2.5;    % back out through manager's door, same height first
        6.5, 7.0;    % NOW redirect: the open workspace cluster
        4.5, 6.9;    % corridor, level with break room door
        1.5, 6.9;    % deep in break room, past the table and chair
        4.5, 6.9;    % back out through break room door, same height first
        4.5, 8.7;    % NOW redirect: goal
    ];

elseif strcmpi(floor_plan, 'D')
    % =====================================================
    % FLOOR PLAN D - ICB Lab (real measured room)
    % Measured on-site: 22 x 26 tiles at 40cm/tile = 8.8m x
    % 10.4m.
    
    % =====================================================
    floor_plan_name = 'ICB Lab (measured)';

    room_W = 8.8;
    room_H = 10.4;
    door_left   = 3.0;    % unused for drawing on this floor plan - see note above
    door_right  = 4.6;    % unused for drawing on this floor plan - see note above
    door_bottom = 8.8;    % actual door: left wall, y in [door_bottom, door_top]
    door_top    = room_H; % flush with the top-left corner, per the sketch

    start_x    = 0.2;
    start_y    = (door_bottom + door_top) / 2;
    theta_init = 0;   % entering through the left wall, facing right (+x) into the room

    % Rectangles: [xmin, xmax, ymin, ymax]
  
    rect_obs = [
        1.7, 2.7, 1.0, 3.4;    % O1  T1 desk - column 1, front row
        1.7, 2.7, 4.4, 6.8;    % O2  T1 desk - column 1, back row
        3.9, 4.9, 1.0, 3.4;    % O3  T1 desk - column 2, front row
        3.9, 4.9, 4.4, 6.8;    % O4  T1 desk - column 2, back row
        6.1, 7.1, 1.0, 3.4;    % O5  T1 desk - column 3, front row
        6.1, 7.1, 4.4, 6.8;    % O6  T1 desk - column 3, back row
        0.1, 0.9, 0.1, 2.1;    % O7  T2 desk - bottom-left corner, long side against left wall
        4.05,4.75,6.9, 7.6;    % O8  Structural pillar - directly above O4
    ];

    rect_colors = [
        0.55 0.40 0.28;   % T1 desks - wood tone (x6)
        0.55 0.40 0.28;
        0.55 0.40 0.28;
        0.55 0.40 0.28;
        0.55 0.40 0.28;
        0.55 0.40 0.28;
        0.55 0.40 0.28;   % T2 desk - same family
        0.50 0.50 0.50;   % pillar - structural, distinct grey
    ];

    rect_names = {
        'Lab desk'; 'Lab desk'; 'Lab desk'; 'Lab desk'; ...
        'Lab desk'; 'Lab desk'; 'Lab desk'; 'Structural pillar'
    };

    % Circles: [centre_x, centre_y, radius]
   
    circ_obs = [
        2.95, 1.3,  0.20;   % O9  Stool 1/4 - column 1 front desk
        2.95, 1.9,  0.20;   % O10 Stool 2/4 - column 1 front desk
        2.95, 2.5,  0.20;   % O11 Stool 3/4 - column 1 front desk
        2.95, 3.1,  0.20;   % O12 Stool 4/4 - column 1 front desk
        2.95, 4.7,  0.20;   % O13 Stool 1/4 - column 1 back desk
        2.95, 5.3,  0.20;   % O14 Stool 2/4 - column 1 back desk
        2.95, 5.9,  0.20;   % O15 Stool 3/4 - column 1 back desk
        2.95, 6.5,  0.20;   % O16 Stool 4/4 - column 1 back desk
        5.15, 1.3,  0.20;   % O17 Stool 1/4 - column 2 front desk
        5.15, 1.9,  0.20;   % O18 Stool 2/4 - column 2 front desk
        5.15, 2.5,  0.20;   % O19 Stool 3/4 - column 2 front desk
        5.15, 3.1,  0.20;   % O20 Stool 4/4 - column 2 front desk
        5.15, 4.7,  0.20;   % O21 Stool 1/4 - column 2 back desk
        5.15, 5.3,  0.20;   % O22 Stool 2/4 - column 2 back desk
        5.15, 5.9,  0.20;   % O23 Stool 3/4 - column 2 back desk
        5.15, 6.5,  0.20;   % O24 Stool 4/4 - column 2 back desk
        7.35, 1.3,  0.20;   % O25 Stool 1/4 - column 3 front desk
        7.35, 1.9,  0.20;   % O26 Stool 2/4 - column 3 front desk
        7.35, 2.5,  0.20;   % O27 Stool 3/4 - column 3 front desk
        7.35, 3.1,  0.20;   % O28 Stool 4/4 - column 3 front desk
        7.35, 4.7,  0.20;   % O29 Stool 1/4 - column 3 back desk
        7.35, 5.3,  0.20;   % O30 Stool 2/4 - column 3 back desk
        7.35, 5.9,  0.20;   % O31 Stool 3/4 - column 3 back desk
        7.35, 6.5,  0.20;   % O32 Stool 4/4 - column 3 back desk
        7.6,  9.3,  0.15;   % O33 Wastebin
        8.0,  9.6,  0.15;   % O34 Wastebin
        7.4,  9.7,  0.15;   % O35 Wastebin
    ];

    circ_colors = [
    0.25 0.55 0.55;   % stools - teal (x24)
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.25 0.55 0.55;
    0.30 0.30 0.30;   % wastebins - charcoal (x3)
    0.30 0.30 0.30;
    0.30 0.30 0.30;
];

    circ_names = [ repmat({'Stool'}, 24, 1); repmat({'Wastebin'}, 3, 1) ];

    % Everything here is solid - no round tables in this lab
    rect_inset = zeros(size(rect_obs,1), 1);
    circ_inset = zeros(size(circ_obs,1), 1);

    waypoints = [
        1.20, 6.60;   % down the left aisle, beside column 1's back desk
        1.10, 4.85;   % continuing down, past the row gap on the left
        1.35, 2.00;   % beside column 1's front desk
        3.15, 3.75;   % crossing right through the front/back row gap
        5.60, 4.15;   % continuing across, between columns 2 and 3
        7.75, 5.35;   % into the right aisle, beside column 3's back desk
        7.90, 8.35;   % clear of the pillar and back row, above the cluster
        8.3,  9.9;    % goal, near the wastebins
    ];

elseif strcmpi(floor_plan, 'E')
    % =====================================================
    % FLOOR PLAN E - EEE Lab 1 (real measured room)
    % Measured on-site: 27 x 23 tiles at 40cm/tile = 10.8m x
    % 9.2m.
    % =====================================================
    floor_plan_name = 'EEE Lab 1 (measured)';

    room_W = 10.8;
    room_H = 9.2;
    door_left  = 1.6;   % shifted slightly left from the first draft
    door_right = 3.2;   % keeps the same doorway width, just left-shifted

    start_x    = (door_left + door_right) / 2;
    start_y    = 0.2;
    theta_init = pi/2;

    % Rectangles: [xmin, xmax, ymin, ymax]
    rect_obs = [
        0.8, 3.0, 6.4, 7.4;    % O1 T2 desk - slightly north of old O2 area
        3.2, 5.6, 3.1, 4.1;    % O2 T1 desk A - rotated horizontal, shifted right
        3.2, 5.6, 5.4, 6.4;    % O3 T1 desk B - rotated horizontal, shifted right
        7.5, 8.6, 0.3, 3.1;    % O4 T3 desk - near the second entrance
        0.3, 1.4, 0.8, 2.8;    % O5 T3 desk - bottom-left corner, not wall-hugging
        8.5, 9.6, 5.0, 7.8;    % O6 T3 desk
        6.3, 7.3, 3.5, 4.1;    % O7 Structural pillar - long arm (L-shape)
        6.3, 6.8, 4.1, 4.6;    % O8 Structural pillar - short arm (L-shape)
    ];

    rect_colors = [
        0.55 0.40 0.28;   % T2 desk - wood tone
        0.55 0.40 0.28;   % T1 desk A
        0.55 0.40 0.28;   % T1 desk B
        0.55 0.40 0.28;   % T3 desks (x3), same family
        0.55 0.40 0.28;
        0.55 0.40 0.28;
        0.50 0.50 0.50;   % pillar - structural, distinct grey (x2 parts)
        0.50 0.50 0.50;
    ];

    rect_names = {
        'Lab desk'; 'Lab desk'; 'Lab desk'; 'Lab desk'; ...
        'Lab desk'; 'Lab desk'; 'Structural pillar'; 'Structural pillar'
    };

    % Circles: [centre_x, centre_y, radius]
    % 3 stools on each long edge of O2 and O3.
    circ_obs = [
        3.7, 4.45, 0.20;   % O9  Stool 1/6 - O2 top edge
        4.4, 4.45, 0.20;   % O10 Stool 2/6 - O2 top edge
        5.1, 4.45, 0.20;   % O11 Stool 3/6 - O2 top edge
        3.7, 2.75, 0.20;   % O12 Stool 4/6 - O2 bottom edge
        4.4, 2.75, 0.20;   % O13 Stool 5/6 - O2 bottom edge
        5.1, 2.75, 0.20;   % O14 Stool 6/6 - O2 bottom edge
        3.7, 6.75, 0.20;   % O15 Stool 1/6 - O3 top edge
        4.4, 6.75, 0.20;   % O16 Stool 2/6 - O3 top edge
        5.1, 6.75, 0.20;   % O17 Stool 3/6 - O3 top edge
        3.7, 5.05, 0.20;   % O18 Stool 4/6 - O3 bottom edge
        4.4, 5.05, 0.20;   % O19 Stool 5/6 - O3 bottom edge
        5.1, 5.05, 0.20;   % O20 Stool 6/6 - O3 bottom edge
    ];

    circ_colors = [
        0.25 0.55 0.55;   % stools - teal (x12)
        0.25 0.55 0.55;
        0.25 0.55 0.55;
        0.25 0.55 0.55;
        0.25 0.55 0.55;
        0.25 0.55 0.55;
        0.25 0.55 0.55;
        0.25 0.55 0.55;
        0.25 0.55 0.55;
        0.25 0.55 0.55;
        0.25 0.55 0.55;
        0.25 0.55 0.55;
    ];

    circ_names = repmat({'Stool'}, 12, 1);

    rect_inset = zeros(size(rect_obs,1), 1);
    circ_inset = zeros(size(circ_obs,1), 1);

   waypoints = [
    2.5, 3.0;    % corridor, between the two T1 desks
    0.6, 4.7;    % past T1 desk A
    1.5, 2.8;    % dip below both desks before heading east - breaks the
                 % goal-perpendicular-to-escape-direction deadlock found
                 % at desk B (same failure class as the doorway bug)
    1.9, 7.9;    % wp4 REPOSITIONED: just north of O1 (T2 desk,
                 % x:0.8-3.0  y:6.4-7.4) - was 3.8, 2.0. Testing
                 % whether the controller can reach it straight
                 % from wp3, no helper waypoint added in between.
    7.6, 4.0;    % past the pillar
    6.4, 6.9;    % past the central T3 desk
    9.0, 2.0;    % goal, near the second entrance
];
end

% ---------------------------------------------------------
% SCENARIO 3 - random obstacle placement within the floor
% ---------------------------------------------------------
if scenario == 3
    if strcmpi(floor_plan, 'C')
        fprintf('NOTE: Scenario 3 replaces Floor Plan C''s walled rooms with a flat random layout - the rooms/doorways only exist under Scenario 1 or 2.\n');
    end
    fprintf('Generating random obstacle layout (Scenario 3)...\n');

    num_random_obs = 9;
    wall_margin    = 0.6;    % keep obstacles this far from every wall
    obj_clearance  = 0.35;   % minimum gap kept between two obstacles
    door_clearance = 0.8;    % keep obstacles clear of the doorway
    max_attempts   = 300;

    rect_list = zeros(0,4);
    circ_list = zeros(0,3);

    for obj_i = 1:num_random_obs
        is_rect  = rand() < 0.6;   % roughly 60% rectangles, 40% circles
        placed   = false;
        attempts = 0;

        while ~placed && attempts < max_attempts
            attempts = attempts + 1;

            if is_rect
                ow = 0.6 + 1.2*rand();
                oh = 0.6 + 1.2*rand();
                ocx = wall_margin + ow/2 + (room_W - 2*wall_margin - ow) * rand();
                ocy = wall_margin + oh/2 + (room_H - 2*wall_margin - oh) * rand();
                oxmin = ocx - ow/2; oxmax = ocx + ow/2;
                oymin = ocy - oh/2; oymax = ocy + oh/2;
            else
                orad = 0.3 + 0.4*rand();
                ocx = wall_margin + orad + (room_W - 2*wall_margin - 2*orad) * rand();
                ocy = wall_margin + orad + (room_H - 2*wall_margin - 2*orad) * rand();
            end

            ok = true;

            % Keep clear of the doorway itself
            if is_rect
                if oymin < door_clearance && oxmax > (door_left-door_clearance) && oxmin < (door_right+door_clearance)
                    ok = false;
                end
            else
                if (ocy-orad) < door_clearance && (ocx+orad) > (door_left-door_clearance) && (ocx-orad) < (door_right+door_clearance)
                    ok = false;
                end
            end

            % Keep clear of previously placed rectangles
            if ok
                for rk = 1:size(rect_list,1)
                    rxmin=rect_list(rk,1); rxmax=rect_list(rk,2);
                    rymin=rect_list(rk,3); rymax=rect_list(rk,4);
                    if is_rect
                        if (oxmin-obj_clearance) < rxmax && (oxmax+obj_clearance) > rxmin && ...
                           (oymin-obj_clearance) < rymax && (oymax+obj_clearance) > rymin
                            ok = false; break;
                        end
                    else
                        cxn = min(max(ocx,rxmin),rxmax);
                        cyn = min(max(ocy,rymin),rymax);
                        dd  = sqrt((ocx-cxn)^2+(ocy-cyn)^2);
                        if dd < (orad+obj_clearance)
                            ok = false; break;
                        end
                    end
                end
            end

            % Keep clear of previously placed circles
            if ok
                for ck = 1:size(circ_list,1)
                    ccx=circ_list(ck,1); ccy=circ_list(ck,2); crad=circ_list(ck,3);
                    if is_rect
                        cxn = min(max(ccx,oxmin),oxmax);
                        cyn = min(max(ccy,oymin),oymax);
                        dd  = sqrt((ccx-cxn)^2+(ccy-cyn)^2);
                        if dd < (crad+obj_clearance)
                            ok = false; break;
                        end
                    else
                        dd = sqrt((ocx-ccx)^2+(ocy-ccy)^2);
                        if dd < (orad+crad+obj_clearance)
                            ok = false; break;
                        end
                    end
                end
            end

            if ok
                placed = true;
            end
        end

        if attempts >= max_attempts
            fprintf('WARNING: Random object %d could not find a fully clear spot -- using best available.\n', obj_i);
        end

        if is_rect
            rect_list = [rect_list; oxmin, oxmax, oymin, oymax];
        else
            circ_list = [circ_list; ocx, ocy, orad];
        end
    end

    rect_obs = rect_list;
    circ_obs = circ_list;

    rect_colors = repmat([0.45 0.50 0.60], size(rect_obs,1), 1);   % slate blue-grey
    circ_colors = repmat([0.70 0.35 0.25], size(circ_obs,1), 1);   % muted terracotta

    rect_names = repmat({'Random obstacle'}, size(rect_obs,1), 1);
    circ_names = repmat({'Random obstacle'}, size(circ_obs,1), 1);

    floor_plan_name = sprintf('%s (random layout)', floor_plan_name);

    % --- Random route, validated against the freshly placed obstacles ---
    num_wp       = 5;
    wp_list      = zeros(num_wp, 2);
    min_spacing  = 2.2;   % keeps waypoints from bunching into one corner
    placed_x     = start_x;
    placed_y     = start_y;
    prev_wx      = start_x;
    prev_wy      = start_y;

    for wp_i = 1:num_wp
        wvalid    = false;
        wattempts = 0;

        while ~wvalid && wattempts < max_attempts
            wattempts = wattempts + 1;

            cwx = wall_margin + (room_W - 2*wall_margin) * rand();
            cwy = wall_margin + (room_H - 2*wall_margin) * rand();

            cxn = min(max(cwx, rect_obs(:,1)), rect_obs(:,2));
            cyn = min(max(cwy, rect_obs(:,3)), rect_obs(:,4));
            dd_r = sqrt((cwx-cxn).^2 + (cwy-cyn).^2);
            dd_c = sqrt((cwx-circ_obs(:,1)).^2 + (cwy-circ_obs(:,2)).^2) - circ_obs(:,3);
            wvalid = all(dd_r >= 0.5) && all(dd_c >= 0.5);

            % Keep this waypoint away from every waypoint already placed,
            % not just the previous one - stops the route bunching into
            % one corner of the room
            if wvalid
                dd_p = sqrt((cwx-placed_x).^2 + (cwy-placed_y).^2);
                wvalid = all(dd_p >= min_spacing);
            end

            if wvalid
                for seg_t = 0:0.1:1
                    sx = prev_wx + seg_t*(cwx-prev_wx);
                    sy = prev_wy + seg_t*(cwy-prev_wy);
                    cxn = min(max(sx, rect_obs(:,1)), rect_obs(:,2));
                    cyn = min(max(sy, rect_obs(:,3)), rect_obs(:,4));
                    dd_r = sqrt((sx-cxn).^2 + (sy-cyn).^2);
                    dd_c = sqrt((sx-circ_obs(:,1)).^2 + (sy-circ_obs(:,2)).^2) - circ_obs(:,3);
                    if any(dd_r < 0.35) || any(dd_c < 0.35)
                        wvalid = false; break;
                    end
                end
            end
        end

        if wattempts >= max_attempts
            fprintf('WARNING: Random waypoint %d could not find a fully clear path -- using best available.\n', wp_i);
        end

        wp_list(wp_i, :) = [cwx, cwy];
        prev_wx  = cwx;
        prev_wy  = cwy;
        placed_x = [placed_x, cwx];
        placed_y = [placed_y, cwy];
    end

    waypoints = wp_list;

elseif scenario ~= 1 && scenario ~= 2
    fprintf('NOTE: Scenario %d is not recognised -- running Scenario 1 behaviour (fixed layout).\n', scenario);
end

num_rect      = size(rect_obs, 1);
num_circ      = size(circ_obs, 1);
num_waypoints = size(waypoints, 1);

% Safety fallback: if a branch above didn't set insets (e.g. Scenario 3's
% randomly generated furniture), everything defaults to fully solid.
if ~exist('rect_inset','var') || length(rect_inset) ~= num_rect
    rect_inset = zeros(num_rect,1);
end
if ~exist('circ_inset','var') || length(circ_inset) ~= num_circ
    circ_inset = zeros(num_circ,1);
end

% Safety fallback for legend names, same idea as the inset fallback above
if ~exist('rect_names','var') || length(rect_names) ~= num_rect
    rect_names = repmat({'Furniture'}, num_rect, 1);
end
if ~exist('circ_names','var') || length(circ_names) ~= num_circ
    circ_names = repmat({'Furniture'}, num_circ, 1);
end

% ---------------------------------------------------------
% DYNAMIC LEGEND ENTRIES - one colour swatch per DISTINCT
% object colour actually used in this run, so the legend
% always matches whatever furniture set the current floor
% plan / scenario produced (fixed or randomly generated)
% instead of a hard-coded list per plan.
% ---------------------------------------------------------
all_obj_colors = [rect_colors; circ_colors];
all_obj_names  = [rect_names;  circ_names];

furn_legend_colors = zeros(0,3);
furn_legend_names  = {};

for oi = 1:size(all_obj_colors,1)
    this_color = all_obj_colors(oi,:);
    is_new = true;
    for gi = 1:size(furn_legend_colors,1)
        if all(abs(furn_legend_colors(gi,:) - this_color) < 1e-6)
            is_new = false;
            break;
        end
    end
    if is_new
        furn_legend_colors = [furn_legend_colors; this_color];
        furn_legend_names{end+1} = all_obj_names{oi};
    end
end

% --- Collision geometry (what the robot actually reacts to) ---
rect_collision = rect_obs;
for k = 1:num_rect
    ins   = rect_inset(k);
    cxmin = rect_obs(k,1) + ins;
    cxmax = rect_obs(k,2) - ins;
    cymin = rect_obs(k,3) + ins;
    cymax = rect_obs(k,4) - ins;
    if cxmin >= cxmax
        midx = (rect_obs(k,1)+rect_obs(k,2))/2;
        cxmin = midx - 0.02; cxmax = midx + 0.02;
    end
    if cymin >= cymax
        midy = (rect_obs(k,3)+rect_obs(k,4))/2;
        cymin = midy - 0.02; cymax = midy + 0.02;
    end
    rect_collision(k,:) = [cxmin, cxmax, cymin, cymax];
end

circ_collision = circ_obs;
for j = 1:num_circ
    circ_collision(j,3) = max(0.05, circ_obs(j,3) - circ_inset(j));
end

% ---------------------------------------------------------
% SCENARIO MODE - dynamic people (Scenario 2 only, for now)
% ---------------------------------------------------------
person_radius      = 0.25;
person_speed_base  = 0.20;
arrival_threshold  = 0.15;
person_clearance   = person_radius + 0.30;   % buffer kept from furniture
max_place_attempts = 300;

if scenario == 2 || add_people

    num_people = max(0, 4 + add_extrapeople);

    pattern_names = {'Linear back-and-forth', 'Waypoint patrol loop', 'Continuous random wandering'};

    % Preallocate struct array fields
    people(num_people).points     = [];
    people(num_people).mode       = [];
    people(num_people).pattern    = [];
    people(num_people).target_idx = [];
    people(num_people).speed      = [];
    people(num_people).radius     = [];
    people(num_people).x          = [];
    people(num_people).y          = [];

    for pk = 1:num_people

        pattern_type = randi(3);
        if pattern_type == 1
            num_pts = 2; person_mode = 1;   % ping-pong
        elseif pattern_type == 2
            num_pts = 4; person_mode = 2;   % forward cycle (patrol loop)
        else
            num_pts = 2; person_mode = 3;   % true wander - regenerated live, see main loop
        end

        pts = zeros(num_pts, 2);

        for p_idx = 1:num_pts
            valid    = false;
            attempts = 0;

            while ~valid && attempts < max_place_attempts
                attempts = attempts + 1;

                cand_x = 0.8 + (room_W - 1.6) * rand();
                cand_y = 0.8 + (room_H - 1.6) * rand();

                % Point clear of every rectangle and circle at once,
                % instead of looping with an early break - same result:
                % valid only if EVERY obstacle is far enough away
                cxn = min(max(cand_x, rect_collision(:,1)), rect_collision(:,2));
                cyn = min(max(cand_y, rect_collision(:,3)), rect_collision(:,4));
                dd_r = sqrt((cand_x-cxn).^2 + (cand_y-cyn).^2);
                dd_c = sqrt((cand_x-circ_collision(:,1)).^2 + (cand_y-circ_collision(:,2)).^2) - circ_collision(:,3);
                valid = all(dd_r >= person_clearance) && all(dd_c >= person_clearance);

                % Segment from the previous point must also stay clear
                % (path check, not just the endpoint), sampled at 11
                % points; each sample is checked against every obstacle
                % in one vectorized pass
                if valid && p_idx > 1
                    prev_x = pts(p_idx-1, 1);
                    prev_y = pts(p_idx-1, 2);
                    for seg_t = 0:0.1:1
                        seg_x = prev_x + seg_t * (cand_x - prev_x);
                        seg_y = prev_y + seg_t * (cand_y - prev_y);
                        cxn = min(max(seg_x, rect_collision(:,1)), rect_collision(:,2));
                        cyn = min(max(seg_y, rect_collision(:,3)), rect_collision(:,4));
                        dd_r = sqrt((seg_x-cxn).^2 + (seg_y-cyn).^2);
                        dd_c = sqrt((seg_x-circ_collision(:,1)).^2 + (seg_y-circ_collision(:,2)).^2) - circ_collision(:,3);
                        if any(dd_r < person_radius) || any(dd_c < person_radius)
                            valid = false; break;
                        end
                    end
                end

                % For a cyclic pattern's LAST point, also check the
                % closing segment back to point 1
                if valid && person_mode == 2 && p_idx == num_pts
                    close_x = pts(1,1);
                    close_y = pts(1,2);
                    for seg_t = 0:0.1:1
                        seg_x = cand_x + seg_t * (close_x - cand_x);
                        seg_y = cand_y + seg_t * (close_y - cand_y);
                        cxn = min(max(seg_x, rect_collision(:,1)), rect_collision(:,2));
                        cyn = min(max(seg_y, rect_collision(:,3)), rect_collision(:,4));
                        dd_r = sqrt((seg_x-cxn).^2 + (seg_y-cyn).^2);
                        dd_c = sqrt((seg_x-circ_collision(:,1)).^2 + (seg_y-circ_collision(:,2)).^2) - circ_collision(:,3);
                        if any(dd_r < person_radius) || any(dd_c < person_radius)
                            valid = false; break;
                        end
                    end
                end
            end

            if attempts >= max_place_attempts
                fprintf('WARNING: Person %d waypoint %d could not find a fully clear spot after %d tries -- using best available.\n', ...
                        pk, p_idx, max_place_attempts);
            end

            pts(p_idx, :) = [cand_x, cand_y];
        end

        people(pk).points     = pts;
        people(pk).mode       = person_mode;
        people(pk).pattern    = pattern_type;
        people(pk).target_idx = 2;
        people(pk).speed      = person_speed_base + 0.10 * rand();
        people(pk).radius     = person_radius;
        people(pk).x          = pts(1,1);
        people(pk).y          = pts(1,2);
    end

else
    num_people = 0;
    people = struct('points',{},'mode',{},'pattern',{},'target_idx',{}, ...
                     'speed',{},'radius',{},'x',{},'y',{});
    if scenario ~= 1 && scenario ~= 3
        fprintf('NOTE: Scenario %d is not recognised -- running Scenario 1 behaviour (no dynamic people).\n', scenario);
    end
end

num_obs = num_rect + num_circ + num_people;

% ---------------------------------------------------------
% INITIAL STATE - robot enters through the doorway
% ---------------------------------------------------------
x     = start_x;
y     = start_y;
theta = theta_init;

% ---------------------------------------------------------
% WAYPOINT SETTINGS
% ---------------------------------------------------------
current_wp         = 1;
waypoint_threshold = 0.35;

% ---------------------------------------------------------
% CONTROL + SENSOR PARAMETERS
% ---------------------------------------------------------
v_forward        = 0.3;
Kp               = 2.0;
detection_radius = 1.2;
switch_margin    = 0.15;   % how much clearer the other side must be
                           % before the robot switches which way it's
                           % passing an obstacle (stops corridor flip-flop)
danger_zone_width = 0.4;   % start slowing down within this distance of
                           % any obstacle's surface
speed_floor       = 0.20;  % slowest the robot ever goes, right at the
                           % edge of an obstacle
render_every      = 8;     % draw a frame every N simulation steps (was
                           % 4). Purely a rendering-overhead knob

% ---------------------------------------------------------
% DEADLOCK RECOVERY
% ---------------------------------------------------------
% The avoidance law above is a tangential/bug-style method, and like
% any reactive local method it has a known failure mode: when an
% obstacle sits close on each side of a passage narrower than
% detection_radius, the two repulsion readings can point roughly
% opposite the goal direction, and switch_margin's hysteresis (there
% deliberately, to stop side flip-flopping) then keeps the robot
% committed to a side that can't make progress. Relying on hand-placed
% obstacle clearances to keep this from ever happening is fragile - it
% recurred the first time a layout edit tightened a gap without anyone
% noticing. This detects the deadlock from actual motion instead and
% forces an escape, so it's caught regardless of what the furniture
% layout happens to be.
stall_window       = 40;    % steps (2.0s @ dt=0.05) of position history
                           % kept to judge real progress, vs merely slow
stall_disp_thresh  = 0.08;  % m of net displacement over that window
                           % below which it's called a stall. 
                           % manoeuvring near obstacles doesn't trip it
stall_recovery_steps = 20;  % steps (1.0s) of forced, goal-ignoring
                           % avoidance heading once a stall triggers -
                           % long enough to physically clear a squeeze
stall_move_thresh     = 0.10;  % m a blocking object must shift from
                           % where it was when it first blocked us
                           % before we call it "moving" rather than
                           % "fixed". ]
stall_max_retries_static  = 4;   % retries allowed against something
                           % that hasn't moved at all since it first
                           % blocked us
stall_max_retries_dynamic = 10;  % retries allowed once the blocking
                           % object is confirmed to have moved during
                           % this same episode
% ---------------------------------------------------------
% STORAGE
% ---------------------------------------------------------
x_path    = zeros(1, N);
y_path    = zeros(1, N);
mode_log  = zeros(1, N);
wp_log    = zeros(1, N);

people_path_x = zeros(max(num_people,1), N);
people_path_y = zeros(max(num_people,1), N);

min_clearance   = inf;
encounter_count = 0;
encounter_log   = zeros(200, 4);

wp_reached_times = zeros(1, num_waypoints);

% ---------------------------------------------------------
% FIGURE 1: MAIN SIMULATION WINDOW
% ---------------------------------------------------------
fig1 = figure('Name', 'Route Inspection Simulation', ...
              'NumberTitle', 'off');
set(fig1, 'Position', [50 50 800 800]);

hold on;
grid on;
axis equal;
axis([-1 room_W+1 -1 room_H+1]);

fill([0, room_W, room_W, 0], [0, 0, room_H, room_H], [0.95 0.95 0.95], ...
     'EdgeColor', 'none', 'FaceAlpha', 0.15);

if strcmpi(floor_plan, 'D')
    % Floor Plan D only: door is on the LEFT wall at the top-left
    % corner, not on the bottom wall. Bottom wall is fully solid.
    plot([0, room_W], [0, 0], '-', 'Color', [0.2 0.2 0.6], 'LineWidth', 3);
    plot([0, 0], [0, door_bottom], '-', 'Color', [0.2 0.2 0.6], 'LineWidth', 3);
    plot([0, 0], [door_top, room_H], '-', 'Color', [0.2 0.2 0.6], 'LineWidth', 3);
    plot([0, room_W], [room_H, room_H], '-', 'Color', [0.2 0.2 0.6], 'LineWidth', 3);
    plot([room_W, room_W], [0, room_H], '-', 'Color', [0.2 0.2 0.6], 'LineWidth', 3);

    plot([0, -0.3], [door_bottom, door_bottom], '-', 'Color', [0 0.7 0], 'LineWidth', 2.5);
    plot([0, -0.3], [door_top,    door_top],    '-', 'Color', [0 0.7 0], 'LineWidth', 2.5);
    text(-0.55, (door_bottom + door_top)/2, 'DOOR', ...
         'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', [0 0.6 0], ...
         'FontWeight', 'bold', 'Rotation', 90);
else
    plot([0, door_left], [0, 0], '-', 'Color', [0.2 0.2 0.6], 'LineWidth', 3);
    plot([door_right, room_W], [0, 0], '-', 'Color', [0.2 0.2 0.6], 'LineWidth', 3);
    plot([0, 0], [0, room_H], '-', 'Color', [0.2 0.2 0.6], 'LineWidth', 3);
    plot([0, room_W], [room_H, room_H], '-', 'Color', [0.2 0.2 0.6], 'LineWidth', 3);
    plot([room_W, room_W], [0, room_H], '-', 'Color', [0.2 0.2 0.6], 'LineWidth', 3);

    plot([door_left,  door_left],  [0, -0.3], '-', 'Color', [0 0.7 0], 'LineWidth', 2.5);
    plot([door_right, door_right], [0, -0.3], '-', 'Color', [0 0.7 0], 'LineWidth', 2.5);
    text((door_left + door_right)/2, -0.55, 'DOOR', ...
         'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', [0 0.6 0], 'FontWeight', 'bold');
end

if strcmpi(floor_plan, 'C')
    text(2.0, 4.0, 'CONFERENCE ROOM', 'HorizontalAlignment', 'center', ...
         'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.55 0.25 0.30]);
    text(6.0, 2.0, 'MANAGER''S OFFICE', 'HorizontalAlignment', 'center', ...
         'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.25 0.30 0.45]);
    text(2.5, 5.8, 'BREAK ROOM', 'HorizontalAlignment', 'center', ...
         'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.80 0.50 0.25]);
end

if num_people > 0
    people_tag = ' + People';
else
    people_tag = '';
end

text(room_W/2, -0.85, sprintf('%s (%.0fm x %.0fm) - Floor Plan %s, Scenario %d%s', ...
     floor_plan_name, room_W, room_H, upper(floor_plan), scenario, people_tag), ...
     'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', [0.2 0.2 0.6], 'FontWeight', 'bold');

xlabel('X Position (m)', 'FontSize', 11);
ylabel('Y Position (m)', 'FontSize', 11);
title(sprintf('Autonomous Indoor Route Inspection Robot - Floor Plan %s, Scenario %d%s', ...
      upper(floor_plan), scenario, people_tag), 'FontSize', 13, 'FontWeight', 'bold');

theta_circle = linspace(0, 2*pi, 100);

% --- Draw rectangular furniture ---
for k = 1:num_rect
    xmin = rect_obs(k, 1); xmax = rect_obs(k, 2);
    ymin = rect_obs(k, 3); ymax = rect_obs(k, 4);

    h_rect = rectangle('Position', [xmin, ymin, xmax-xmin, ymax-ymin], ...
              'Curvature', 0.15, 'FaceColor', rect_colors(k,:), ...
              'EdgeColor', 'k', 'LineWidth', 1.5);

    if rect_inset(k) > 0
        set(h_rect, 'FaceAlpha', 0.45);   % passable furniture - reads as "not fully solid"
    end

    text((xmin+xmax)/2, (ymin+ymax)/2, sprintf('O%d', k), ...
         'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'white');

    % Rounded-rectangle detection boundary: straight edges offset by
    % detection_radius, quarter-circle arcs at the corners, built around
    % the true COLLISION footprint (so passable furniture correctly
    % shows a smaller ring than its visual size would suggest)
    cxmin = rect_collision(k,1); cxmax = rect_collision(k,2);
    cymin = rect_collision(k,3); cymax = rect_collision(k,4);
    dr = detection_radius;

    th_tr = linspace(0, pi/2, 15);
    th_tl = linspace(pi/2, pi, 15);
    th_bl = linspace(pi, 3*pi/2, 15);
    th_br = linspace(3*pi/2, 2*pi, 15);

    rrx = [cxmax+dr*cos(th_tr), cxmin+dr*cos(th_tl), cxmin+dr*cos(th_bl), cxmax+dr*cos(th_br)];
    rry = [cymax+dr*sin(th_tr), cymax+dr*sin(th_tl), cymin+dr*sin(th_bl), cymin+dr*sin(th_br)];

    plot([rrx, rrx(1)], [rry, rry(1)], 'r--', 'LineWidth', 0.8);
end

% --- Draw circular furniture ---
for j = 1:num_circ
    cx  = circ_obs(j, 1); cy = circ_obs(j, 2); rad = circ_obs(j, 3);
    gid = num_rect + j;

    if circ_inset(j) > 0
        face_alpha = 0.45;
    else
        face_alpha = 1.0;
    end

    fill(cx + rad * cos(theta_circle), cy + rad * sin(theta_circle), ...
         circ_colors(j,:), 'EdgeColor', 'k', 'LineWidth', 1.5, 'FaceAlpha', face_alpha);

    text(cx, cy, sprintf('O%d', gid), ...
         'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'white');

    % Detection ring drawn around the true collision radius, not the
    % visual one - matches where avoidance actually starts reacting
    coll_rad = circ_collision(j,3);
    plot(cx + detection_radius * cos(theta_circle), cy + detection_radius * sin(theta_circle), ...
         'r--', 'LineWidth', 0.8);
end

% --- Draw moving people (initial positions) ---
h_people_body   = gobjects(1, num_people);
h_people_detect = gobjects(1, num_people);
for pk = 1:num_people
    h_people_body(pk) = fill(people(pk).x + person_radius*cos(theta_circle), ...
                              people(pk).y + person_radius*sin(theta_circle), ...
                              [0.85 0.10 0.55], 'EdgeColor', 'k', 'LineWidth', 1.2);
    h_people_detect(pk) = plot(people(pk).x + detection_radius*cos(theta_circle), ...
                                people(pk).y + detection_radius*sin(theta_circle), ...
                                'm:', 'LineWidth', 0.6);
end

% --- Draw planned route ---
route_x = [start_x, waypoints(:,1)'];
route_y = [start_y, waypoints(:,2)'];
plot(route_x, route_y, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.2);

% --- Draw waypoints ---
for w = 1:num_waypoints
    if w < num_waypoints
        plot(waypoints(w,1), waypoints(w,2), 'd', 'MarkerSize', 11, ...
             'MarkerFaceColor', [0 0.85 0.85], 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
        text(waypoints(w,1) + 0.2, waypoints(w,2), sprintf('WP%d', w), ...
             'FontSize', 9, 'Color', [0 0.6 0.6], 'FontWeight', 'bold');
    else
        plot(waypoints(w,1), waypoints(w,2), 'p', 'MarkerSize', 18, ...
             'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
        text(waypoints(w,1) + 0.2, waypoints(w,2), 'GOAL', ...
             'FontSize', 11, 'Color', [0 0.7 0], 'FontWeight', 'bold');
    end
end

% --- Start marker ---
plot(start_x, start_y, 's', 'MarkerSize', 12, 'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'k');
text(start_x - 0.9, start_y + 0.3, 'START', 'FontSize', 10, 'Color', [0 0.6 0], 'FontWeight', 'bold');

% --- Animated handles ---
h_seek_trail  = plot(NaN, NaN, 'b-',  'LineWidth', 2.5);
h_avoid_trail = plot(NaN, NaN, '-',   'LineWidth', 2.5, 'Color', [1 0.4 0]);
h_robot       = plot(NaN, NaN, 'ko',  'MarkerSize', 9, 'MarkerFaceColor', [0.2 0.55 1], 'LineWidth', 1.5);
h_head        = plot(NaN, NaN, 'k-',  'LineWidth', 2.2);
h_furn_radius = plot(NaN, NaN, 'r--', 'LineWidth', 0.8);
h_people_radius = plot(NaN, NaN, 'm:', 'LineWidth', 0.6);

% --- Legend-only dummy handles ---
% These plot NaN (nothing actually shows on the map) purely so every
% marker type gets its own correctly-shaped, correctly-coloured entry
% in the legend. People get a CIRCLE marker here (not the rectangular
% patch swatch MATLAB would otherwise auto-generate from h_people_body),
% and every distinct furniture colour used in this run gets its own
% square swatch, built from furn_legend_colors/furn_legend_names above.
h_start_legend  = plot(NaN, NaN, 's', 'MarkerSize', 10, 'MarkerFaceColor', [0 0.8 0],       'MarkerEdgeColor', 'k');
h_wp_legend     = plot(NaN, NaN, 'd', 'MarkerSize', 9,  'MarkerFaceColor', [0 0.85 0.85],   'MarkerEdgeColor', 'k');
h_goal_legend   = plot(NaN, NaN, 'p', 'MarkerSize', 12, 'MarkerFaceColor', [0 0.8 0],       'MarkerEdgeColor', 'k');
h_person_legend = plot(NaN, NaN, 'o', 'MarkerSize', 9,  'MarkerFaceColor', [0.85 0.10 0.55],'MarkerEdgeColor', 'k');

num_furn_legend = size(furn_legend_colors, 1);
h_furn_legend   = gobjects(1, num_furn_legend);
for fl = 1:num_furn_legend
    h_furn_legend(fl) = plot(NaN, NaN, 's', 'MarkerSize', 10, ...
        'MarkerFaceColor', furn_legend_colors(fl,:), 'MarkerEdgeColor', 'k');
end

if strcmpi(floor_plan, 'D')
    % Default status-text spot (top-left) is now where the door and
    % START label live for this floor plan - moved to open floor
    % space at the bottom instead (O7 no longer sits there).
    h_mode   = text(5.2, 1.0, 'Mode: INITIALISING', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');
    h_wptxt  = text(5.2, 0.6, 'Target: WP1', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0.6 0.6]);
    h_timetxt = text(5.2, 0.2, 'Time: 0.00 s', 'FontSize', 10, 'Color', [0.3 0.3 0.3]);
else
    h_mode   = text(0.1, 9.4, 'Mode: INITIALISING', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');
    h_wptxt  = text(-0.8, 8.8, 'Target: WP1', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0.6 0.6]);
    h_timetxt = text(-0.8, 8.2, 'Time: 0.00 s', 'FontSize', 10, 'Color', [0.3 0.3 0.3]);
end

% --- Build the legend dynamically so it always lists exactly what's
% actually on the map: paths/robot, start/waypoint/goal markers, every
% distinct furniture colour in this run, and (if present) people ---
legend_handles = [h_seek_trail, h_avoid_trail, h_robot, ...
                   h_start_legend, h_wp_legend, h_goal_legend, ...
                   h_furn_legend, h_furn_radius];
legend_labels  = [{'Goal-Seeking Path', 'Avoidance Path', 'Robot', ...
                   'Start', 'Waypoint', 'Goal'}, ...
                   furn_legend_names, {'Furniture Detection Radius'}];

if num_people > 0
    legend_handles = [legend_handles, h_person_legend, h_people_radius];
    legend_labels  = [legend_labels, {'Person (moving)', 'Person Detection Radius'}];
end

legend(legend_handles, legend_labels, 'Location', 'eastoutside', 'FontSize', 8);

% ---------------------------------------------------------
% SEGMENT STORAGE for colour-coded trail
% ---------------------------------------------------------
seek_x  = zeros(1, N);   seek_y  = zeros(1, N);   seek_n  = 0;
avoid_x = zeros(1, N);   avoid_y = zeros(1, N);   avoid_n = 0;

% ---------------------------------------------------------
% MAIN SIMULATION LOOP
% ---------------------------------------------------------
reached_goal  = false;
route_blocked = false;   % true only if stall_max_retries is exceeded -
                          % distinguishes "genuinely stuck" from "ran
                          % out of simulated time" in the final report
steps_taken   = N;
avoid_side    = 0;       % 0 = undecided, 1 = leaning left, -1 = leaning right
prev_detected = false;   % tracks true detection onset, independent of mode_log

pos_hist_x            = nan(1, stall_window);
pos_hist_y            = nan(1, stall_window);
recovery_countdown    = 0;   % >0 while a forced escape is in progress
forced_side           = 0;   % avoid_side value held during that escape
stall_retry_count     = 0;   % consecutive triggers with no real progress
stall_recovery_count  = 0;   % total triggers over the whole run, for the report
stall_recovery_log    = zeros(50, 3);   % [time, x, y] per trigger, capped at 50

stall_first_id     = 0;      % which object first blocked us this episode
stall_first_x      = 0;      % ...and where it was standing right then
stall_first_y      = 0;
stall_is_dynamic   = false;  % latches true the moment that same object
                              % is caught having moved during this episode

for i = 1:N

    % --- Deadlock-recovery bookkeeping: record position and measure
    % net progress over the last stall_window steps, before anything
    % else touches x/y this iteration ---
    hist_slot = mod(i-1, stall_window) + 1;
    if i > stall_window
        net_disp = sqrt((x - pos_hist_x(hist_slot))^2 + (y - pos_hist_y(hist_slot))^2);
    else
        net_disp = inf;   % not enough history yet - assume fine
    end
    pos_hist_x(hist_slot) = x;
    pos_hist_y(hist_slot) = y;

    x_target = waypoints(current_wp, 1);
    y_target = waypoints(current_wp, 2);

    dist_to_wp = sqrt((x_target - x)^2 + (y_target - y)^2);

    if dist_to_wp < waypoint_threshold
        wp_reached_times(current_wp) = i * dt;

        if current_wp == num_waypoints
            fprintf('>>> GOAL reached at t = %.2f s\n', i * dt);
            steps_taken  = i;
            reached_goal = true;
            x_path(i)    = x;
            y_path(i)    = y;
            wp_log(i)    = current_wp;
            break;
        else
            fprintf('>>> Waypoint %d reached at t = %.2f s\n', current_wp, i * dt);
            plot(waypoints(current_wp,1), waypoints(current_wp,2), 'd', ...
                 'MarkerSize', 11, 'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'k');

            current_wp = current_wp + 1;
            x_target   = waypoints(current_wp, 1);
            y_target   = waypoints(current_wp, 2);

            if current_wp == num_waypoints
                set(h_wptxt, 'String', 'Target: GOAL');
            else
                set(h_wptxt, 'String', sprintf('Target: WP%d', current_wp));
            end
        end
    end

    % --- Move each person along their assigned pattern ---
    for pk = 1:num_people
        tgt = people(pk).points(people(pk).target_idx, :);
        dx  = tgt(1) - people(pk).x;
        dy  = tgt(2) - people(pk).y;
        dist_to_target = sqrt(dx^2 + dy^2);

        if dist_to_target < arrival_threshold
            if people(pk).mode == 1
                people(pk).target_idx = 3 - people(pk).target_idx;   % toggle 1<->2

            elseif people(pk).mode == 2
                people(pk).target_idx = people(pk).target_idx + 1;
                if people(pk).target_idx > size(people(pk).points, 1)
                    people(pk).target_idx = 1;
                end

            else
                % mode == 3: true wandering - regenerate a fresh,
                % validated random target from the person's current spot
                w_valid    = false;
                w_attempts = 0;

                while ~w_valid && w_attempts < max_place_attempts
                    w_attempts = w_attempts + 1;

                    w_cand_x = 0.8 + (room_W - 1.6) * rand();
                    w_cand_y = 0.8 + (room_H - 1.6) * rand();

                    cxn = min(max(w_cand_x, rect_collision(:,1)), rect_collision(:,2));
                    cyn = min(max(w_cand_y, rect_collision(:,3)), rect_collision(:,4));
                    dd_r = sqrt((w_cand_x-cxn).^2 + (w_cand_y-cyn).^2);
                    dd_c = sqrt((w_cand_x-circ_collision(:,1)).^2 + (w_cand_y-circ_collision(:,2)).^2) - circ_collision(:,3);
                    w_valid = all(dd_r >= person_clearance) && all(dd_c >= person_clearance);

                    if w_valid
                        for seg_t = 0:0.1:1
                            seg_x = people(pk).x + seg_t * (w_cand_x - people(pk).x);
                            seg_y = people(pk).y + seg_t * (w_cand_y - people(pk).y);
                            cxn = min(max(seg_x, rect_collision(:,1)), rect_collision(:,2));
                            cyn = min(max(seg_y, rect_collision(:,3)), rect_collision(:,4));
                            dd_r = sqrt((seg_x-cxn).^2 + (seg_y-cyn).^2);
                            dd_c = sqrt((seg_x-circ_collision(:,1)).^2 + (seg_y-circ_collision(:,2)).^2) - circ_collision(:,3);
                            if any(dd_r < person_radius) || any(dd_c < person_radius)
                                w_valid = false; break;
                            end
                        end
                    end
                end

                people(pk).points(people(pk).target_idx, :) = [w_cand_x, w_cand_y];
                % target_idx is left unchanged - the person keeps heading
                % at whichever row it already points to, now refreshed
            end

            tgt = people(pk).points(people(pk).target_idx, :);
            dx  = tgt(1) - people(pk).x;
            dy  = tgt(2) - people(pk).y;
            dist_to_target = sqrt(dx^2 + dy^2);
        end

        if dist_to_target > 1e-6
            step = min(people(pk).speed * dt, dist_to_target);
            people(pk).x = people(pk).x + step * dx / dist_to_target;
            people(pk).y = people(pk).y + step * dy / dist_to_target;
        end

        people_path_x(pk, i) = people(pk).x;
        people_path_y(pk, i) = people(pk).y;
    end

    % --- Sensor: composite repulsion from ALL detected objects ---
    obstacle_detected = false;
    min_dist          = inf;
    nearest_id        = 0;
    repulse_x         = 0;
    repulse_y         = 0;
    speed_scale       = 1.0;

    % Rectangles
    for k = 1:num_rect
        xmin = rect_collision(k,1); xmax = rect_collision(k,2);
        ymin = rect_collision(k,3); ymax = rect_collision(k,4);

        cxn = min(max(x, xmin), xmax);
        cyn = min(max(y, ymin), ymax);
        d   = sqrt((x-cxn)^2 + (y-cyn)^2);

        if d < min_clearance
            min_clearance = d;
        end

        if d < detection_radius
            obstacle_detected = true;

            if d > 1e-6
                dirx = (x-cxn)/d;
                diry = (y-cyn)/d;
            else
                dirx = 0; diry = 1;
            end

            weight    = 1 / max(d, 0.01);
            repulse_x = repulse_x + weight * dirx;
            repulse_y = repulse_y + weight * diry;

            if d < min_dist
                min_dist   = d;
                nearest_id = k;
            end

            if d < danger_zone_width
                new_scale   = max(d / danger_zone_width, speed_floor);
                speed_scale = min(speed_scale, new_scale);
            end
        end
    end

    % Circles (static furniture)
    for j = 1:num_circ
        cx  = circ_collision(j,1); cy = circ_collision(j,2); rad = circ_collision(j,3);
        dc  = sqrt((x-cx)^2 + (y-cy)^2);
        d   = dc - rad;

        if d < min_clearance
            min_clearance = d;
        end

        if d < detection_radius
            obstacle_detected = true;

            dirx = (x-cx)/max(dc, 0.001);
            diry = (y-cy)/max(dc, 0.001);

            weight    = 1 / max(d, 0.01);
            repulse_x = repulse_x + weight * dirx;
            repulse_y = repulse_y + weight * diry;

            gid = num_rect + j;
            if d < min_dist
                min_dist   = d;
                nearest_id = gid;
            end

            if d < danger_zone_width
                new_scale   = max(d / danger_zone_width, speed_floor);
                speed_scale = min(speed_scale, new_scale);
            end
        end
    end

    % Moving people
    for pk = 1:num_people
        cx  = people(pk).x; cy = people(pk).y; rad = people(pk).radius;
        dc  = sqrt((x-cx)^2 + (y-cy)^2);
        d   = dc - rad;

        if d < min_clearance
            min_clearance = d;
        end

        if d < detection_radius
            obstacle_detected = true;

            dirx = (x-cx)/max(dc, 0.001);
            diry = (y-cy)/max(dc, 0.001);

            weight    = 1 / max(d, 0.01);
            repulse_x = repulse_x + weight * dirx;
            repulse_y = repulse_y + weight * diry;

            gid = num_rect + num_circ + pk;
            if d < min_dist
                min_dist   = d;
                nearest_id = gid;
            end

            if d < danger_zone_width
                new_scale   = max(d / danger_zone_width, speed_floor);
                speed_scale = min(speed_scale, new_scale);
            end
        end
    end

    if obstacle_detected && ~prev_detected
        encounter_count = encounter_count + 1;
        if encounter_count <= 200
            encounter_log(encounter_count, :) = [i, nearest_id, min_dist, current_wp];
        end
    end
    prev_detected = obstacle_detected;

    % --- Deadlock check: only meaningful once genuinely near something,
    % and only once any earlier forced escape has finished ---
    if recovery_countdown == 0 && obstacle_detected && net_disp < stall_disp_thresh
        stall_recovery_count = stall_recovery_count + 1;
        stall_retry_count    = stall_retry_count + 1;
        if stall_recovery_count <= 50
            stall_recovery_log(stall_recovery_count, :) = [i*dt, x, y];
        end

        % First trigger of a fresh episode: note which object blocked us
        % and exactly where it was standing at that moment
        if stall_retry_count == 1
            stall_first_id   = nearest_id;
            stall_is_dynamic = false;
            if stall_first_id > (num_rect + num_circ)
                stall_first_x = people(stall_first_id - num_rect - num_circ).x;
                stall_first_y = people(stall_first_id - num_rect - num_circ).y;
            elseif stall_first_id > num_rect
                stall_first_x = circ_obs(stall_first_id - num_rect, 1);
                stall_first_y = circ_obs(stall_first_id - num_rect, 2);
            else
                stall_first_x = (rect_obs(stall_first_id,1) + rect_obs(stall_first_id,2)) / 2;
                stall_first_y = (rect_obs(stall_first_id,3) + rect_obs(stall_first_id,4)) / 2;
            end
        end

        % Every trigger: has that SAME object moved since we first got
        % stuck on it? This is a live position comparison, not a lookup
        % of which array it came from - furniture always fails this
        % check (its coordinates are literally constant), a person
        % passes it the moment they've actually shifted position
        if stall_first_id > (num_rect + num_circ)
            cur_ox = people(stall_first_id - num_rect - num_circ).x;
            cur_oy = people(stall_first_id - num_rect - num_circ).y;
        elseif stall_first_id > num_rect
            cur_ox = circ_obs(stall_first_id - num_rect, 1);
            cur_oy = circ_obs(stall_first_id - num_rect, 2);
        else
            cur_ox = (rect_obs(stall_first_id,1) + rect_obs(stall_first_id,2)) / 2;
            cur_oy = (rect_obs(stall_first_id,3) + rect_obs(stall_first_id,4)) / 2;
        end
        obj_moved = sqrt((cur_ox-stall_first_x)^2 + (cur_oy-stall_first_y)^2);
        if obj_moved > stall_move_thresh
            stall_is_dynamic = true;
        end

        if stall_is_dynamic
            retry_cap = stall_max_retries_dynamic;
            cap_label = 'moving obstacle - patient cap';
        else
            retry_cap = stall_max_retries_static;
            cap_label = 'fixed obstacle - strict cap';
        end

        forced_side = -avoid_side;
        if forced_side == 0
            forced_side = 1;   % avoid_side was still undecided - just pick a side
        end
        recovery_countdown = stall_recovery_steps;
        pos_hist_x(:) = nan; pos_hist_y(:) = nan;   % don't re-trigger mid-escape

        if forced_side == 1
            side_label = 'left';
        else
            side_label = 'right';
        end
        fprintf('>>> Deadlock detected near [%.2f, %.2f] at t=%.2fs - forcing a %s-side escape (attempt %d/%d, %s)\n', ...
                x, y, i*dt, side_label, stall_retry_count, retry_cap, cap_label);

        if stall_retry_count > retry_cap
            fprintf('>>> Route BLOCKED: %d recoveries near [%.2f, %.2f] with no progress between them - stopping early.\n', ...
                    stall_retry_count, x, y);
            route_blocked = true;
            steps_taken    = i;
            x_path(i) = x; y_path(i) = y; wp_log(i) = current_wp;
            break;
        end
    elseif net_disp >= stall_disp_thresh
        stall_retry_count = 0;   % real progress happened - clear the counter
        stall_first_id    = 0;
        stall_is_dynamic  = false;
    end

    % --- Control law: proportional blending, not a hard switch ---
    theta_goal = atan2(y_target - y, x_target - x);

    if obstacle_detected
        perp_left   = [-repulse_y,  repulse_x];
        perp_right  = [ repulse_y, -repulse_x];
        repulse_mag = sqrt(repulse_x^2 + repulse_y^2);

        vec_to_goal      = [x_target - x, y_target - y];
        dist_to_goal_vec = sqrt(vec_to_goal(1)^2 + vec_to_goal(2)^2);

        if repulse_mag > 1e-6 && dist_to_goal_vec > 1e-6
            cos_left  = dot(perp_left,  vec_to_goal) / (repulse_mag * dist_to_goal_vec);
            cos_right = dot(perp_right, vec_to_goal) / (repulse_mag * dist_to_goal_vec);
        else
            cos_left  = 0;
            cos_right = 0;
        end

        % Sticky side choice - only switch sides if the other side is
        % CLEARLY better, not just marginally, to stop the corridor-stall
        % flip-flop when obstacles sit roughly evenly on both sides.
        % During a forced escape (recovery_countdown > 0) that
        % hysteresis is exactly what caused the deadlock in the first
        % place, so it's bypassed and the flipped side held instead.
        if recovery_countdown > 0
            avoid_side = forced_side;
        elseif avoid_side == 0
            if cos_left >= cos_right
                avoid_side = 1;
            else
                avoid_side = -1;
            end
        elseif avoid_side == 1
            if cos_right > cos_left + switch_margin
                avoid_side = -1;
            end
        else
            if cos_left > cos_right + switch_margin
                avoid_side = 1;
            end
        end

        if avoid_side == 1
            theta_steer = atan2(perp_left(2),  perp_left(1));
        else
            theta_steer = atan2(perp_right(2), perp_right(1));
        end

        % How urgent is avoidance right now? 0 = only just detected,
        % 1 = right up against the nearest surface. Forced to 1 during
        % a recovery so the goal direction plays no part until clear.
        if recovery_countdown > 0
            blend_weight = 1;
        else
            blend_weight = 1 - (min_dist / detection_radius);
            blend_weight = max(0, min(1, blend_weight));
        end

        % Blend the two headings via their unit vectors, not the raw
        % angles, so the interpolation never fights angle-wrap issues
        goal_vec  = [cos(theta_goal),  sin(theta_goal)];
        avoid_vec = [cos(theta_steer), sin(theta_steer)];
        blend_vec = (1 - blend_weight) * goal_vec + blend_weight * avoid_vec;

        if norm(blend_vec) > 1e-6
            theta_cmd = atan2(blend_vec(2), blend_vec(1));
        else
            theta_cmd = theta_goal;   % degenerate cancellation - fall back
        end

        e_cmd = atan2(sin(theta_cmd - theta), cos(theta_cmd - theta));
        omega = Kp * e_cmd;

        if nearest_id > (num_rect + num_circ)
            near_label = sprintf('P%d', nearest_id - (num_rect + num_circ));
        else
            near_label = sprintf('O%d', nearest_id);
        end

        if blend_weight > 0.5
            mode_log(i) = 1;
            avoid_n = avoid_n + 1;
            avoid_x(avoid_n) = x;
            avoid_y(avoid_n) = y;
            if recovery_countdown > 0
                set(h_mode, 'String', sprintf('Mode: DEADLOCK RECOVERY (%.1fs left)', recovery_countdown*dt), 'Color', [0.7 0 0.7]);
            else
                set(h_mode, 'String', sprintf('Mode: AVOIDING %s (%.0f%%)', near_label, blend_weight*100), 'Color', [1 0.3 0]);
            end
        else
            mode_log(i) = 0;
            seek_n = seek_n + 1;
            seek_x(seek_n) = x;
            seek_y(seek_n) = y;
            set(h_mode, 'String', sprintf('Mode: GOAL-SEEKING (%.0f%% avoid)', blend_weight*100), 'Color', 'blue');
        end
    else
        avoid_side = 0;   % clear the sticky choice for the next encounter

        e_theta     = atan2(sin(theta_goal - theta), cos(theta_goal - theta));
        omega       = Kp * e_theta;
        mode_log(i) = 0;

        seek_n = seek_n + 1;
        seek_x(seek_n) = x;
        seek_y(seek_n) = y;

        set(h_mode, 'String', 'Mode: GOAL-SEEKING', 'Color', 'blue');
    end

    if recovery_countdown > 0
        recovery_countdown = recovery_countdown - 1;
    end

    % --- Kinematic update ---
    x     = x     + v_forward * speed_scale * cos(theta) * dt;
    y     = y     + v_forward * speed_scale * sin(theta) * dt;
    theta = theta + omega * dt;

    % --- Hard boundary vs rectangles ---
    for k = 1:num_rect
        xmin = rect_collision(k,1); xmax = rect_collision(k,2);
        ymin = rect_collision(k,3); ymax = rect_collision(k,4);

        cxn = min(max(x, xmin), xmax);
        cyn = min(max(y, ymin), ymax);
        d   = sqrt((x-cxn)^2 + (y-cyn)^2);

        if d < 0.15
            if d > 1e-6
                px = (x-cxn)/d;
                py = (y-cyn)/d;
                x  = cxn + px*0.15;
                y  = cyn + py*0.15;
            else
                dist_left  = x - xmin;
                dist_right = xmax - x;
                dist_bot   = y - ymin;
                dist_top   = ymax - y;
                m = min([dist_left, dist_right, dist_bot, dist_top]);
                if m == dist_left
                    x = xmin - 0.15;
                elseif m == dist_right
                    x = xmax + 0.15;
                elseif m == dist_bot
                    y = ymin - 0.15;
                else
                    y = ymax + 0.15;
                end
            end
        end
    end

    % --- Hard boundary vs circles ---
    for j = 1:num_circ
        cx  = circ_collision(j,1); cy = circ_collision(j,2); rad = circ_collision(j,3);
        dc  = sqrt((x-cx)^2 + (y-cy)^2);
        if dc < (rad + 0.15)
            push_dir = [(x-cx), (y-cy)] / max(dc, 0.001);
            x = cx + push_dir(1) * (rad + 0.15);
            y = cy + push_dir(2) * (rad + 0.15);
        end
    end

    % --- Hard boundary vs people (robot only; people don't react) ---
    for pk = 1:num_people
        cx  = people(pk).x; cy = people(pk).y; rad = people(pk).radius;
        dc  = sqrt((x-cx)^2 + (y-cy)^2);
        if dc < (rad + 0.15)
            push_dir = [(x-cx), (y-cy)] / max(dc, 0.001);
            x = cx + push_dir(1) * (rad + 0.15);
            y = cy + push_dir(2) * (rad + 0.15);
        end
    end

    % --- Store ---
    x_path(i) = x;
    y_path(i) = y;
    wp_log(i) = current_wp;

    % --- Animation ---
    if seek_n > 0
        set(h_seek_trail, 'XData', seek_x(1:seek_n), 'YData', seek_y(1:seek_n));
    end
    if avoid_n > 0
        set(h_avoid_trail, 'XData', avoid_x(1:avoid_n), 'YData', avoid_y(1:avoid_n));
    end

    set(h_robot, 'XData', x, 'YData', y);

    arrow_len = 0.35;
    set(h_head, 'XData', [x, x + arrow_len * cos(theta)], 'YData', [y, y + arrow_len * sin(theta)]);

    for pk = 1:num_people
        set(h_people_body(pk), ...
            'XData', people(pk).x + person_radius*cos(theta_circle), ...
            'YData', people(pk).y + person_radius*sin(theta_circle));
        set(h_people_detect(pk), ...
            'XData', people(pk).x + detection_radius*cos(theta_circle), ...
            'YData', people(pk).y + detection_radius*sin(theta_circle));
    end

    set(h_timetxt, 'String', sprintf('Time: %.2f s', i * dt));

    if mod(i, render_every) == 0
        drawnow;
    end

end

% ---------------------------------------------------------
% FINAL FRAME
% ---------------------------------------------------------
x_path        = x_path(1:steps_taken);
y_path        = y_path(1:steps_taken);
mode_log      = mode_log(1:steps_taken);
people_path_x = people_path_x(:, 1:steps_taken);
people_path_y = people_path_y(:, 1:steps_taken);

if reached_goal
    plot(x, y, 'g*', 'MarkerSize', 22, 'LineWidth', 2.5);
    set(h_mode, 'String', 'GOAL REACHED -- Route: PASS', 'Color', [0 0.6 0]);
elseif route_blocked
    plot(x, y, 'rx', 'MarkerSize', 18, 'LineWidth', 3);
    set(h_mode, 'String', 'ROUTE BLOCKED -- deadlock recovery exhausted', 'Color', [0.8 0 0]);
end

drawnow;

% ---------------------------------------------------------
% FIGURE 2: POST-SIMULATION ANALYSIS (static, 2 subplots)
% ---------------------------------------------------------
fig2 = figure('Name', 'Route Inspection Analysis', 'NumberTitle', 'off');
set(fig2, 'Position', [100 100 1000 420]);

time_axis = (1:steps_taken) * dt;

subplot(1, 2, 1);
hold on;
grid on;

for i = 1:steps_taken
    if mode_log(i) == 0
        plot([time_axis(i) time_axis(i)], [0 1], 'b-', 'LineWidth', 1.2);
    else
        plot([time_axis(i) time_axis(i)], [0 1], '-', 'LineWidth', 1.2, 'Color', [1 0.4 0]);
    end
end

ylim([0 1.2]);
set(gca, 'YTick', [0 1], 'YTickLabel', {'Goal-Seeking', 'Avoidance'});
xlabel('Time (s)', 'FontSize', 10);
title('Robot Mode Over Time', 'FontSize', 11, 'FontWeight', 'bold');

for w = 1:num_waypoints
    if wp_reached_times(w) > 0
        plot([wp_reached_times(w) wp_reached_times(w)], [0 1.2], 'g--', 'LineWidth', 1.5);
        text(wp_reached_times(w), 1.15, sprintf('WP%d', w), ...
             'FontSize', 8, 'Color', [0 0.6 0], 'HorizontalAlignment', 'center');
    end
end

subplot(1, 2, 2);
hold on;
grid on;

colors_obs  = lines(num_obs);
dist_matrix = zeros(num_obs, steps_taken);

for k = 1:num_rect
    xmin = rect_collision(k,1); xmax = rect_collision(k,2);
    ymin = rect_collision(k,3); ymax = rect_collision(k,4);
    for i = 1:steps_taken
        cxn = min(max(x_path(i), xmin), xmax);
        cyn = min(max(y_path(i), ymin), ymax);
        dist_matrix(k,i) = sqrt((x_path(i)-cxn)^2 + (y_path(i)-cyn)^2);
    end
    plot(time_axis, dist_matrix(k, :), 'Color', colors_obs(k,:), 'LineWidth', 1.5);
end

for j = 1:num_circ
    cx  = circ_collision(j,1); cy = circ_collision(j,2); rad = circ_collision(j,3);
    gid = num_rect + j;
    for i = 1:steps_taken
        dc = sqrt((x_path(i)-cx)^2 + (y_path(i)-cy)^2);
        dist_matrix(gid,i) = dc - rad;
    end
    plot(time_axis, dist_matrix(gid, :), 'Color', colors_obs(gid,:), 'LineWidth', 1.5);
end

for pk = 1:num_people
    gid = num_rect + num_circ + pk;
    for i = 1:steps_taken
        dc = sqrt((x_path(i)-people_path_x(pk,i))^2 + (y_path(i)-people_path_y(pk,i))^2);
        dist_matrix(gid,i) = dc - people(pk).radius;
    end
    plot(time_axis, dist_matrix(gid, :), 'Color', colors_obs(gid,:), 'LineWidth', 1.5, 'LineStyle', '--');
end

plot([time_axis(1) time_axis(end)], [detection_radius detection_radius], 'k--', 'LineWidth', 2);

xlabel('Time (s)', 'FontSize', 10);
ylabel('Distance to Object (m)', 'FontSize', 10);
title('Sensor Distance Readings', 'FontSize', 11, 'FontWeight', 'bold');

leg_labels = cell(1, num_obs + 1);
for k = 1:(num_rect + num_circ)
    leg_labels{k} = sprintf('Object O%d', k);
end
for pk = 1:num_people
    leg_labels{num_rect + num_circ + pk} = sprintf('Person P%d', pk);
end
leg_labels{end} = 'Detection Threshold';
legend(leg_labels, 'Location', 'eastoutside', 'FontSize', 7);

% ---------------------------------------------------------
% INSPECTION REPORT - Command Window
% ---------------------------------------------------------
path_length   = sum(sqrt(diff(x_path).^2 + diff(y_path).^2));
straight_dist = sqrt((waypoints(end,1) - start_x)^2 + (waypoints(end,2) - start_y)^2);

if path_length > 0
    efficiency = (straight_dist / path_length) * 100;
else
    efficiency = 0;
end

avoidance_pct = (sum(mode_log) / steps_taken) * 100;

fprintf('\n');
fprintf('====================================================\n');
fprintf(' ROUTE INSPECTION REPORT\n');
fprintf(' Floor Plan %s (%s) - Scenario %d\n', upper(floor_plan), floor_plan_name, scenario);
fprintf('====================================================\n');
fprintf(' NAVIGATION SUMMARY\n');
fprintf('====================================================\n');
fprintf('  Goal Reached         : %s\n',     mat2str(reached_goal));
fprintf('  Waypoints Visited    : %d / %d\n', min(current_wp, num_waypoints), num_waypoints);
fprintf('  Total Time           : %.2f s\n',  steps_taken * dt);
fprintf('  Path Length          : %.3f m\n',  path_length);
fprintf('  Straight-Line Dist   : %.3f m\n',  straight_dist);
fprintf('  Path Efficiency      : %.1f%%\n',  efficiency);
fprintf('  Avoidance Time       : %.1f%% of journey\n', avoidance_pct);
fprintf('  Min Obstacle Clearance: %.3f m\n', min_clearance);
fprintf('\n');

if num_people > 0
    fprintf('====================================================\n');
    fprintf(' DYNAMIC OBSTACLES (SCENARIO 2)\n');
    fprintf('====================================================\n');
    for pk = 1:num_people
        fprintf('  Person P%d : %s (speed %.2f m/s)\n', pk, pattern_names{people(pk).pattern}, people(pk).speed);
    end
    fprintf('\n');
end

fprintf('====================================================\n');
fprintf(' WAYPOINT LOG\n');
fprintf('====================================================\n');
for w = 1:num_waypoints
    if wp_reached_times(w) > 0
        if w == num_waypoints
            fprintf('  GOAL   [%.1f, %.1f]  : reached at t = %.2f s\n', waypoints(w,1), waypoints(w,2), wp_reached_times(w));
        else
            fprintf('  WP%-2d   [%.1f, %.1f]  : reached at t = %.2f s\n', w, waypoints(w,1), waypoints(w,2), wp_reached_times(w));
        end
    else
        fprintf('  WP%-2d   [%.1f, %.1f]  : NOT REACHED\n', w, waypoints(w,1), waypoints(w,2));
    end
end
fprintf('\n');
fprintf('====================================================\n');
fprintf(' OBSTACLE ENCOUNTER LOG\n');
fprintf('====================================================\n');
if encounter_count == 0
    fprintf('  No obstacle encounters recorded.\n');
else
    for e = 1:encounter_count
        obs_id = encounter_log(e,2);
        if obs_id > (num_rect + num_circ)
            e_label = sprintf('P%d', obs_id - (num_rect + num_circ));
        else
            e_label = sprintf('O%d', obs_id);
        end
        fprintf('  Encounter %d : %s detected at t=%.2fs  (dist=%.3fm)  near WP%d\n', ...
                e, e_label, encounter_log(e,1) * dt, encounter_log(e,3), encounter_log(e,4));
    end
end
fprintf('\n');
fprintf('====================================================\n');
fprintf(' DEADLOCK RECOVERY LOG\n');
fprintf('====================================================\n');
if stall_recovery_count == 0
    fprintf('  No deadlocks detected - no forced recoveries were needed.\n');
else
    fprintf('  Recoveries triggered  : %d\n', stall_recovery_count);
    for r = 1:min(stall_recovery_count, 50)
        fprintf('  Recovery %d : t=%.2fs at [%.2f, %.2f]\n', ...
                r, stall_recovery_log(r,1), stall_recovery_log(r,2), stall_recovery_log(r,3));
    end
    if stall_recovery_count > 50
        fprintf('  ... (%d more, log capped at 50 entries)\n', stall_recovery_count - 50);
    end
end
fprintf('\n');
fprintf('====================================================\n');
fprintf(' SAFETY ASSESSMENT\n');
fprintf('====================================================\n');

if min_clearance < 0.2
    safety_flag = 'HIGH RISK -- clearance critically low';
elseif min_clearance < 0.5
    safety_flag = 'MODERATE RISK -- clearance acceptable';
else
    safety_flag = 'LOW RISK -- clearance satisfactory';
end

fprintf('  Min Clearance        : %.3f m\n', min_clearance);
fprintf('  Safety Assessment    : %s\n',     safety_flag);
fprintf('\n');
fprintf('====================================================\n');
fprintf(' ROUTE FEASIBILITY\n');
fprintf('====================================================\n');

if reached_goal
    fprintf('  Feasibility Result   : *** PASS ***\n');
    fprintf('  Route is traversable from START to GOAL.\n');
    if stall_recovery_count > 0
        fprintf('  (%d deadlock(s) were hit en route and recovered from automatically.)\n', stall_recovery_count);
    end
elseif route_blocked
    fprintf('  Feasibility Result   : *** FAIL (BLOCKED) ***\n');
    fprintf('  Robot hit a deadlock near [%.2f, %.2f] it could not recover from\n', x, y);
    fprintf('  after %d attempts.\n', stall_retry_count);
    if stall_is_dynamic
        fprintf('  The blocking object was moving (confirmed - it shifted during the\n');
        fprintf('  episode), so this is most likely unlucky timing with a person\n');
        fprintf('  rather than a real gap problem. Re-run, or raise\n');
        fprintf('  stall_max_retries_dynamic, before concluding the layout is at fault.\n');
    else
        fprintf('  The blocking object never moved, so this is a real layout problem:\n');
        fprintf('  the gap there is narrower than the robot can physically fit\n');
        fprintf('  through. Widen it or re-route around it.\n');
    end
else
    fprintf('  Feasibility Result   : *** FAIL ***\n');
    fprintf('  Robot did not reach the goal.\n');
end

fprintf('====================================================\n\n');

