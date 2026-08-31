%% ================================================================ 
% BOX THERMAL SIMULATION - assumes all material is PLA  
% Moving forward, we could do a resistance network and do the same thing 
% for a box as it has easier "meshing" in matlab than an assembly, or we 
% could do a union of an assembly with predefined materials 
% The box rotates about the z-axis, and this causes a convection 
% 
% The laser follows the center path about the middle (waist) of the drone 
% Going forward, we could have it focus on a specific corner (etc.) as well 
% as implement the appropriate convection of each motor  
%  
% The top and bottom faces are disregarded  
% 
% ================================================================ 


clear; 
clc; 
close all; 


% USER CONTROLS 
STEP_FILE = 'basic_drone.step';

rotation_frequencies = [0 1 2 3 4 5 10 20 40 60];

% Number of frequency cases
nFreq = numel(rotation_frequencies); 

t_final = 8; 

dt = 0.02; 

tlist = 0:dt:t_final; 


% INITIAL TEMPERATURE 
T0 = 293.15; 

Tinf = 293.15; 


% FEM MESH - can play around with this to get quicker results 
meshSize = 0.05; 


% PLA MATERIAL - need to cross reference  
PLA.rho = 1240; 

PLA.cp = 1800; 

PLA.k = 0.13; 

PLA.emissivity = 0.90; 

PLA.absorptivity = 0.80; 


% AIR PROPERTIES - need to cross reference 
air_density = 1.225; 

air_viscosity = 1.81e-5; 

air_conductivity = 0.026; 

air_prandtl = 0.71; 


% CONVECTION PARAMETERS 
L_characteristic = 0.3302; 

h_free = 0.0; % can mess with this later and couple with the actual rotation of the drone


% STEFAN-BOLTZMANN CONSTANT 
sigma = 5.670374419e-8; 


% LASER PARAMETERS - once we determine feasibility of the system, these can 
% be played around with and iterated over to create the state space 
P0 = 5000; 

ALPHA_ATM = 1.65e-4; 

WAVELENGTH = 1064e-9; 

D = 0.1; 

M2 = 1.5; 


% LASER ABSORPTION 
laser_absorptivity = PLA.absorptivity; 


% LASER ORIGIN 
laser_origin_world = [0.0; 0.0; 0.0]; 


% LASER INITIAL ANGLE - leads to different patterns, can play with this too 
% to explore state space  
theta_0 = 0; 


% IMPORT GEOMETRY 
fprintf("\n"); 

fprintf(" IMPORTING BOX GEOMETRY\n"); 

g = fegeometry(STEP_FILE); 

fprintf("Number of cells: %d\n", g.NumCells); 

fprintf("Number of faces: %d\n", g.NumFaces); 


% DISPLAY GEOMETRY 
figure("Name", "Box Geometry"); 

pdegplot(g, FaceLabels="on", CellLabels="on", FaceAlpha=0.25); 

axis equal; 

grid on; 

xlabel("x [m]"); 

ylabel("y [m]"); 

zlabel("z [m]"); 

title("Box Geometry"); 


% CREATE FEM MODEL 
model = femodel(AnalysisType="thermalTransient", Geometry=g); 

model.StefanBoltzmann = sigma; 


% ASSIGN PLA TO ENTIRE BOX - in the future this would be an assignment on a 
% part by part basis 
PLA_CELLS = 1:g.NumCells; 

model.MaterialProperties(PLA_CELLS) = materialProperties(ThermalConductivity=PLA.k, MassDensity=PLA.rho, SpecificHeat=PLA.cp); 


% INITIAL TEMPERATURE 
model.CellIC = cellIC(Temperature=T0); 


% GENERATE FEM MESH 
fprintf("\n"); 

fprintf(" GENERATING FEM MESH\n"); 

model = generateMesh(model, Hmax=meshSize); 

fprintf("Number of nodes: %d\n", size(model.Geometry.Mesh.Nodes, 2)); 

fprintf("Number of elements: %d\n", size(model.Geometry.Mesh.Elements, 2)); 


% DISPLAY MESH 
figure("Name", "Box FEM Mesh"); 

pdemesh(model.Geometry); 

axis equal; 

grid on; 

title("Box FEM Mesh"); 


% DETERMINE BOX DIMENSIONS 
nodes = model.Geometry.Mesh.Nodes; 

x_nodes = nodes(1, :); 

y_nodes = nodes(2, :); 

z_nodes = nodes(3, :); 

x_min = min(x_nodes); 

x_max = max(x_nodes); 

y_min = min(y_nodes); 

y_max = max(y_nodes); 

z_min = min(z_nodes); 

z_max = max(z_nodes); 

box_length = x_max - x_min; 

box_width = y_max - y_min; 

box_height = z_max - z_min; 


% BOX CENTROID 
box_centroid = [(x_min + x_max) / 2; 
                (y_min + y_max) / 2; 
                (z_min + z_max) / 2]; 


fprintf("\n"); 

fprintf(" BOX GEOMETRY\n");  

fprintf("Length: %.4f m\n", box_length); 

fprintf("Width: %.4f m\n", box_width); 

fprintf("Height: %.4f m\n", box_height); 

fprintf("Centroid: [%.4f, %.4f, %.4f] m\n", box_centroid(1), box_centroid(2), box_centroid(3)); 


% LASER TRAJECTORY 
% The laser travels around the centroid at the midpoint height. 
trajectory_radius = min(box_length, box_width) / 2 - 0.010; 

laser_target_z = box_centroid(3); 


% DISPLAY LASER TRAJECTORY 
trajectory_theta = linspace(0, 2*pi, 500); 

trajectory_x = box_centroid(1) + trajectory_radius .* cos(trajectory_theta); 

trajectory_y = box_centroid(2) + trajectory_radius .* sin(trajectory_theta); 

trajectory_z = laser_target_z .* ones(size(trajectory_theta)); 


figure("Name", "Laser Trajectory"); 

pdegplot(model.Geometry, FaceAlpha=0.15); 

hold on; 

plot3(trajectory_x, trajectory_y, trajectory_z, "LineWidth", 2); 

scatter3(box_centroid(1), box_centroid(2), box_centroid(3), 50, "filled"); 

axis equal; 

grid on; 

xlabel("x [m]"); 

ylabel("y [m]"); 

zlabel("z [m]"); 

title("Laser Trajectory Around Box Centroid"); 

legend({"Box", "Laser trajectory", "Box centroid"}); 


% CALCULATE FACE CENTROIDS 
faceCentroids = zeros(g.NumFaces, 3); 

for faceID = 1:g.NumFaces 

    nodeIDs = findNodes(model.Geometry.Mesh, "region", "Face", faceID); 

    xyz = model.Geometry.Mesh.Nodes(:, nodeIDs)'; 

    faceCentroids(faceID, :) = mean(xyz, 1); 

end 


% IDENTIFY TOP AND BOTTOM FACES 
z_tolerance = 0.01 * box_height; 

top_faces = find(abs(faceCentroids(:, 3) - z_max) < z_tolerance); 

bottom_faces = find(abs(faceCentroids(:, 3) - z_min) < z_tolerance); 


% IDENTIFY SIDE FACES 
all_faces = (1:g.NumFaces)'; 

excluded_laser_faces = unique([top_faces; bottom_faces]); 

side_faces = setdiff(all_faces, excluded_laser_faces); 


fprintf("\n"); 

fprintf(" FACE IDENTIFICATION\n"); 

fprintf("Top faces:\n"); 

disp(top_faces'); 

fprintf("Bottom faces:\n"); 

disp(bottom_faces'); 

fprintf("Side faces:\n"); 

disp(side_faces'); 


% SURFACE TRIANGULATION FOR LASER ENERGY INTEGRATION
% Integrate the absorbed heat flux [W/m^2] over the exposed side surface
% to obtain instantaneous laser power [W] and cumulative laser energy [J].
tetElements = model.Geometry.Mesh.Elements;
meshNodes = model.Geometry.Mesh.Nodes;
if size(tetElements, 1) < 4
    error("Unexpected FEM mesh: each tetrahedral element must have at least 4 nodes.");
end

% triangulation() requires 4 corner nodes per tetrahedron.
% Quadratic FEM tetrahedra can contain 10 nodes, so use the 4 corner nodes.
tetCornerElements = tetElements(1:4, :)';
TR = triangulation(tetCornerElements, meshNodes');
[boundaryTriangles, ~] = freeBoundary(TR);

boundaryTriangleCentroids = (meshNodes(:, boundaryTriangles(:,1)) + ...
    meshNodes(:, boundaryTriangles(:,2)) + meshNodes(:, boundaryTriangles(:,3)))' / 3;

boundarySideMask = boundaryTriangleCentroids(:,3) > (z_min + z_tolerance) & ...
                   boundaryTriangleCentroids(:,3) < (z_max - z_tolerance);

sideBoundaryTriangles = boundaryTriangles(boundarySideMask, :);

p1 = meshNodes(:, sideBoundaryTriangles(:,1));
p2 = meshNodes(:, sideBoundaryTriangles(:,2));
p3 = meshNodes(:, sideBoundaryTriangles(:,3));
sideTriangleAreas = 0.5 .* vecnorm(cross((p2-p1)', (p3-p1)', 2), 2, 2);
sideNodeIDs = unique(sideBoundaryTriangles(:));
sideSurfaceAreaTotal = sum(sideTriangleAreas);

fprintf("\n");
fprintf(" LASER ENERGY SURFACE\n");
fprintf("Side surface triangles: %d\n", size(sideBoundaryTriangles, 1));
fprintf("Side surface area: %.6f m^2\n", sideSurfaceAreaTotal);


% RESULTS STORAGE
maxTemperature = zeros(nFreq, 1);
finalTemperature = zeros(nFreq, 1);
averageFinalTemperature = zeros(nFreq, 1);
averageH = zeros(nFreq, 1);
timeOfMaximum = zeros(nFreq, 1);

% ENERGY / LASER METRICS
maxLaserPower = zeros(nFreq, 1);              % Maximum absorbed laser power [W]
totalLaserEnergy = zeros(nFreq, 1);           % Total absorbed laser energy [J]
finalStoredThermalEnergy = zeros(nFreq, 1);   % Thermal energy above T0 at final time [J]
maxStoredThermalEnergy = zeros(nFreq, 1);     % Maximum thermal energy above T0 [J]
sideSurfaceArea = zeros(nFreq, 1);             % Laser-exposed side area [m^2]

allResults = cell(nFreq, 1); 



% FREQUENCY SWEEP 
for freqIndex = 1:nFreq 

    frequency_Hz = rotation_frequencies(freqIndex); 

    omega_z = 2 * pi * frequency_Hz; 

    fprintf("\n\n"); 

    fprintf(" RUN %d OF %d\n", freqIndex, nFreq); 

    fprintf("Frequency: %.2f Hz\n", frequency_Hz); 

    fprintf("Angular velocity: %.4f rad/s\n", omega_z); 

    fprintf("RPM: %.2f\n", frequency_Hz * 60); 

    fprintf("Total rotations: %.2f\n", frequency_Hz * t_final);

    sideSurfaceArea(freqIndex) = sideSurfaceAreaTotal;

    % LASER POWER HISTORY: integrate absorbed heat flux over the exposed
    % side surface at every requested solution time.
    laserPowerTime = zeros(size(tlist));

    for timeIndex = 1:numel(tlist)
        currentTime = tlist(timeIndex);

        % Evaluate the laser flux at every mesh node so the boundary
        % triangle node IDs can be used directly below.
        location = struct( ...
            "x", meshNodes(1, :), ...
            "y", meshNodes(2, :), ...
            "z", meshNodes(3, :));

        state = struct("time", currentTime);

        qNodes = laserHeatFluxCircular(location, state, ...
            box_centroid, trajectory_radius, omega_z, theta_0, ...
            laser_origin_world, P0, ALPHA_ATM, WAVELENGTH, D, M2, ...
            laser_absorptivity);

        % Piecewise-linear surface integration: use the mean flux at the
        % three vertices of each boundary triangle.
        qTriangle = mean(qNodes(sideBoundaryTriangles), 2);
        laserPowerTime(timeIndex) = sum(qTriangle .* sideTriangleAreas);
    end

    maxLaserPower(freqIndex) = max(laserPowerTime);
    totalLaserEnergy(freqIndex) = trapz(tlist, laserPowerTime);


    % APPLY CONVECTION AND RADIATION 
    for faceID = 1:g.NumFaces 

        model.FaceLoad(faceID) = faceLoad(ConvectionCoefficient=@(location, state) convectionCoefficientRotation(location, omega_z, box_centroid, air_density, air_viscosity, air_conductivity, air_prandtl, L_characteristic, h_free), AmbientTemperature=Tinf, Emissivity=PLA.emissivity); 

    end 


    % APPLY LASER TO SIDE FACES 
    for faceIndex = 1:length(side_faces) 

        faceID = side_faces(faceIndex); 

        model.FaceLoad(faceID) = faceLoad(Heat=@(location, state) laserHeatFluxCircular(location, state, box_centroid, trajectory_radius, omega_z, theta_0, laser_origin_world, P0, ALPHA_ATM, WAVELENGTH, D, M2, laser_absorptivity), ConvectionCoefficient=@(location, state) convectionCoefficientRotation(location, omega_z, box_centroid, air_density, air_viscosity, air_conductivity, air_prandtl, L_characteristic, h_free), AmbientTemperature=Tinf, Emissivity=PLA.emissivity); 

    end 


    % SOLVE THERMAL MODEL - takes the longest time - could maybe try to 
    % delegate more cores - don't know if this is possible in Matlab 
    fprintf("\n"); 

    fprintf("Solving thermal model...\n"); 

    thermalResults = solve(model, tlist); 

    fprintf("Solution complete.\n"); 


    % TEMPERATURE
    T = thermalResults.Temperature;

    % STORED THERMAL ENERGY ABOVE THE INITIAL TEMPERATURE.
    % For linear tetrahedral elements, the element-average temperature is
    % the mean of its four nodal temperatures.
    a = meshNodes(:, tetElements(2,:)) - meshNodes(:, tetElements(1,:));
    b = meshNodes(:, tetElements(3,:)) - meshNodes(:, tetElements(1,:));
    c = meshNodes(:, tetElements(4,:)) - meshNodes(:, tetElements(1,:));
    tetVolumes = abs(dot(a, cross(b, c, 1), 1)) ./ 6;

    thermalEnergyTime = zeros(size(tlist));
    for timeIndex = 1:numel(tlist)
        tetTemperature = mean(T(tetElements(1:4,:), timeIndex), 1);
        thermalEnergyTime(timeIndex) = sum(PLA.rho .* PLA.cp .* ...
            (tetTemperature - T0) .* tetVolumes);
    end

    finalStoredThermalEnergy(freqIndex) = thermalEnergyTime(end);
    maxStoredThermalEnergy(freqIndex) = max(thermalEnergyTime);


    % MAXIMUM TEMPERATURE THROUGH TIME 
    TmaxTime = max(T, [], 1); 


    % MAXIMUM TEMPERATURE 
    maxTemperature(freqIndex) = max(TmaxTime); 


    % FINAL MAXIMUM TEMPERATURE 
    finalTemperature(freqIndex) = TmaxTime(end); 


    % AVERAGE FINAL TEMPERATURE 
    averageFinalTemperature(freqIndex) = mean(T(:, end)); 


    % TIME OF MAXIMUM TEMPERATURE 
    [~, maxIndex] = max(TmaxTime); 

    timeOfMaximum(freqIndex) = tlist(maxIndex); 


    % CALCULATE AVERAGE CONVECTION 
    nodeX = model.Geometry.Mesh.Nodes(1, :); 

    nodeY = model.Geometry.Mesh.Nodes(2, :); 

    radialDistance = sqrt((nodeX - box_centroid(1)).^2 + (nodeY - box_centroid(2)).^2); 

    velocity = abs(omega_z) .* radialDistance; 

    Re = air_density .* velocity .* L_characteristic ./ air_viscosity; 

    Re = max(Re, 1); 

    Nu = 0.664 .* sqrt(Re) .* air_prandtl.^(1 / 3); 

    turbulent = Re >= 5e5; 

    Nu(turbulent) = (0.037 .* Re(turbulent).^0.8 - 871) .* air_prandtl.^(1 / 3); 

    h_nodes = h_free + Nu .* air_conductivity ./ L_characteristic; 

    averageH(freqIndex) = mean(h_nodes); 


    % STORE RESULTS 
    allResults{freqIndex} = thermalResults; 


    % PRINT RESULTS 
    fprintf("\n"); 

    fprintf(" RESULTS: %.2f Hz\n", frequency_Hz); 

    fprintf("Maximum temperature: %.2f K\n", maxTemperature(freqIndex)); 

    fprintf("Maximum temperature: %.2f C\n", maxTemperature(freqIndex) - 273.15); 

    fprintf("Final maximum temperature: %.2f C\n", finalTemperature(freqIndex) - 273.15); 

    fprintf("Average final temperature: %.2f C\n", averageFinalTemperature(freqIndex) - 273.15); 

    fprintf("Average convection coefficient: %.2f W/(m^2*K)\n", averageH(freqIndex)); 

    fprintf("Time of maximum temperature: %.2f s\n", timeOfMaximum(freqIndex));
fprintf("Maximum absorbed laser power: %.4f W\n", maxLaserPower(freqIndex));
fprintf("Total laser energy absorbed by side surface: %.4f J\n", totalLaserEnergy(freqIndex));
fprintf("Final stored thermal energy above T0: %.4f J\n", finalStoredThermalEnergy(freqIndex));
fprintf("Maximum stored thermal energy above T0: %.4f J\n", maxStoredThermalEnergy(freqIndex));


    % FINAL TEMPERATURE PLOT 
    figure("Name", sprintf("Temperature %.2f Hz", frequency_Hz)); 

    pdeplot3D(thermalResults.Mesh, ColorMapData=T(:, end)); 

    axis equal; 

    grid on; 

    colorbar; 

    xlabel("x [m]"); 

    ylabel("y [m]"); 

    zlabel("z [m]"); 

    title(sprintf("Final Temperature - %.2f Hz", frequency_Hz)); 


    % TEMPERATURE HISTORY 
    figure("Name", sprintf("Temperature History %.2f Hz", frequency_Hz)); 

    plot(tlist, TmaxTime - 273.15, "LineWidth", 2); 

    grid on; 

    xlabel("Time [s]"); 

    ylabel("Maximum Temperature [C]"); 

    title(sprintf("Maximum Temperature - %.2f Hz", frequency_Hz));

    % LASER POWER AND ENERGY HISTORY
    figure("Name", sprintf("Laser Energy %.2f Hz", frequency_Hz));
    yyaxis left;
    plot(tlist, laserPowerTime, "LineWidth", 2);
    ylabel("Absorbed Laser Power [W]");
    yyaxis right;
    plot(tlist, cumtrapz(tlist, laserPowerTime), "LineWidth", 2);
    ylabel("Cumulative Laser Energy [J]");
    grid on;
    xlabel("Time [s]");
    title(sprintf("Laser Energy - %.2f Hz", frequency_Hz));

    % STORED THERMAL ENERGY HISTORY
    figure("Name", sprintf("Thermal Energy %.2f Hz", frequency_Hz));
    plot(tlist, thermalEnergyTime, "LineWidth", 2);
    grid on;
    xlabel("Time [s]");
    ylabel("Stored Thermal Energy Above T0 [J]");
    title(sprintf("Stored Thermal Energy - %.2f Hz", frequency_Hz));

end


% RESULTS TABLE
% Force EVERY result to be an N-by-1 column before constructing the table.
% This prevents row/column shape mismatches and avoids the older MATLAB
% VariableNames parsing issue.
N = numel(rotation_frequencies);

frequencyColumn = reshape(rotation_frequencies, N, 1);
rpmColumn = reshape(rotation_frequencies .* 60, N, 1);
angularVelocityColumn = reshape(2 .* pi .* rotation_frequencies, N, 1);
maximumTemperatureColumn = reshape(maxTemperature - 273.15, N, 1);
finalMaximumTemperatureColumn = reshape(finalTemperature - 273.15, N, 1);
averageFinalTemperatureColumn = reshape(averageFinalTemperature - 273.15, N, 1);
averageHColumn = reshape(averageH, N, 1);
timeOfMaximumColumn = reshape(timeOfMaximum, N, 1);
maxLaserPowerColumn = reshape(maxLaserPower, N, 1);
totalLaserEnergyColumn = reshape(totalLaserEnergy, N, 1);
finalStoredThermalEnergyColumn = reshape(finalStoredThermalEnergy, N, 1);
maxStoredThermalEnergyColumn = reshape(maxStoredThermalEnergy, N, 1);
sideSurfaceAreaColumn = reshape(sideSurfaceArea, N, 1);

% Explicitly build the table first, then assign names.  This works across
% MATLAB versions and guarantees that VariableNames is not interpreted as
% a data variable.
resultsTable = table( ...
    frequencyColumn, rpmColumn, angularVelocityColumn, ...
    maximumTemperatureColumn, finalMaximumTemperatureColumn, ...
    averageFinalTemperatureColumn, averageHColumn, timeOfMaximumColumn, ...
    maxLaserPowerColumn, totalLaserEnergyColumn, ...
    finalStoredThermalEnergyColumn, maxStoredThermalEnergyColumn, ...
    sideSurfaceAreaColumn);

resultsTable.Properties.VariableNames = { ...
    "Frequency_Hz", ...
    "RPM", ...
    "AngularVelocity_rad_s", ...
    "MaximumTemperature_C", ...
    "FinalMaximumTemperature_C", ...
    "AverageFinalTemperature_C", ...
    "AverageConvection_W_m2K", ...
    "TimeOfMaximum_s", ...
    "MaxAbsorbedLaserPower_W", ...
    "TotalLaserEnergy_J", ...
    "FinalStoredThermalEnergy_J", ...
    "MaxStoredThermalEnergy_J", ...
    "LaserSurfaceArea_m2"};

% VERIFY RESULTS
fprintf("\n");
fprintf(" RESULTS TABLE: %d rows x %d columns\n", height(resultsTable), width(resultsTable));

% DISPLAY RESULTS
fprintf("\n\n");
fprintf(" FINAL FREQUENCY SWEEP RESULTS\n");
disp(resultsTable);


% ENERGY SUMMARY
fprintf("\n");
fprintf(" ENERGY SUMMARY\n");
fprintf("Maximum absorbed laser power across sweep: %.4f W\n", max(maxLaserPower));
fprintf("Maximum total laser energy across sweep: %.4f J\n", max(totalLaserEnergy));
fprintf("Maximum stored thermal energy across sweep: %.4f J\n", max(maxStoredThermalEnergy));


% MAXIMUM TEMPERATURE VS FREQUENCY 
figure("Name", "Maximum Temperature vs Frequency"); 

plot(rotation_frequencies, maxTemperature - 273.15, "-o", "LineWidth", 2); 

grid on; 

xlabel("Rotation Frequency [Hz]"); 

ylabel("Maximum Temperature [C]"); 

title("Maximum Temperature vs Rotation Frequency"); 


% FINAL TEMPERATURE VS FREQUENCY 
figure("Name", "Final Temperature vs Frequency"); 

plot(rotation_frequencies, finalTemperature - 273.15, "-o", "LineWidth", 2); 

grid on; 

xlabel("Rotation Frequency [Hz]"); 

ylabel("Final Maximum Temperature [C]"); 

title("Final Maximum Temperature vs Rotation Frequency"); 


% CONVECTION VS FREQUENCY 
figure("Name", "Rotational Convection vs Frequency"); 

plot(rotation_frequencies, averageH, "-o", "LineWidth", 2); 

grid on; 

xlabel("Rotation Frequency [Hz]"); 

ylabel("Average h [W/(m^2*K)]"); 

title("Rotational Convection vs Rotation Frequency");

% MAXIMUM ABSORBED LASER POWER VS FREQUENCY
figure("Name", "Max Laser Power vs Frequency");
plot(rotation_frequencies, maxLaserPower, "-o", "LineWidth", 2);
grid on;
xlabel("Rotation Frequency [Hz]");
ylabel("Maximum Absorbed Laser Power [W]");
title("Maximum Laser Power vs Frequency");

% TOTAL LASER ENERGY VS FREQUENCY
figure("Name", "Total Laser Energy vs Frequency");
plot(rotation_frequencies, totalLaserEnergy, "-o", "LineWidth", 2);
grid on;
xlabel("Rotation Frequency [Hz]");
ylabel("Total Absorbed Laser Energy [J]");
title("Total Laser Energy vs Frequency");

% STORED THERMAL ENERGY VS FREQUENCY
figure("Name", "Stored Thermal Energy vs Frequency");
plot(rotation_frequencies, finalStoredThermalEnergy, "-o", "LineWidth", 2);
grid on;
xlabel("Rotation Frequency [Hz]");
ylabel("Final Stored Thermal Energy Above T0 [J]");
title("Stored Thermal Energy vs Frequency");


% SAVE RESULTS 
save("box_thermal_frequency_sweep.mat", "allResults", "resultsTable", ...
    "rotation_frequencies", "maxTemperature", "finalTemperature", ...
    "averageFinalTemperature", "averageH", "timeOfMaximum", ...
    "maxLaserPower", "totalLaserEnergy", "finalStoredThermalEnergy", ...
    "maxStoredThermalEnergy", "sideSurfaceArea", "box_centroid", ...
    "trajectory_radius"); 

writetable(resultsTable, "box_thermal_frequency_results.csv"); 


fprintf("\n"); 

fprintf(" RESULTS SAVED\n");  

fprintf("box_thermal_frequency_sweep.mat\n"); 

fprintf("box_thermal_frequency_results.csv\n");
fprintf("\nEnergy metrics include maximum absorbed laser power, total absorbed laser energy,\n");
fprintf("final/max stored thermal energy above the initial temperature, and laser surface area.\n"); 


% LASER HEAT FLUX 
function q = laserHeatFluxCircular(location, state, box_centroid, trajectory_radius, omega_z, theta_0, laser_origin_world, P0, ALPHA_ATM, WAVELENGTH, D, M2, absorptivity) 


    if isempty(state.time) 

        t = 0; 

    else 

        t = state.time; 

    end 


    % LASER POSITION 
    theta = theta_0 + omega_z * t; 

    target = [box_centroid(1) + trajectory_radius * cos(theta); 
              box_centroid(2) + trajectory_radius * sin(theta); 
              box_centroid(3)]; 


    % LASER DIRECTION 
    laser_direction = target - laser_origin_world; 

    range = norm(laser_direction); 

    laser_direction = laser_direction / max(range, 1e-12); 


    % BEAM WAIST 
    w0 = D / (pi * M2); 


    % BEAM WIDTH 
    w = sqrt(w0^2 + (WAVELENGTH * range)^2); 


    % ATMOSPHERIC ATTENUATION 
    I0 = P0 * exp(-ALPHA_ATM * range); 


    % SURFACE POSITIONS 
    xyz = [location.x(:)'; 
           location.y(:)'; 
           location.z(:)']; 


    % POSITION RELATIVE TO LASER TARGET 
    relative = xyz - target; 


    % CONSTRUCT TRANSVERSE LASER BASIS 
    referenceVector = [0; 0; 1]; 


    if abs(dot(referenceVector, laser_direction)) > 0.95 

        referenceVector = [0; 1; 0]; 

    end 


    e1 = cross(laser_direction, referenceVector); 

    e1 = e1 / max(norm(e1), 1e-12); 


    e2 = cross(laser_direction, e1); 

    e2 = e2 / max(norm(e2), 1e-12); 


    % TRANSVERSE COORDINATES 
    X = e1' * relative; 

    Y = e2' * relative; 


    % RADIAL DISTANCE 
    r2 = X.^2 + Y.^2; 


    % GAUSSIAN LASER INTENSITY 
    I = (2 * I0 ./ (pi * w^2)) .* exp(-2 * r2 ./ w^2); 


    % ABSORBED HEAT FLUX 
    q = absorptivity .* I; 

    q = reshape(q, 1, []); 

end 


% ROTATIONAL CONVECTION COEFFICIENT 
function h = convectionCoefficientRotation(location, omega_z, box_centroid, air_density, air_viscosity, air_conductivity, air_prandtl, L_characteristic, h_free) 


    % POSITION 
    x = location.x; 

    y = location.y; 


    % RADIAL DISTANCE FROM ROTATION AXIS 
    r = sqrt((x - box_centroid(1)).^2 + (y - box_centroid(2)).^2); 


    % ROTATIONAL VELOCITY 
    velocity = abs(omega_z) .* r; 


    % REYNOLDS NUMBER 
    Re = air_density .* velocity .* L_characteristic ./ air_viscosity; 

    Re = max(Re, 1); 


    % NUSSELT NUMBER 
    Nu = 0.664 .* sqrt(Re) .* air_prandtl.^(1 / 3); 


    % TURBULENT REGION 
    turbulent = Re >= 5e5; 

    Nu(turbulent) = (0.037 .* Re(turbulent).^0.8 - 871) .* air_prandtl.^(1 / 3); 


    % FORCED CONVECTION 
    h_forced = Nu .* air_conductivity ./ L_characteristic; 


    % TOTAL CONVECTION 
    h = h_free + h_forced; 

    h = max(h, 1e-6); 

end 
