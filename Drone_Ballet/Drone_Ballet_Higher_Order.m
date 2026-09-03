%% ================================================================
% PHANTOM DRONE - COMPLETE 3MF THERMAL SIMULATION
%
% Input:
%       Drone.3mf
%
% FEATURES
%   - Imports complete Fusion 360 3MF assembly
%   - Keeps each 3MF object independent
%   - Independent thermal solution for each object
%   - Individual material assignment
%   - Component-specific laser targeting
%   - Optional laser spillover
%   - Gaussian laser beam
%   - Convection
%   - Radiation
%   - Automatic FEM mesh retry
%   - Rotation specified in DEGREES/SECOND
%
% MATLAB R2025a
% Partial Differential Equation Toolbox
%
% ================================================================

clear;
clc;
close all;


%% ================================================================
% 1. USER SETTINGS
% ================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('       PHANTOM DRONE 3MF THERMAL SIMULATION\n');
fprintf('============================================================\n');


%% ---------------------------------------------------------------
% 3MF FILE
% ---------------------------------------------------------------

DRONE_FILE = ...
    "C:\Users\jonat\Downloads\Drone.3mf";


%% ---------------------------------------------------------------
% THERMAL SETTINGS
% ---------------------------------------------------------------

T0 = 298.15;                    % Initial temperature [K]

Tinf = 298.15;                  % Ambient temperature [K]

tFinal = 60;                    % Simulation time [s]

numTimeSteps = 61;

tList = ...
    linspace(0,tFinal,numTimeSteps);


%% ---------------------------------------------------------------
% CONVECTION
% ---------------------------------------------------------------

USE_CONVECTION = true;

h_free = 10;                    % W/(m^2 K)


%% ---------------------------------------------------------------
% RADIATION
% ---------------------------------------------------------------

USE_RADIATION = true;

emissivity = 0.85;

sigma = 5.670374419e-8;         % W/(m^2 K^4)


%% ---------------------------------------------------------------
% BASE FEM MESH SIZE
% ---------------------------------------------------------------

% Base maximum element size [m].
%
% The script automatically retries smaller sizes if MATLAB
% cannot mesh an imported component.

meshSize = 0.005;


%% ================================================================
% 2. MATERIAL DATABASE
% ================================================================

materials = struct();


%% ---------------------------------------------------------------
% PLA
% ---------------------------------------------------------------

materials.PLA.rho = 1240;

materials.PLA.cp = 1800;

materials.PLA.k = 0.130;


%% ---------------------------------------------------------------
% Aluminum 6061-T6
% ---------------------------------------------------------------

materials.Aluminum6061.rho = 2700;

materials.Aluminum6061.cp = 896;

materials.Aluminum6061.k = 167;


%% ---------------------------------------------------------------
% Copper
% ---------------------------------------------------------------

materials.Copper.rho = 8960;

materials.Copper.cp = 385;

materials.Copper.k = 401;


%% ---------------------------------------------------------------
% Carbon Fiber
% ---------------------------------------------------------------

materials.CarbonFiber.rho = 1700;

materials.CarbonFiber.cp = 710;

materials.CarbonFiber.k = 10;


%% ---------------------------------------------------------------
% Steel
% ---------------------------------------------------------------

materials.Steel.rho = 7850;

materials.Steel.cp = 490;

materials.Steel.k = 45;


%% ================================================================
% 3. MATERIAL ASSIGNMENTS
% ================================================================

% Main drone body.

bodyMaterial = "PLA";


% Motors.

motorMaterial = "Aluminum6061";


% Propellers.

propMaterial = "CarbonFiber";


% Default material for any unclassified object.

defaultMaterial = "Aluminum6061";


%% ================================================================
% 4. ROTATION
% ================================================================

% IMPORTANT:
%
% This value is DEGREES PER SECOND.
%
% No RPM conversion is used anywhere.

rotationRate_deg_s = 360;


% Rotation axis.

rotationAxis = ...
    [0 0 1];


%% ================================================================
% 5. LASER PARAMETERS
% ================================================================

P0 = 5000;                      % Laser power [W]

alpha = 1.65e-4;               % Absorption coefficient [1/m]

WAVELENGTH = 1064e-9;          % Wavelength [m]

D = 0.10;                      % Initial beam diameter [m]

M2 = 1.5;                      % Beam quality factor


%% ================================================================
% 6. LASER POSITION
% ================================================================

fprintf('\n');
fprintf('LASER POSITION\n');
fprintf('------------------------------------------------------------\n');


laser_offset_x = ...
    input( ...
    'Laser X offset from drone centroid [m] (default 2000): ');


if isempty(laser_offset_x)

    laser_offset_x = 2000;

end


laser_offset_y = ...
    input( ...
    'Laser Y offset from drone centroid [m] (default 0): ');


if isempty(laser_offset_y)

    laser_offset_y = 0;

end


laser_offset_z = ...
    input( ...
    'Laser Z offset from drone centroid [m] (default -80): ');


if isempty(laser_offset_z)

    laser_offset_z = -80;

end


%% ================================================================
% 7. LASER AIM POINT
% ================================================================

fprintf('\n');
fprintf('LASER AIM POINT\n');
fprintf('------------------------------------------------------------\n');


laserAimPoint = ...
    input( ...
    'Laser aim point [x y z] in meters (default assembly centroid): ');


% The actual assembly centroid is not known until the 3MF is
% imported. Empty input is therefore handled later.


if ~isempty(laserAimPoint)

    laserAimPoint = ...
        reshape(laserAimPoint,1,3);

else

    laserAimPoint = [];

end


%% ================================================================
% 8. LASER TARGET
% ================================================================

fprintf('\n');
fprintf('LASER TARGET\n');
fprintf('------------------------------------------------------------\n');


fprintf('Target options:\n\n');

fprintf('    body\n');

fprintf('    motor1\n');
fprintf('    motor2\n');
fprintf('    motor3\n');
fprintf('    motor4\n');

fprintf('    prop1\n');
fprintf('    prop2\n');
fprintf('    prop3\n');
fprintf('    prop4\n');

fprintf('    all\n');

fprintf('    none\n');


laserTarget = ...
    input( ...
    '\nLaser target [default body]: ', ...
    's');


if isempty(laserTarget)

    laserTarget = "body";

end


laserTarget = ...
    lower(string(laserTarget));


%% ================================================================
% 9. LASER SPILLOVER
% ================================================================

% The selected target receives:
%
%       100% laser intensity
%
% Other components receive:
%
%       LASER_SPILLOVER_FRACTION
%
%
% Examples:
%
%       0       = no spillover
%       0.01    = 1%
%       0.05    = 5%

LASER_SPILLOVER_FRACTION = 0.00;


%% ================================================================
% 10. OUTPUT
% ================================================================

outputFolder = ...
    "phantom_drone_3mf_thermal_results";


if ~isfolder(outputFolder)

    mkdir(outputFolder);

end


%% ================================================================
% 11. CHECK INPUT FILE
% ================================================================

if ~isfile(DRONE_FILE)

    error( ...
        "Drone 3MF file does not exist:" + ...
        newline + ...
        "%s", ...
        DRONE_FILE);

end


%% ================================================================
% 12. IMPORT COMPLETE ASSEMBLY
% ================================================================

fprintf('\n');
fprintf('IMPORTING COMPLETE DRONE ASSEMBLY\n');
fprintf('------------------------------------------------------------\n');


[assemblyObjects, ...
    assemblyVertices, ...
    assemblyTriangles] = ...
    readDrone3MF(DRONE_FILE);


fprintf('\n');
fprintf('3MF IMPORT COMPLETE\n');


fprintf( ...
    'Total objects:   %d\n', ...
    length(assemblyObjects));


fprintf( ...
    'Total vertices:  %d\n', ...
    size(assemblyVertices,1));


fprintf( ...
    'Total triangles: %d\n', ...
    size(assemblyTriangles,1));


%% ================================================================
% 13. PRINT OBJECT INFORMATION
% ================================================================

fprintf('\n');
fprintf('ASSEMBLY OBJECTS\n');
fprintf('------------------------------------------------------------\n');


for i = 1:length(assemblyObjects)

    fprintf( ...
        'Object %2d | %-10s | %6d vertices | %6d triangles\n', ...
        i, ...
        assemblyObjects(i).name, ...
        size(assemblyObjects(i).points,1), ...
        size(assemblyObjects(i).triangles,1));

end


%% ================================================================
% 14. IDENTIFY MAIN BODY
% ================================================================

numObjects = ...
    length(assemblyObjects);


triangleCounts = ...
    zeros(numObjects,1);


for i = 1:numObjects

    triangleCounts(i) = ...
        size( ...
        assemblyObjects(i).triangles, ...
        1);

end


[~,bodyIndex] = ...
    max(triangleCounts);


%% ================================================================
% 15. INITIALIZE COMPONENT INFORMATION
% ================================================================

for i = 1:numObjects

    assemblyObjects(i).componentType = ...
        "other";


    assemblyObjects(i).componentNumber = ...
        0;


    assemblyObjects(i).material = ...
        defaultMaterial;

end


%% ---------------------------------------------------------------
% Main body
% ---------------------------------------------------------------

assemblyObjects(bodyIndex).componentType = ...
    "body";


assemblyObjects(bodyIndex).componentNumber = ...
    0;


assemblyObjects(bodyIndex).material = ...
    bodyMaterial;


%% ================================================================
% 16. COMPONENT GROUPS
% ================================================================

% Your current 3MF contains:
%
%   Objects 1-4
%   Objects 5-8
%   Objects 9-12
%   Object 13
%   Objects 14-17
%
% Object 13 is automatically detected as the main body.
%
% The four repeated groups below are explicitly mapped.
%
% If you later determine that a group corresponds to a different
% physical component, ONLY change these arrays.
%


groupA = ...
    [1 2 3 4];


groupB = ...
    [5 6 7 8];


groupC = ...
    [9 10 11 12];


groupD = ...
    [14 15 16 17];


%% ================================================================
% 17. COMPONENT GROUP TYPE
% ================================================================

% ---------------------------------------------------------------
% Group A
% ---------------------------------------------------------------

for i = 1:length(groupA)

    idx = groupA(i);

    assemblyObjects(idx).componentType = ...
        "motor";

    assemblyObjects(idx).componentNumber = ...
        i;

    assemblyObjects(idx).material = ...
        motorMaterial;

end


% ---------------------------------------------------------------
% Group B
% ---------------------------------------------------------------

for i = 1:length(groupB)

    idx = groupB(i);

    assemblyObjects(idx).componentType = ...
        "motor";

    assemblyObjects(idx).componentNumber = ...
        i;

    assemblyObjects(idx).material = ...
        motorMaterial;

end


% ---------------------------------------------------------------
% Group C
% ---------------------------------------------------------------

for i = 1:length(groupC)

    idx = groupC(i);

    assemblyObjects(idx).componentType = ...
        "prop";

    assemblyObjects(idx).componentNumber = ...
        i;

    assemblyObjects(idx).material = ...
        propMaterial;

end


% ---------------------------------------------------------------
% Group D
% ---------------------------------------------------------------

for i = 1:length(groupD)

    idx = groupD(i);

    assemblyObjects(idx).componentType = ...
        "prop";

    assemblyObjects(idx).componentNumber = ...
        i;

    assemblyObjects(idx).material = ...
        propMaterial;

end


%% ================================================================
% 18. PRINT COMPONENT MAP
% ================================================================

fprintf('\n');
fprintf('FINAL COMPONENT MAP\n');
fprintf('------------------------------------------------------------\n');


for i = 1:numObjects

    fprintf( ...
        'Object %2d -> %-6s %d -> %s\n', ...
        i, ...
        assemblyObjects(i).componentType, ...
        assemblyObjects(i).componentNumber, ...
        assemblyObjects(i).material);

end


%% ================================================================
% 19. ASSEMBLY CENTROID
% ================================================================

assemblyCentroid = ...
    mean(assemblyVertices,1);


fprintf('\n');
fprintf('Assembly centroid:\n');


fprintf( ...
    '    [%.6f %.6f %.6f] m\n', ...
    assemblyCentroid(1), ...
    assemblyCentroid(2), ...
    assemblyCentroid(3));


%% ================================================================
% 20. DEFAULT LASER AIM POINT
% ================================================================

if isempty(laserAimPoint)

    laserAimPoint = ...
        assemblyCentroid;

end


%% ================================================================
% 21. LASER ORIGIN
% ================================================================

laserOrigin = ...
    assemblyCentroid + ...
    [ ...
    laser_offset_x, ...
    laser_offset_y, ...
    laser_offset_z ...
    ];


fprintf('\n');
fprintf('Laser origin:\n');


fprintf( ...
    '    [%.6f %.6f %.6f] m\n', ...
    laserOrigin(1), ...
    laserOrigin(2), ...
    laserOrigin(3));


fprintf('\n');
fprintf('Laser aim point:\n');


fprintf( ...
    '    [%.6f %.6f %.6f] m\n', ...
    laserAimPoint(1), ...
    laserAimPoint(2), ...
    laserAimPoint(3));


%% ================================================================
% 22. LASER DIRECTION
% ================================================================

laserDirection = ...
    laserAimPoint - laserOrigin;


laserDistance = ...
    norm(laserDirection);


if laserDistance < eps

    error( ...
        "Laser origin and laser aim point are identical.");

end


laserDirection = ...
    laserDirection ./ ...
    laserDistance;


fprintf('\n');
fprintf('Laser direction:\n');


fprintf( ...
    '    [%.6f %.6f %.6f]\n', ...
    laserDirection(1), ...
    laserDirection(2), ...
    laserDirection(3));


%% ================================================================
% 23. GAUSSIAN BEAM PARAMETERS
% ================================================================

w0 = ...
    D ./ ...
    (pi*M2);


zR = ...
    pi*w0^2 ./ ...
    (M2*WAVELENGTH);


fprintf('\n');
fprintf('Gaussian beam parameters:\n');


fprintf( ...
    '    Beam waist:     %.6e m\n', ...
    w0);


fprintf( ...
    '    Rayleigh range: %.6e m\n', ...
    zR);


%% ================================================================
% 24. IMPORT FEM GEOMETRIES
% ================================================================

fprintf('\n');
fprintf('CREATING FEM GEOMETRIES\n');
fprintf('------------------------------------------------------------\n');


for i = 1:numObjects

    fprintf( ...
        'Creating geometry for object %d...\n', ...
        i);


    TR = ...
        triangulation( ...
        assemblyObjects(i).triangles, ...
        assemblyObjects(i).points);


    try

        assemblyObjects(i).geometry = ...
            fegeometry(TR);


    catch ME

        error( ...
            "Object %d (%s) could not be converted to FEM geometry." + ...
            newline + ...
            "MATLAB reported:" + ...
            newline + ...
            "%s", ...
            i, ...
            assemblyObjects(i).name, ...
            ME.message);

    end

end



%% ================================================================
% 25. TRANSIENT / RPM SWEEP SETTINGS
% ================================================================

% These are taken from the transient settings in
% drone_thermal_combined_fixed.m.
t_final = 60.0;                    % [s]
dt = 0.01;                        % [s]

% Requested frequency sweep: 0 to 60 RPM in 10 RPM increments.
rpmSweep = 0:10:60;
frequencySweep_Hz = rpmSweep ./ 60;
omegaSweep_rad_s = 2*pi*frequencySweep_Hz;
nRPM = numel(rpmSweep);

% Air properties from the transient source script.
air_density = 1.184;              % [kg/m^3]
air_viscosity = 1.849e-5;         % [Pa*s]
air_conductivity = 0.0263;       % [W/(m*K)]
air_prandtl = 0.707;
L_characteristic = max(meshSize,1e-3);

fprintf('\n');
fprintf('============================================================\n');
fprintf('TRANSIENT RPM SWEEP\n');
fprintf('============================================================\n');
fprintf('Time step: %.3f s\n',dt);
fprintf('Final time: %.1f s\n',t_final);
fprintf('RPM cases: ');
fprintf('%g ',rpmSweep);
fprintf('\n');

%% ================================================================
% 26. CREATE STORAGE
% ================================================================

results = cell(numObjects,nRPM);
models = cell(numObjects,1);
meshHmaxUsed = nan(numObjects,1);
meshHminUsed = nan(numObjects,1);

maxTemperature_K = nan(numObjects,nRPM);
finalTemperature_K = nan(numObjects,nRPM);
timeOfMaximum_s = nan(numObjects,nRPM);
finalAverageTemperature_K = nan(numObjects,nRPM);
maxLaserPower_W = nan(numObjects,nRPM);
totalLaserEnergy_J = nan(numObjects,nRPM);
averageConvection_W_m2K = nan(numObjects,nRPM);
terminationReason = strings(numObjects,nRPM);

% Store time histories in cells so the script retains the transient
% information from each RPM case without forcing all cases to have
% identical termination behavior.
timeHistories = cell(numObjects,nRPM);
maxTemperatureHistories_K = cell(numObjects,nRPM);
laserPowerHistories_W = cell(numObjects,nRPM);

%% ================================================================
% 27. MESH EACH COMPONENT ONCE
% ================================================================

for i = 1:numObjects

    fprintf('\n------------------------------------------------------------\n');
    fprintf('OBJECT %d / %d\n',i,numObjects);
    fprintf('Component: %s %d\n', ...
        assemblyObjects(i).componentType,...
        assemblyObjects(i).componentNumber);
    fprintf('Material:  %s\n',assemblyObjects(i).material);

    materialName = assemblyObjects(i).material;
    if ~isfield(materials,materialName)
        error("Material '%s' is not defined.",materialName);
    end
    mat = materials.(materialName);

    fprintf('rho = %.3f kg/m^3\n',mat.rho);
    fprintf('cp  = %.3f J/(kg K)\n',mat.cp);
    fprintf('k   = %.3f W/(m K)\n',mat.k);

    model = femodel(...
        AnalysisType="thermalTransient",...
        Geometry=assemblyObjects(i).geometry);

    model.MaterialProperties = materialProperties(...
        ThermalConductivity=mat.k,...
        MassDensity=mat.rho,...
        SpecificHeat=mat.cp);

    model.CellIC = cellIC(Temperature=T0);
    model.StefanBoltzmann = sigma;

    nFaces = assemblyObjects(i).geometry.NumFaces;
    allFaces = 1:nFaces;

    receivesLaser = isTargetComponent(assemblyObjects(i),laserTarget);

    if receivesLaser
        laserScale = 1.0;
    else
        laserScale = LASER_SPILLOVER_FRACTION;
    end

    if receivesLaser
        fprintf('LASER: FULL POWER TARGET\n');
    elseif laserScale > 0
        fprintf('LASER: %.3f %% SPILLOVER\n',100*laserScale);
    else
        fprintf('LASER: OFF FOR THIS COMPONENT\n');
    end

    %% ------------------------------------------------------------
    % Mesh retry settings from phantom3_assembly_thermal.m
    % -------------------------------------------------------------

    fprintf('\nGenerating FEM mesh...\n');

    meshAttempts = [ ...
        meshSize,...
        meshSize*0.75,...
        meshSize*0.50,...
        meshSize*0.30,...
        meshSize*0.20,...
        meshSize*0.10];

    meshSuccess = false;
    lastMeshError = [];

    for meshAttempt = 1:length(meshAttempts)
        currentHmax = meshAttempts(meshAttempt);
        currentHmin = currentHmax/5;

        fprintf('  Mesh attempt %d/%d: Hmax = %.5f m, Hmin = %.5f m\n',...
            meshAttempt,length(meshAttempts),currentHmax,currentHmin);

        try
            model = generateMesh(...
                model,...
                Hmax=currentHmax,...
                Hmin=currentHmin);

            meshSuccess = true;
            meshHmaxUsed(i) = currentHmax;
            meshHminUsed(i) = currentHmin;

            fprintf('  Mesh successful.\n');
            fprintf('  FEM nodes:    %d\n',size(model.Geometry.Mesh.Nodes,2));
            fprintf('  FEM elements: %d\n',size(model.Geometry.Mesh.Elements,2));
            break;

        catch ME
            lastMeshError = ME;
            fprintf('  Mesh failed.\n');
            fprintf('  Reason: %s\n',ME.message);
            if meshAttempt < length(meshAttempts)
                fprintf('  Retrying with a finer mesh...\n');
            end
        end
    end

    if ~meshSuccess
        error("Meshing failed for object %d.\nComponent: %s %d\nMaterial: %s\nLast MATLAB error:\n%s",...
            i,assemblyObjects(i).componentType,assemblyObjects(i).componentNumber,...
            assemblyObjects(i).material,lastMeshError.message);
    end

    models{i} = model;

    %% ------------------------------------------------------------
    % Mesh geometry needed by the transient source script
    % -------------------------------------------------------------

    meshNodes = model.Geometry.Mesh.Nodes;
    tetElements = model.Geometry.Mesh.Elements;

    if size(tetElements,1) < 4
        error('Unexpected FEM mesh for object %d: tetrahedral elements need four corner nodes.',i);
    end

    % Geometry bounds for rotational laser/convection calculations.
    x_nodes = meshNodes(1,:);
    y_nodes = meshNodes(2,:);
    z_nodes = meshNodes(3,:);

    x_min = min(x_nodes); x_max = max(x_nodes);
    y_min = min(y_nodes); y_max = max(y_nodes);
    z_min = min(z_nodes); z_max = max(z_nodes);

    box_length = x_max - x_min;
    box_width = y_max - y_min;
    box_height = z_max - z_min;

    box_centroid = [ ...
        (x_min+x_max)/2; ...
        (y_min+y_max)/2; ...
        (z_min+z_max)/2];

    % Use the smaller horizontal dimension, as in the transient source.
    trajectory_radius = 0.40*min(box_length,box_width);

    % Use the laser origin defined from the assembly centroid in the
    % Phantom script. The moving target is centered on this component's
    % bounding-box centroid, matching the transient script's rotating
    % target formulation.
    componentLaserOrigin = laserOrigin(:);

    %% ------------------------------------------------------------
    % Surface triangulation and areas
    % -------------------------------------------------------------

    tetCornerElements = tetElements(1:4,:)';
    TRmesh = triangulation(tetCornerElements,meshNodes');
    [boundaryTriangles,~] = freeBoundary(TRmesh);

    p1 = meshNodes(:,boundaryTriangles(:,1));
    p2 = meshNodes(:,boundaryTriangles(:,2));
    p3 = meshNodes(:,boundaryTriangles(:,3));

    triangleVectors1 = p2-p1;
    triangleVectors2 = p3-p1;
    triangleCross = cross(triangleVectors1',triangleVectors2',2);

    boundaryTriangleAreas = 0.5*vecnorm(triangleCross,2,2);
    boundaryTriangleCentroids = (p1+p2+p3)'/3;

    %% ------------------------------------------------------------
    % Tetrahedral volumes
    % -------------------------------------------------------------

    a = meshNodes(:,tetElements(2,:))-meshNodes(:,tetElements(1,:));
    b = meshNodes(:,tetElements(3,:))-meshNodes(:,tetElements(1,:));
    c = meshNodes(:,tetElements(4,:))-meshNodes(:,tetElements(1,:));

    tetVolumes = abs(dot(a,cross(b,c,1),1))/6;

    %% ============================================================
    % 28. RPM LOOP
    % ============================================================

    for freqIndex = 1:nRPM

        rpm = rpmSweep(freqIndex);
        frequency_Hz = frequencySweep_Hz(freqIndex);
        omega_z = omegaSweep_rad_s(freqIndex);

        fprintf('\n------------------------------------------------------------\n');
        fprintf('OBJECT %d | %s | RPM = %.0f | f = %.4f Hz\n',...
            i,assemblyObjects(i).material,rpm,frequency_Hz);
        fprintf('Angular velocity: %.6f rad/s\n',omega_z);
        fprintf('------------------------------------------------------------\n');

        % Initial condition at the beginning of this RPM case.
        model.CellIC = cellIC(Temperature=T0);

        % Scale the laser exactly according to the Phantom component target.
        laserHeatFunction = @(location,state) ...
            laserScale .* laserHeatFluxCircular(...
            location,state,...
            box_centroid,...
            trajectory_radius,...
            omega_z,...
            0,...
            componentLaserOrigin,...
            P0,...
            alpha,...
            WAVELENGTH,...
            D,...
            M2,...
            1.0);

        if USE_CONVECTION
            convectionFunction = @(location,state) ...
                convectionCoefficientRotation(...
                location,omega_z,box_centroid,...
                air_density,air_viscosity,air_conductivity,...
                air_prandtl,L_characteristic,h_free);
        end

        % Apply the transient source script's time-dependent boundary
        % conditions to the entire external boundary.
        if USE_CONVECTION && USE_RADIATION
            model.FaceLoad(allFaces) = faceLoad(...
                Heat=laserHeatFunction,...
                ConvectionCoefficient=convectionFunction,...
                AmbientTemperature=Tinf,...
                Emissivity=emissivity);
        elseif USE_CONVECTION
            model.FaceLoad(allFaces) = faceLoad(...
                Heat=laserHeatFunction,...
                ConvectionCoefficient=convectionFunction,...
                AmbientTemperature=Tinf);
        elseif USE_RADIATION
            model.FaceLoad(allFaces) = faceLoad(...
                Heat=laserHeatFunction,...
                AmbientTemperature=Tinf,...
                Emissivity=emissivity);
        else
            model.FaceLoad(allFaces) = faceLoad(Heat=laserHeatFunction);
        end

        currentTime = 0;
        currentTemperature = T0*ones(size(meshNodes,2),1);

        timeHistory = 0;
        maxTemperatureHistory = max(currentTemperature);
        laserPowerHistory = 0;

        maxT = max(currentTemperature);
        maxTTime = 0;
        terminated = false;

        while currentTime < t_final

            nextTime = min(currentTime+dt,t_final);

            % Calculate absorbed laser power on the actual external
            % triangular surface, following the second source script.
            location = struct(...
                "x",boundaryTriangleCentroids(:,1)',...
                "y",boundaryTriangleCentroids(:,2)',...
                "z",boundaryTriangleCentroids(:,3)');

            state = struct("time",nextTime);

            qTriangle = laserScale .* laserHeatFluxCircular(...
                location,state,...
                box_centroid,...
                trajectory_radius,...
                omega_z,...
                0,...
                componentLaserOrigin,...
                P0,...
                alpha,...
                WAVELENGTH,...
                D,...
                M2,...
                1.0);

            qTriangle = reshape(qTriangle,[],1);
            laserPower = sum(qTriangle.*boundaryTriangleAreas);

            % Prevent numerical integration from exceeding the incident
            % laser power available at the source-to-target range.
            laserRange = norm(box_centroid-componentLaserOrigin);
            availableIncidentPower = P0*exp(-alpha*laserRange);
            laserPower = min(max(laserPower,0),availableIncidentPower*laserScale);

            % Pass the temperature at the end of the previous short step
            % into the next solve interval. Since this object keeps the
            % same mesh throughout the case, interpolation is exact up
            % to the FEM nodal representation.
            if currentTime > 0
                previousNodes = meshNodes;
                previousTemperature = currentTemperature;

                T_initial_function = @(location) ...
                    interpolatePreviousTemperature(...
                    location,previousNodes,previousTemperature);

                model.CellIC = cellIC(Temperature=T_initial_function);
            end

            try
                stepResults = solve(model,[currentTime,nextTime]);
            catch ME
                error("Thermal solve failed for object %d at %.0f RPM and t = %.3f s.\nComponent: %s %d\nMaterial: %s\nMATLAB reported:\n%s",...
                    i,rpm,currentTime,...
                    assemblyObjects(i).componentType,...
                    assemblyObjects(i).componentNumber,...
                    assemblyObjects(i).material,...
                    ME.message);
            end

            currentTemperature = stepResults.Temperature(:,end);
            currentTime = nextTime;

            currentMax = max(currentTemperature);
            if currentMax > maxT
                maxT = currentMax;
                maxTTime = currentTime;
            end

            timeHistory(end+1) = currentTime; %#ok<SAGROW>
            maxTemperatureHistory(end+1) = currentMax; %#ok<SAGROW>
            laserPowerHistory(end+1) = laserPower; %#ok<SAGROW>

            % Keep the terminal output style of the transient source.
            if abs(currentTime-round(currentTime)) < dt/2 || currentTime >= t_final
                fprintf('t = %6.2f s | Tmax = %8.2f C | laser P = %9.3f W\n',...
                    currentTime,currentMax-273.15,laserPower);
            end
        end

        if terminated
            terminationReason(i,freqIndex) = "Temperature threshold";
        else
            terminationReason(i,freqIndex) = "60 second limit";
        end

        % Final statistics.
        maxTemperature_K(i,freqIndex) = maxT;
        finalTemperature_K(i,freqIndex) = currentMax;
        finalAverageTemperature_K(i,freqIndex) = mean(currentTemperature);
        timeOfMaximum_s(i,freqIndex) = maxTTime;
        maxLaserPower_W(i,freqIndex) = max(laserPowerHistory);
        totalLaserEnergy_J(i,freqIndex) = trapz(timeHistory,laserPowerHistory);

        if USE_CONVECTION
            nodeX = meshNodes(1,:);
            nodeY = meshNodes(2,:);
            radialDistance = sqrt(...
                (nodeX-box_centroid(1)).^2 + ...
                (nodeY-box_centroid(2)).^2);
            velocity = abs(omega_z).*radialDistance;
            Re = air_density.*velocity.*L_characteristic./air_viscosity;
            Re = max(Re,1);
            Nu = 0.664.*sqrt(Re).*air_prandtl.^(1/3);
            turbulent = Re >= 5e5;
            Nu(turbulent) = ...
                (0.037.*Re(turbulent).^0.8-871).*air_prandtl.^(1/3);
            h_nodes = h_free + Nu.*air_conductivity./L_characteristic;
            averageConvection_W_m2K(i,freqIndex) = mean(h_nodes);
        else
            averageConvection_W_m2K(i,freqIndex) = 0;
        end

        timeHistories{i,freqIndex} = timeHistory;
        maxTemperatureHistories_K{i,freqIndex} = maxTemperatureHistory;
        laserPowerHistories_W{i,freqIndex} = laserPowerHistory;
        results{i,freqIndex} = stepResults;

    end
end

%% ================================================================
% 29. RPM SUMMARY TABLE
% ================================================================

rows = numObjects*nRPM;
summaryObject = strings(rows,1);
summaryType = strings(rows,1);
summaryNumber = zeros(rows,1);
summaryMaterial = strings(rows,1);
summaryRPM = zeros(rows,1);
summaryFrequency_Hz = zeros(rows,1);
summaryMaxK = zeros(rows,1);
summaryMaxC = zeros(rows,1);
summaryFinalK = zeros(rows,1);
summaryFinalC = zeros(rows,1);
summaryTimeMax_s = zeros(rows,1);
summaryMaxLaserPower_W = zeros(rows,1);
summaryLaserEnergy_J = zeros(rows,1);
summaryAverageH = zeros(rows,1);
summaryMeshHmax = zeros(rows,1);
summaryMeshHmin = zeros(rows,1);
summaryTermination = strings(rows,1);

r = 0;
for i = 1:numObjects
    for j = 1:nRPM
        r = r+1;
        summaryObject(r) = assemblyObjects(i).name;
        summaryType(r) = assemblyObjects(i).componentType;
        summaryNumber(r) = assemblyObjects(i).componentNumber;
        summaryMaterial(r) = assemblyObjects(i).material;
        summaryRPM(r) = rpmSweep(j);
        summaryFrequency_Hz(r) = frequencySweep_Hz(j);
        summaryMaxK(r) = maxTemperature_K(i,j);
        summaryMaxC(r) = maxTemperature_K(i,j)-273.15;
        summaryFinalK(r) = finalTemperature_K(i,j);
        summaryFinalC(r) = finalTemperature_K(i,j)-273.15;
        summaryTimeMax_s(r) = timeOfMaximum_s(i,j);
        summaryMaxLaserPower_W(r) = maxLaserPower_W(i,j);
        summaryLaserEnergy_J(r) = totalLaserEnergy_J(i,j);
        summaryAverageH(r) = averageConvection_W_m2K(i,j);
        summaryMeshHmax(r) = meshHmaxUsed(i);
        summaryMeshHmin(r) = meshHminUsed(i);
        summaryTermination(r) = terminationReason(i,j);
    end
end

summaryTable = table(...
    summaryObject,summaryType,summaryNumber,summaryMaterial,...
    summaryRPM,summaryFrequency_Hz,...
    summaryMaxK,summaryMaxC,summaryFinalK,summaryFinalC,...
    summaryTimeMax_s,summaryMaxLaserPower_W,summaryLaserEnergy_J,...
    summaryAverageH,summaryMeshHmax,summaryMeshHmin,summaryTermination,...
    'VariableNames',{...
    'ObjectName','ComponentType','ComponentNumber','Material',...
    'RPM','Frequency_Hz',...
    'MaximumTemperature_K','MaximumTemperature_C',...
    'FinalTemperature_K','FinalTemperature_C',...
    'TimeOfMaximum_s','MaximumLaserPower_W','TotalLaserEnergy_J',...
    'AverageConvection_W_m2K','MeshHmax_m','MeshHmin_m','Termination'});

%% ================================================================
% 30. PRINT FINAL SUMMARY
% ================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('FINAL RPM SWEEP SUMMARY\n');
fprintf('============================================================\n');

for j = 1:nRPM
    fprintf('\nRPM = %.0f\n',rpmSweep(j));
    for i = 1:numObjects
        fprintf('  Object %2d | %-6s %d | %-18s | Tmax = %10.3f C | Final = %10.3f C\n',...
            i,assemblyObjects(i).componentType,assemblyObjects(i).componentNumber,...
            assemblyObjects(i).material,...
            maxTemperature_K(i,j)-273.15,...
            finalTemperature_K(i,j)-273.15);
    end
end

%% ================================================================
% 31. PLOT MAXIMUM TEMPERATURE VS RPM
% ================================================================

figure('Name','Maximum Temperature vs RPM');
hold on;

for i = 1:numObjects
    plot(rpmSweep,maxTemperature_K(i,:)-273.15,'-o','LineWidth',1.2,...
        'DisplayName',sprintf('Object %d - %s %d',...
        i,assemblyObjects(i).componentType,assemblyObjects(i).componentNumber));
end

grid on;
xlabel('Rotation speed [RPM]');
ylabel('Maximum temperature [C]');
title('Maximum Temperature vs RPM');
legend('Location','eastoutside');

rpmFigure = fullfile(outputFolder,'maximum_temperature_vs_RPM.png');
exportgraphics(gcf,rpmFigure);

%% ================================================================
% 32. PLOT MAXIMUM TEMPERATURE BY MATERIAL VS RPM
% ================================================================

materialNames = unique(string({assemblyObjects.material}));

figure('Name','Maximum Temperature by Material vs RPM');
hold on;

for m = 1:numel(materialNames)
    materialMask = string({assemblyObjects.material}) == materialNames(m);
    materialMax = max(maxTemperature_K(materialMask,:),[],1);
    plot(rpmSweep,materialMax-273.15,'-o','LineWidth',1.5,...
        'DisplayName',materialNames(m));
end

grid on;
xlabel('Rotation speed [RPM]');
ylabel('Maximum temperature [C]');
title('Maximum Temperature by Material vs RPM');
legend('Location','best');

materialFigure = fullfile(outputFolder,'maximum_temperature_by_material_vs_RPM.png');
exportgraphics(gcf,materialFigure);

%% ================================================================
% 33. SAVE CSV
% ================================================================

summaryFile = fullfile(outputFolder,'thermal_RPM_sweep_summary.csv');
writetable(summaryTable,summaryFile);

%% ================================================================
% 34. SAVE MATLAB DATA
% ================================================================

matFile = fullfile(outputFolder,'thermal_RPM_sweep_results.mat');

save(matFile,...
    'results','models','assemblyObjects','summaryTable',...
    'rpmSweep','frequencySweep_Hz','omegaSweep_rad_s',...
    'maxTemperature_K','finalTemperature_K',...
    'finalAverageTemperature_K','timeOfMaximum_s',...
    'maxLaserPower_W','totalLaserEnergy_J',...
    'averageConvection_W_m2K','terminationReason',...
    'timeHistories','maxTemperatureHistories_K','laserPowerHistories_W',...
    'meshHmaxUsed','meshHminUsed',...
    'laserOrigin','laserAimPoint','laserDirection','laserTarget',...
    'laser_offset_x','laser_offset_y','laser_offset_z',...
    't_final','dt');

%% ================================================================
% 35. FINAL OUTPUT
% ================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('SIMULATION COMPLETE\n');
fprintf('============================================================\n');
fprintf('RPM sweep: 0 to 60 RPM in 10 RPM increments\n');
fprintf('Time step: %.3f s\n',dt);
fprintf('Final time: %.1f s\n',t_final);
fprintf('Output folder:\n%s\n',outputFolder);
fprintf('CSV:\n%s\n',summaryFile);
fprintf('MAT:\n%s\n',matFile);
fprintf('RPM plot:\n%s\n',rpmFigure);
fprintf('Material plot:\n%s\n',materialFigure);
fprintf('============================================================\n');



function tf = ...
    isTargetComponent(component,target)


target = ...
    lower(string(target));


componentType = ...
    lower(string(component.componentType));


componentNumber = ...
    component.componentNumber;


tf = false;


%% ---------------------------------------------------------------
% No laser
% ---------------------------------------------------------------

if target == "none"

    return;

end


%% ---------------------------------------------------------------
% Whole assembly
% ---------------------------------------------------------------

if target == "all"

    tf = true;

    return;

end


%% ---------------------------------------------------------------
% Body
% ---------------------------------------------------------------

if target == "body"

    tf = ...
        componentType == "body";

    return;

end


%% ---------------------------------------------------------------
% Motor
% ---------------------------------------------------------------

if startsWith(target,"motor")


    numberString = ...
        extractAfter(target,"motor");


    targetNumber = ...
        str2double(numberString);


    if ~isnan(targetNumber)

        tf = ...
            componentType == "motor" && ...
            componentNumber == targetNumber;

    end


    return;

end


%% ---------------------------------------------------------------
% Propeller
% ---------------------------------------------------------------

if startsWith(target,"prop")


    numberString = ...
        extractAfter(target,"prop");


    targetNumber = ...
        str2double(numberString);


    if ~isnan(targetNumber)

        tf = ...
            componentType == "prop" && ...
            componentNumber == targetNumber;

    end


    return;

end


end


%% =================================================================
% LOCAL FUNCTION:
% READ 3MF ASSEMBLY
% =================================================================

function ...
    [objects,allPoints,allTriangles] = ...
    readDrone3MF(filename)


tempFolder = ...
    tempname;


mkdir(tempFolder);


cleanupObj = ...
    onCleanup( ...
    @()cleanup3MF(tempFolder));


%% ---------------------------------------------------------------
% Unzip 3MF
% ---------------------------------------------------------------

try

    unzip( ...
        filename, ...
        tempFolder);

catch ME

    error( ...
        "Could not unzip the 3MF file." + ...
        newline + ...
        "%s", ...
        ME.message);

end


%% ---------------------------------------------------------------
% Locate model XML
% ---------------------------------------------------------------

modelFile = ...
    fullfile( ...
    tempFolder, ...
    '3D', ...
    '3dmodel.model');


if ~isfile(modelFile)

    error( ...
        "3MF model file was not found:" + ...
        newline + ...
        "%s", ...
        modelFile);

end


%% ---------------------------------------------------------------
% Read XML
% ---------------------------------------------------------------

try

    xmlDoc = ...
        xmlread(modelFile);

catch ME

    error( ...
        "Could not parse 3MF XML." + ...
        newline + ...
        "%s", ...
        ME.message);

end


root = ...
    xmlDoc.getDocumentElement();


%% ---------------------------------------------------------------
% Units
% ---------------------------------------------------------------

unitName = ...
    lower(string( ...
    char(root.getAttribute('unit'))));


if strlength(unitName) == 0

    unitName = ...
        "millimeter";

end


switch unitName


    case "millimeter"

        unitScale = ...
            1e-3;


    case "centimeter"

        unitScale = ...
            1e-2;


    case "meter"

        unitScale = ...
            1;


    case "inch"

        unitScale = ...
            0.0254;


    case "micron"

        unitScale = ...
            1e-6;


    otherwise

        warning( ...
            "Unknown 3MF unit '%s'. Assuming millimeters.", ...
            unitName);


        unitScale = ...
            1e-3;

end


fprintf( ...
    '3MF units: %s\n', ...
    unitName);


%% ---------------------------------------------------------------
% Objects
% ---------------------------------------------------------------

objectNodes = ...
    root.getElementsByTagName('object');


numObjects = ...
    objectNodes.getLength;


if numObjects == 0

    error( ...
        "No mesh objects were found in the 3MF file.");

end


objects = ...
    struct( ...
    'name',{}, ...
    'points',{}, ...
    'triangles',{}, ...
    'geometry',{}, ...
    'material',{}, ...
    'componentType',{}, ...
    'componentNumber',{});


allPoints = ...
    zeros(0,3);


allTriangles = ...
    zeros(0,3);


%% ---------------------------------------------------------------
% Read every mesh object
% ---------------------------------------------------------------

for objectIndex = ...
        0:numObjects-1


    objectNode = ...
        objectNodes.item(objectIndex);


    objectName = ...
        string( ...
        char(objectNode.getAttribute('name')));


    if strlength(objectName) == 0

        objectName = ...
            "Object" + ...
            string(objectIndex+1);

    end


    %% ------------------------------------------------------------
    % Mesh node
    % -------------------------------------------------------------

    meshNodes = ...
        objectNode.getElementsByTagName('mesh');


    if meshNodes.getLength == 0

        continue;

    end


    meshNode = ...
        meshNodes.item(0);


    %% ------------------------------------------------------------
    % Vertices
    % -------------------------------------------------------------

    vertexNodes = ...
        meshNode.getElementsByTagName('vertex');


    numVertices = ...
        vertexNodes.getLength;


    points = ...
        zeros(numVertices,3);


    for v = ...
            0:numVertices-1


        vertex = ...
            vertexNodes.item(v);


        x = ...
            str2double( ...
            char( ...
            vertex.getAttribute('x')));


        y = ...
            str2double( ...
            char( ...
            vertex.getAttribute('y')));


        z = ...
            str2double( ...
            char( ...
            vertex.getAttribute('z')));


        points(v+1,:) = ...
            [x y z] .* ...
            unitScale;

    end


    %% ------------------------------------------------------------
    % Triangles
    % -------------------------------------------------------------

    triangleNodes = ...
        meshNode.getElementsByTagName('triangle');


    numTriangles = ...
        triangleNodes.getLength;


    triangles = ...
        zeros(numTriangles,3);


    for t = ...
            0:numTriangles-1


        triangleNode = ...
            triangleNodes.item(t);


        v1 = ...
            str2double( ...
            char( ...
            triangleNode.getAttribute('v1')));


        v2 = ...
            str2double( ...
            char( ...
            triangleNode.getAttribute('v2')));


        v3 = ...
            str2double( ...
            char( ...
            triangleNode.getAttribute('v3')));


        triangles(t+1,:) = ...
            [v1 v2 v3] + 1;

    end


    %% ------------------------------------------------------------
    % Save object
    % -------------------------------------------------------------

    newObject.name = ...
        objectName;


    newObject.points = ...
        points;


    newObject.triangles = ...
        triangles;


    newObject.geometry = ...
        [];


    newObject.material = ...
        "";


    newObject.componentType = ...
        "other";


    newObject.componentNumber = ...
        0;


    objects(end+1) = ...
        newObject;


    %% ------------------------------------------------------------
    % Global mesh
    % -------------------------------------------------------------

    vertexOffset = ...
        size(allPoints,1);


    allPoints = ...
        [ ...
        allPoints; ...
        points ...
        ];


    allTriangles = ...
        [ ...
        allTriangles; ...
        triangles + vertexOffset ...
        ];

end


%% ---------------------------------------------------------------
% Remove unused vertices
% ---------------------------------------------------------------

if ~isempty(allTriangles)


    usedVertices = ...
        unique( ...
        allTriangles(:));


    vertexMap = ...
        zeros( ...
        size(allPoints,1), ...
        1);


    vertexMap(usedVertices) = ...
        1:length(usedVertices);


    allPoints = ...
        allPoints(usedVertices,:);


    allTriangles = ...
        vertexMap(allTriangles);


    %% ------------------------------------------------------------
    % Remove duplicate triangles
    % -------------------------------------------------------------

    sortedTriangles = ...
        sort( ...
        allTriangles, ...
        2);


    [~,uniqueIndices] = ...
        unique( ...
        sortedTriangles, ...
        'rows', ...
        'stable');


    allTriangles = ...
        allTriangles(uniqueIndices,:);

end


end


%% =================================================================
% LOCAL FUNCTION:
% ROTATING CIRCULAR LASER HEAT FLUX
% =================================================================

function q = laserHeatFluxCircular(...
    location, ...
    state, ...
    box_centroid, ...
    trajectory_radius, ...
    omega_z, ...
    theta_0, ...
    laser_origin_world, ...
    P0, ...
    ALPHA_ATM, ...
    WAVELENGTH, ...
    D, ...
    M2, ...
    absorptivity)

    % Get evaluation time safely.
    if isempty(state) || ...
            ~isfield(state,"time") || ...
            isempty(state.time)
        t = 0;
    else
        t = state.time;
    end

    % During FEM setup MATLAB may request the boundary condition with
    % an undefined time. Return NaN in that case, matching the behavior
    % expected by a time-dependent thermal boundary condition.
    if any(isnan(t(:)))
        q = NaN(size(location.x));
        return;
    end

    % Rotating target point.
    theta = theta_0 + omega_z .* t;

    target = [ ...
        box_centroid(1) + trajectory_radius .* cos(theta); ...
        box_centroid(2) + trajectory_radius .* sin(theta); ...
        box_centroid(3)];

    % Laser direction from source to moving target.
    laser_direction = target - laser_origin_world;
    range = norm(laser_direction);

    if range < 1e-12
        q = zeros(size(location.x));
        return;
    end

    laser_direction = laser_direction ./ range;

    % Gaussian beam parameters, using the same definitions as the
    % transient source script.
    w0 = D ./ (pi .* M2);
    zR = pi .* w0.^2 ./ (M2 .* WAVELENGTH);
    w = w0 .* sqrt(1 + (range ./ max(zR,eps)).^2);

    % Atmospheric attenuation.
    I0 = P0 .* exp(-ALPHA_ATM .* range);

    % Evaluation locations.
    x = location.x(:);
    y = location.y(:);
    z = location.z(:);
    xyz = [x,y,z];

    % Coordinates relative to the instantaneous laser target.
    relative = xyz - target.';

    % Construct a transverse basis perpendicular to the laser direction.
    referenceVector = [0;0;1];
    if abs(dot(referenceVector,laser_direction)) > 0.95
        referenceVector = [0;1;0];
    end

    e1 = cross(laser_direction,referenceVector);
    e1 = e1 ./ max(norm(e1),1e-12);

    e2 = cross(laser_direction,e1);
    e2 = e2 ./ max(norm(e2),1e-12);

    X = relative * e1;
    Y = relative * e2;
    r2 = X.^2 + Y.^2;

    % Gaussian intensity.
    I = (2 .* I0 ./ (pi .* w.^2)) .* exp(-2 .* r2 ./ w.^2);

    % Absorbed surface heat flux.
    q = absorptivity .* I;
    q = reshape(q,size(location.x));
end


%% =================================================================
% LOCAL FUNCTION:
% ROTATIONAL CONVECTION COEFFICIENT
% =================================================================

function h = convectionCoefficientRotation(...
    location, ...
    omega_z, ...
    box_centroid, ...
    air_density, ...
    air_viscosity, ...
    air_conductivity, ...
    air_prandtl, ...
    L_characteristic, ...
    h_free)

    x = location.x;
    y = location.y;

    r = sqrt((x - box_centroid(1)).^2 + ...
             (y - box_centroid(2)).^2);

    velocity = abs(omega_z) .* r;

    Re = air_density .* velocity .* L_characteristic ./ air_viscosity;
    Re = max(Re,1);

    Nu = 0.664 .* sqrt(Re) .* air_prandtl.^(1/3);

    turbulent = Re >= 5e5;
    Nu(turbulent) = ...
        (0.037 .* Re(turbulent).^0.8 - 871) .* air_prandtl.^(1/3);

    h_forced = Nu .* air_conductivity ./ L_characteristic;

    h = h_free + h_forced;
    h = max(h,1e-6);
end


%% =================================================================
% LOCAL FUNCTION:
% INTERPOLATE PREVIOUS TEMPERATURE
% =================================================================

function T_initial = interpolatePreviousTemperature(...
    location, ...
    previousNodes, ...
    previousTemperature)

    previousNodes = previousNodes';

    xq = location.x(:);
    yq = location.y(:);
    zq = location.z(:);

    F = scatteredInterpolant(...
        previousNodes(:,1), ...
        previousNodes(:,2), ...
        previousNodes(:,3), ...
        previousTemperature(:), ...
        "linear", ...
        "nearest");

    T_initial = F(xq,yq,zq);
    T_initial = reshape(T_initial,size(location.x));
end


%% =================================================================
% LOCAL FUNCTION:
% CLEANUP TEMPORARY 3MF DIRECTORY
% =================================================================

function cleanup3MF(folder)


if isfolder(folder)


    try

        rmdir( ...
            folder, ...
            's');

    catch

    end

end


end
