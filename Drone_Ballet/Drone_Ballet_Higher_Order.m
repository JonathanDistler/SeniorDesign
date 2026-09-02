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
% 25. CREATE STORAGE
% ================================================================

results = ...
    cell(numObjects,1);


models = ...
    cell(numObjects,1);


meshHmaxUsed = ...
    nan(numObjects,1);


meshHminUsed = ...
    nan(numObjects,1);


%% ================================================================
% 26. SOLVE EACH OBJECT
% ================================================================

for i = 1:numObjects


    fprintf('\n');
    fprintf('------------------------------------------------------------\n');


    fprintf( ...
        'OBJECT %d / %d\n', ...
        i, ...
        numObjects);


    fprintf( ...
        'Component: %s %d\n', ...
        assemblyObjects(i).componentType, ...
        assemblyObjects(i).componentNumber);


    fprintf( ...
        'Material:  %s\n', ...
        assemblyObjects(i).material);


    %% ------------------------------------------------------------
    % Material
    % -------------------------------------------------------------

    materialName = ...
        assemblyObjects(i).material;


    if ~isfield(materials,materialName)

        error( ...
            "Material '%s' is not defined.", ...
            materialName);

    end


    mat = ...
        materials.(materialName);


    fprintf( ...
        'rho = %.3f kg/m^3\n', ...
        mat.rho);


    fprintf( ...
        'cp  = %.3f J/(kg K)\n', ...
        mat.cp);


    fprintf( ...
        'k   = %.3f W/(m K)\n', ...
        mat.k);


    %% ------------------------------------------------------------
    % Create thermal model
    % -------------------------------------------------------------

    model = ...
        femodel( ...
        AnalysisType="thermalTransient", ...
        Geometry=assemblyObjects(i).geometry);


    %% ------------------------------------------------------------
    % Material properties
    % -------------------------------------------------------------

    model.MaterialProperties = ...
        materialProperties( ...
        ThermalConductivity=mat.k, ...
        MassDensity=mat.rho, ...
        SpecificHeat=mat.cp);


    %% ------------------------------------------------------------
    % Initial temperature
    % -------------------------------------------------------------

    model.CellIC = ...
        cellIC( ...
        Temperature=T0);


    %% ------------------------------------------------------------
    % Stefan-Boltzmann constant
    % -------------------------------------------------------------

    model.StefanBoltzmann = ...
        sigma;


    %% ------------------------------------------------------------
    % All component faces
    % -------------------------------------------------------------

    nFaces = ...
        assemblyObjects(i).geometry.NumFaces;


    allFaces = ...
        1:nFaces;


    %% ------------------------------------------------------------
    % Laser targeting
    % -------------------------------------------------------------

    receivesLaser = ...
        isTargetComponent( ...
        assemblyObjects(i), ...
        laserTarget);


    if receivesLaser

        laserScale = ...
            1.0;

    else

        laserScale = ...
            LASER_SPILLOVER_FRACTION;

    end


    %% ------------------------------------------------------------
    % Laser heat function
    % -------------------------------------------------------------

    laserFunction = ...
        @(location,state) ...
        laserHeatFlux( ...
        location, ...
        state, ...
        laserOrigin, ...
        laserDirection, ...
        P0, ...
        alpha, ...
        w0, ...
        zR, ...
        laserScale);


    %% ------------------------------------------------------------
    % Surface boundary condition
    % -------------------------------------------------------------

    if USE_CONVECTION && USE_RADIATION


        if laserScale > 0

            surfaceLoad = ...
                faceLoad( ...
                Heat=laserFunction, ...
                ConvectionCoefficient=h_free, ...
                AmbientTemperature=Tinf, ...
                Emissivity=emissivity);

        else

            surfaceLoad = ...
                faceLoad( ...
                ConvectionCoefficient=h_free, ...
                AmbientTemperature=Tinf, ...
                Emissivity=emissivity);

        end


    elseif USE_CONVECTION


        if laserScale > 0

            surfaceLoad = ...
                faceLoad( ...
                Heat=laserFunction, ...
                ConvectionCoefficient=h_free, ...
                AmbientTemperature=Tinf);

        else

            surfaceLoad = ...
                faceLoad( ...
                ConvectionCoefficient=h_free, ...
                AmbientTemperature=Tinf);

        end


    elseif USE_RADIATION


        if laserScale > 0

            surfaceLoad = ...
                faceLoad( ...
                Heat=laserFunction, ...
                Emissivity=emissivity);

        else

            surfaceLoad = ...
                faceLoad( ...
                Emissivity=emissivity);

        end


    else


        if laserScale > 0

            surfaceLoad = ...
                faceLoad( ...
                Heat=laserFunction);

        else

            surfaceLoad = ...
                faceLoad();

        end

    end


    %% ------------------------------------------------------------
    % Apply surface load
    % -------------------------------------------------------------

    model.FaceLoad(allFaces) = ...
        surfaceLoad;


    %% ------------------------------------------------------------
    % Laser status
    % -------------------------------------------------------------

    if receivesLaser

        fprintf( ...
            'LASER: FULL POWER TARGET\n');

    elseif laserScale > 0

        fprintf( ...
            'LASER: %.3f %% SPILLOVER\n', ...
            100*laserScale);

    else

        fprintf( ...
            'LASER: OFF FOR THIS COMPONENT\n');

    end


    %% ============================================================
    % AUTOMATIC MESH RETRY
    % ============================================================

    fprintf('\n');
    fprintf('Generating FEM mesh...\n');


    % -------------------------------------------------------------
    % Mesh candidates
    %
    % Hmin is intentionally smaller than Hmax.
    %
    % This is particularly important for imported CAD geometry
    % containing small curved or thin features.
    % -------------------------------------------------------------

    meshAttempts = ...
        [ ...
        meshSize, ...
        meshSize*0.75, ...
        meshSize*0.50, ...
        meshSize*0.30, ...
        meshSize*0.20, ...
        meshSize*0.10 ...
        ];


    meshSuccess = ...
        false;


    lastMeshError = [];


    %% ------------------------------------------------------------
    % Try each mesh
    % -------------------------------------------------------------

    for meshAttempt = ...
            1:length(meshAttempts)


        currentHmax = ...
            meshAttempts(meshAttempt);


        currentHmin = ...
            currentHmax / 5;


        fprintf( ...
            '  Mesh attempt %d/%d: Hmax = %.5f m, Hmin = %.5f m\n', ...
            meshAttempt, ...
            length(meshAttempts), ...
            currentHmax, ...
            currentHmin);


        try


            model = ...
                generateMesh( ...
                model, ...
                Hmax=currentHmax, ...
                Hmin=currentHmin);


            meshSuccess = ...
                true;


            meshHmaxUsed(i) = ...
                currentHmax;


            meshHminUsed(i) = ...
                currentHmin;


            fprintf( ...
                '  Mesh successful.\n');


            fprintf( ...
                '  FEM nodes:    %d\n', ...
                size( ...
                model.Geometry.Mesh.Nodes, ...
                2));


            fprintf( ...
                '  FEM elements: %d\n', ...
                size( ...
                model.Geometry.Mesh.Elements, ...
                2));


            break;


        catch ME


            lastMeshError = ...
                ME;


            fprintf( ...
                '  Mesh failed.\n');


            fprintf( ...
                '  Reason: %s\n', ...
                ME.message);


            if meshAttempt < ...
                    length(meshAttempts)

                fprintf( ...
                    '  Retrying with a finer mesh...\n');

            end

        end

    end


    %% ------------------------------------------------------------
    % Mesh failure
    % -------------------------------------------------------------

    if ~meshSuccess

        error( ...
            "Meshing failed for object %d." + ...
            newline + ...
            "Component: %s %d" + ...
            newline + ...
            "Material: %s" + ...
            newline + ...
            "The script tried multiple Hmax/Hmin values." + ...
            newline + ...
            "Last MATLAB error:" + ...
            newline + ...
            "%s", ...
            i, ...
            assemblyObjects(i).componentType, ...
            assemblyObjects(i).componentNumber, ...
            assemblyObjects(i).material, ...
            lastMeshError.message);

    end


    %% ============================================================
    % THERMAL SOLVE
    % ============================================================

    fprintf('\n');
    fprintf('Solving thermal transient...\n');


    try

        result = ...
            solve( ...
            model, ...
            tList);


    catch ME

        error( ...
            "Thermal solve failed for object %d." + ...
            newline + ...
            "Component: %s %d" + ...
            newline + ...
            "Material: %s" + ...
            newline + ...
            "MATLAB reported:" + ...
            newline + ...
            "%s", ...
            i, ...
            assemblyObjects(i).componentType, ...
            assemblyObjects(i).componentNumber, ...
            assemblyObjects(i).material, ...
            ME.message);

    end


    fprintf( ...
        'Thermal solve complete.\n');


    %% ------------------------------------------------------------
    % Store
    % -------------------------------------------------------------

    results{i} = ...
        result;


    models{i} = ...
        model;


    %% ------------------------------------------------------------
    % Temperature statistics
    % -------------------------------------------------------------

    finalTemperature = ...
        result.Temperature(:,end);


    Tmin = ...
        min(finalTemperature);


    Tmax = ...
        max(finalTemperature);


    fprintf('\n');


    fprintf( ...
        'Minimum final temperature: %.3f K\n', ...
        Tmin);


    fprintf( ...
        'Maximum final temperature: %.3f K\n', ...
        Tmax);


    fprintf( ...
        'Maximum final temperature: %.3f C\n', ...
        Tmax - 273.15);


end


%% ================================================================
% 27. FINAL TEMPERATURE SUMMARY
% ================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('FINAL TEMPERATURE SUMMARY\n');
fprintf('============================================================\n');


summaryName = ...
    strings(numObjects,1);


summaryType = ...
    strings(numObjects,1);


summaryNumber = ...
    zeros(numObjects,1);


summaryMaterial = ...
    strings(numObjects,1);


summaryMaxK = ...
    zeros(numObjects,1);


summaryMaxC = ...
    zeros(numObjects,1);


summaryMeshHmax = ...
    zeros(numObjects,1);


summaryMeshHmin = ...
    zeros(numObjects,1);


for i = 1:numObjects


    Tfinal = ...
        results{i}.Temperature(:,end);


    summaryName(i) = ...
        assemblyObjects(i).name;


    summaryType(i) = ...
        assemblyObjects(i).componentType;


    summaryNumber(i) = ...
        assemblyObjects(i).componentNumber;


    summaryMaterial(i) = ...
        assemblyObjects(i).material;


    summaryMaxK(i) = ...
        max(Tfinal);


    summaryMaxC(i) = ...
        summaryMaxK(i) - 273.15;


    summaryMeshHmax(i) = ...
        meshHmaxUsed(i);


    summaryMeshHmin(i) = ...
        meshHminUsed(i);


    fprintf( ...
        '%-10s %-7s %d %-18s %10.3f K %10.3f C\n', ...
        assemblyObjects(i).name, ...
        assemblyObjects(i).componentType, ...
        assemblyObjects(i).componentNumber, ...
        assemblyObjects(i).material, ...
        summaryMaxK(i), ...
        summaryMaxC(i));

end


%% ================================================================
% 28. SUMMARY TABLE
% ================================================================

summaryTable = ...
    table( ...
    summaryName, ...
    summaryType, ...
    summaryNumber, ...
    summaryMaterial, ...
    summaryMaxK, ...
    summaryMaxC, ...
    summaryMeshHmax, ...
    summaryMeshHmin, ...
    'VariableNames', ...
    { ...
    'ObjectName', ...
    'ComponentType', ...
    'ComponentNumber', ...
    'Material', ...
    'MaximumTemperature_K', ...
    'MaximumTemperature_C', ...
    'MeshHmax_m', ...
    'MeshHmin_m' ...
    });


%% ================================================================
% 29. ASSEMBLY TEMPERATURE PLOT
% ================================================================

fprintf('\n');
fprintf('CREATING ASSEMBLY TEMPERATURE PLOT\n');


figure( ...
    'Name', ...
    'Phantom Drone Final Temperature');


hold on;


for i = 1:numObjects


    finalTemperature = ...
        results{i}.Temperature(:,end);


    pdeplot3D( ...
        results{i}.Mesh, ...
        ColorMapData=finalTemperature, ...
        FaceAlpha=1);

end


axis equal;


xlabel('X [m]');

ylabel('Y [m]');

zlabel('Z [m]');


title( ...
    'Phantom Drone - Final Temperature');


colorbar;

grid on;

view(3);


assemblyFigure = ...
    fullfile( ...
    outputFolder, ...
    'drone_final_temperature.png');


exportgraphics( ...
    gcf, ...
    assemblyFigure);


%% ================================================================
% 30. LASER GEOMETRY PLOT
% ================================================================

fprintf( ...
    'CREATING LASER GEOMETRY PLOT\n');


figure( ...
    'Name', ...
    'Drone and Laser');


hold on;


%% ---------------------------------------------------------------
% Assembly
% ---------------------------------------------------------------

for i = 1:numObjects


    pdegplot( ...
        assemblyObjects(i).geometry, ...
        FaceAlpha=0.15);

end


%% ---------------------------------------------------------------
% Laser origin
% ---------------------------------------------------------------

plot3( ...
    laserOrigin(1), ...
    laserOrigin(2), ...
    laserOrigin(3), ...
    'o', ...
    'MarkerSize',10, ...
    'LineWidth',2);


%% ---------------------------------------------------------------
% Laser target
% ---------------------------------------------------------------

plot3( ...
    laserAimPoint(1), ...
    laserAimPoint(2), ...
    laserAimPoint(3), ...
    'x', ...
    'MarkerSize',12, ...
    'LineWidth',2);


%% ---------------------------------------------------------------
% Laser beam
% ---------------------------------------------------------------

plot3( ...
    [ ...
    laserOrigin(1), ...
    laserAimPoint(1) ...
    ], ...
    [ ...
    laserOrigin(2), ...
    laserAimPoint(2) ...
    ], ...
    [ ...
    laserOrigin(3), ...
    laserAimPoint(3) ...
    ], ...
    'LineWidth',2);


axis equal;


xlabel('X [m]');

ylabel('Y [m]');

zlabel('Z [m]');


title( ...
    'Drone Assembly and Laser');


legend( ...
    'Assembly', ...
    'Laser Origin', ...
    'Laser Aim Point', ...
    'Laser Beam', ...
    'Location','best');


grid on;

view(3);


laserFigure = ...
    fullfile( ...
    outputFolder, ...
    'laser_geometry.png');


exportgraphics( ...
    gcf, ...
    laserFigure);


%% ================================================================
% 31. SAVE CSV
% ================================================================

summaryFile = ...
    fullfile( ...
    outputFolder, ...
    'thermal_summary.csv');


writetable( ...
    summaryTable, ...
    summaryFile);


%% ================================================================
% 32. SAVE MATLAB DATA
% ================================================================

matFile = ...
    fullfile( ...
    outputFolder, ...
    'thermal_results.mat');


save( ...
    matFile, ...
    'results', ...
    'models', ...
    'assemblyObjects', ...
    'summaryTable', ...
    'laserOrigin', ...
    'laserAimPoint', ...
    'laserDirection', ...
    'laserTarget', ...
    'laser_offset_x', ...
    'laser_offset_y', ...
    'laser_offset_z', ...
    'rotationRate_deg_s');


%% ================================================================
% 33. FINAL OUTPUT
% ================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('SIMULATION COMPLETE\n');
fprintf('============================================================\n');


fprintf('\n');


fprintf( ...
    'Output folder:\n%s\n', ...
    outputFolder);


fprintf('\n');


fprintf( ...
    'Laser target: %s\n', ...
    laserTarget);


fprintf( ...
    'Laser spillover: %.3f %%\n', ...
    100*LASER_SPILLOVER_FRACTION);


fprintf( ...
    'Rotation rate: %.3f deg/s\n', ...
    rotationRate_deg_s);


fprintf('\n');


fprintf( ...
    'Laser origin:\n');


fprintf( ...
    '[%.6f %.6f %.6f] m\n', ...
    laserOrigin);


fprintf('\n');


fprintf( ...
    'Laser aim point:\n');


fprintf( ...
    '[%.6f %.6f %.6f] m\n', ...
    laserAimPoint);


fprintf('\n');


fprintf( ...
    'Results saved to:\n%s\n', ...
    matFile);


fprintf('\n');


fprintf('============================================================\n');


%% =================================================================
% LOCAL FUNCTION:
% DETERMINE LASER TARGET
% =================================================================

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
% LASER HEAT FLUX
% =================================================================

function q = ...
    laserHeatFlux( ...
    location, ...
    state, ...
    laserOrigin, ...
    laserDirection, ...
    P0, ...
    alpha, ...
    w0, ...
    zR, ...
    laserScale)


% ================================================================
% IMPORTANT MATLAB FEM REQUIREMENT:
%
% Boundary-condition functions must return a 1 x N vector.
%
% This function therefore explicitly converts q to a ROW VECTOR
% at the end.
% ================================================================


%% ---------------------------------------------------------------
% Number of evaluation points
% ---------------------------------------------------------------

n = ...
    numel(location.x);


%% ---------------------------------------------------------------
% Coordinates
% ---------------------------------------------------------------

x = ...
    location.x(:);


y = ...
    location.y(:);


zCoord = ...
    location.z(:);


P = ...
    [ ...
    x, ...
    y, ...
    zCoord ...
    ];


%% ---------------------------------------------------------------
% Position relative to laser origin
% ---------------------------------------------------------------

R = ...
    P - laserOrigin;


%% ---------------------------------------------------------------
% Distance along beam
% ---------------------------------------------------------------

zBeam = ...
    R * ...
    laserDirection(:);


%% ---------------------------------------------------------------
% Perpendicular distance
% ---------------------------------------------------------------

perpendicular = ...
    R - ...
    zBeam .* ...
    laserDirection;


r = ...
    sqrt( ...
    sum( ...
    perpendicular.^2, ...
    2));


%% ---------------------------------------------------------------
% Beam radius
% ---------------------------------------------------------------

w = ...
    w0 .* ...
    sqrt( ...
    1 + ...
    (zBeam ./ ...
    max(zR,eps)).^2);


%% ---------------------------------------------------------------
% Gaussian irradiance
% ---------------------------------------------------------------

I = ...
    (2 .* P0) ./ ...
    (pi .* w.^2) .* ...
    exp( ...
    -2 .* r.^2 ./ ...
    max(w.^2,eps));


%% ---------------------------------------------------------------
% Atmospheric attenuation
% ---------------------------------------------------------------

I = ...
    I .* ...
    exp( ...
    -alpha .* ...
    max(zBeam,0));


%% ---------------------------------------------------------------
% Initialize heat flux
% ---------------------------------------------------------------

q = ...
    zeros(n,1);


%% ---------------------------------------------------------------
% Only illuminate points in front of laser
% ---------------------------------------------------------------

valid = ...
    zBeam >= 0;


q(valid) = ...
    laserScale .* ...
    I(valid);


%% ================================================================
% CRITICAL FIX:
%
% Return 1 x N instead of N x 1.
%
% This prevents MATLAB from creating an N x N matrix when the
% laser heat flux is combined with convection and radiation.
% ================================================================

q = ...
    reshape(q,1,[]);


% State is intentionally unused.

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
