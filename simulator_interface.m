 %   simulator_interface.m
%   Interface to simulator in CoppeliaSim simulator
%   2024/02/19
%   % Copyright (C) 2024
%   Author: Luís Louro, llouro@dei.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt

classdef simulator_interface < handle

    properties (Access = private)
        vrep
        clientID
        PositionHandle          % Kuka target position
        PositionNames           % Kuka target positions names
        TargetHandle            % handle for the target object (96 different position in the shelfs)
        TargetNames             % list of target names
        ObjectHandle            % handle for the object (6 differents)
        ObjectNames             % list of object names
        IntermPosHandle         % handle for the intermediate positions of cans
        IntermPosNames
        ConveyorBeltHandle      % handle for the conveyor object
        TopVisionHandle         % handle fot top view of conveyor and rejects table
        ConveyorVisionHandle    % handle for conveyor vision sensor
        ConveyorProxHandle      % handle for conveyor proximity sensor
        ProxShelfHandle         % handle for the proximity sensors (96)
        ProxShelfNames          % list of prox sensors name

    end

    properties
        TARGET_Number           % Number of targets
        OBJECT_Number           % Number of objects
        PROXSHELF_Number        % Number of prox sensors in the shelfs
        POSKUKA_Number
        INTERM_Number

    end

    methods
        % Create a connection handler to a SAMU Simulation scene
        % remote_ip_address The remote IP address
        % remote_port The remote port
        function [obj,error] = simulator_interface(remote_ip_address, remote_port)
            error = 0;
            ip_address = '127.0.0.1';
            port = 19997;
            if nargin >= 1
                % TODO: Add safe guarding for well formated ip
                ip_address = remote_ip_address;
            end
            if nargin >= 2
                % TODO: Check for valid number
                port = remote_port;
            end

            obj.vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            obj.vrep.simxFinish(-1); % just in case, close all opened connections

            obj.clientID = obj.vrep.simxStart(ip_address, port, true, true, 5000, 5);

            if (obj.clientID > -1)
                fprintf('\n');
                disp('INFO: Connected successfully to CoppeliaSim!');
                fprintf('\n');

            else
                clear obj;
                disp('ERROR: Failed connecting to remote API server. Ensure CoppeliaSim is running and a scenario is loaded.');
                error = 1;
                return;
            end

            %% Synchronous mode
            obj.vrep.simxSynchronous(obj.clientID, true); % Enable the synchronous mode
            obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);

            obj.vrep.simxSynchronousTrigger(obj.clientID); % Trigger next simulation step (Blocking function call)

            % The first simulation step is now being executed
            %ensure it is finished so we can access signals
            obj.vrep.simxGetPingTime(obj.clientID);

            % Six different objects
            obj.OBJECT_Number = 6;
            obj.ObjectNames{1} = 'lata_salsichas_1';
            obj.ObjectNames{2} = 'lata_salsichas_2';
            obj.ObjectNames{3} = 'lata_salsichas_3';
            obj.ObjectNames{4} = 'lata_cogumelos_1';
            obj.ObjectNames{5} = 'lata_cogumelos_2';
            obj.ObjectNames{6} = 'lata_cogumelos_3';

            

            %Get Targets Handle
            for a=1:obj.OBJECT_Number
                object_name = ['/',obj.ObjectNames{a}];
                [res, obj.ObjectHandle{a}] = obj.vrep.simxGetObjectHandle(obj.clientID, object_name, obj.vrep.simx_opmode_blocking);
                if (res ~= obj.vrep.simx_return_ok)
                    disp('ERROR: Failed getting object handle');
                    error = 1;
                    return;
                end
                [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.ObjectHandle{a},-1,obj.vrep.simx_opmode_streaming);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed getting target position information');
                    error = 1;
                    return;
                end
            end
            
            obj.POSKUKA_Number = 5;
            obj.PositionNames{1} = 'Landing';
            obj.PositionNames{2} = 'Idle';
            obj.PositionNames{3} = 'Target1';
            obj.PositionNames{4} = 'Target2';
            obj.PositionNames{5} = 'Landing_error';


            %Get Kuka Positions Handle
            for a=1:obj.POSKUKA_Number
                PositionNames = ['/',obj.PositionNames{a}];
                [res, obj.PositionHandle{a}] = obj.vrep.simxGetObjectHandle(obj.clientID, PositionNames, obj.vrep.simx_opmode_blocking);
                if (res ~= obj.vrep.simx_return_ok)
                    disp('ERROR: Failed getting KUKA target handle');
                    error = 1;
                    return;
                end
                [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.PositionHandle{a},-1,obj.vrep.simx_opmode_streaming);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed getting KUKA target position information');
                    error = 1;
                    return;
                end
            end
            

            % 96 different targets
            obj.TARGET_Number = 96;
           
            % Targets for LEFT SHELF

            obj.TargetNames{1} = 'R6_L1';
            obj.TargetNames{2} = 'R6_L2';
            obj.TargetNames{3} = 'R6_L3';
            obj.TargetNames{4} = 'R6_L4';
            obj.TargetNames{5} = 'R6_L5';
            obj.TargetNames{6} = 'R6_L6';
            obj.TargetNames{7} = 'R6_L7';
            obj.TargetNames{8} = 'R6_L8';
            obj.TargetNames{9} = 'R5_L1';
            obj.TargetNames{10} = 'R5_L2';
            obj.TargetNames{11} = 'R5_L3';
            obj.TargetNames{12} = 'R5_L4';
            obj.TargetNames{13} = 'R5_L5';
            obj.TargetNames{14} = 'R5_L6';
            obj.TargetNames{15} = 'R5_L7';
            obj.TargetNames{16} = 'R5_L8';
            obj.TargetNames{17} = 'R4_L1';
            obj.TargetNames{18} = 'R4_L2';
            obj.TargetNames{19} = 'R4_L3';
            obj.TargetNames{20} = 'R4_L4';
            obj.TargetNames{21} = 'R4_L5';
            obj.TargetNames{22} = 'R4_L6';
            obj.TargetNames{23} = 'R4_L7';
            obj.TargetNames{24} = 'R4_L8';
            obj.TargetNames{25} = 'R3_L1';
            obj.TargetNames{26} = 'R3_L2';
            obj.TargetNames{27} = 'R3_L3';
            obj.TargetNames{28} = 'R3_L4';
            obj.TargetNames{29} = 'R3_L5';
            obj.TargetNames{30} = 'R3_L6';
            obj.TargetNames{31} = 'R3_L7';
            obj.TargetNames{32} = 'R3_L8';
            obj.TargetNames{33} = 'R2_L1';
            obj.TargetNames{34} = 'R2_L2';
            obj.TargetNames{35} = 'R2_L3';
            obj.TargetNames{36} = 'R2_L4';
            obj.TargetNames{37} = 'R2_L5';
            obj.TargetNames{38} = 'R2_L6';
            obj.TargetNames{39} = 'R2_L7';
            obj.TargetNames{40} = 'R2_L8';
            obj.TargetNames{41} = 'R1_L1';
            obj.TargetNames{42} = 'R1_L2';
            obj.TargetNames{43} = 'R1_L3';
            obj.TargetNames{44} = 'R1_L4';
            obj.TargetNames{45} = 'R1_L5';
            obj.TargetNames{46} = 'R1_L6';
            obj.TargetNames{47} = 'R1_L7';
            obj.TargetNames{48} = 'R1_L8';

            % Targets for RIGHT SHELF

            obj.TargetNames{49} = 'R1_R1';
            obj.TargetNames{50} = 'R1_R2';
            obj.TargetNames{51} = 'R1_R3';
            obj.TargetNames{52} = 'R1_R4';
            obj.TargetNames{53} = 'R1_R5';
            obj.TargetNames{54} = 'R1_R6';
            obj.TargetNames{55} = 'R1_R7';
            obj.TargetNames{56} = 'R1_R8';
            obj.TargetNames{57} = 'R2_R1';
            obj.TargetNames{58} = 'R2_R2';
            obj.TargetNames{59} = 'R2_R3';
            obj.TargetNames{60} = 'R2_R4';
            obj.TargetNames{61} = 'R2_R5';
            obj.TargetNames{62} = 'R2_R6';
            obj.TargetNames{63} = 'R2_R7';
            obj.TargetNames{64} = 'R2_R8';
            obj.TargetNames{65} = 'R3_R1';
            obj.TargetNames{66} = 'R3_R2';
            obj.TargetNames{67} = 'R3_R3';
            obj.TargetNames{68} = 'R3_R4';
            obj.TargetNames{69} = 'R3_R5';
            obj.TargetNames{70} = 'R3_R6';
            obj.TargetNames{71} = 'R3_R7';
            obj.TargetNames{72} = 'R3_R8';
            obj.TargetNames{73} = 'R4_R1';
            obj.TargetNames{74} = 'R4_R2';
            obj.TargetNames{75} = 'R4_R3';
            obj.TargetNames{76} = 'R4_R4';
            obj.TargetNames{77} = 'R4_R5';
            obj.TargetNames{78} = 'R4_R6';
            obj.TargetNames{79} = 'R4_R7';
            obj.TargetNames{80} = 'R4_R8';
            obj.TargetNames{81} = 'R5_R1';
            obj.TargetNames{82} = 'R5_R2';
            obj.TargetNames{83} = 'R5_R3';
            obj.TargetNames{84} = 'R5_R4';
            obj.TargetNames{85} = 'R5_R5';
            obj.TargetNames{86} = 'R5_R6';
            obj.TargetNames{87} = 'R5_R7';
            obj.TargetNames{88} = 'R5_R8';
            obj.TargetNames{89} = 'R6_R1';
            obj.TargetNames{90} = 'R6_R2';
            obj.TargetNames{91} = 'R6_R3';
            obj.TargetNames{92} = 'R6_R4';
            obj.TargetNames{93} = 'R6_R5';
            obj.TargetNames{94} = 'R6_R6';
            obj.TargetNames{95} = 'R6_R7';
            obj.TargetNames{96} = 'R6_R8';

            %Get Targets Handle
            for a=1:obj.TARGET_Number
                target_name = ['/',obj.TargetNames{a}];
                [res, obj.TargetHandle{a}] = obj.vrep.simxGetObjectHandle(obj.clientID, target_name, obj.vrep.simx_opmode_blocking);
                if (res ~= obj.vrep.simx_return_ok)
                    disp('ERROR: Failed getting shelf handle');
                    error = 1;
                    return;
                end
                [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.TargetHandle{a},-1,obj.vrep.simx_opmode_streaming);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed getting target position information');
                    error = 1;
                    return;
                end
            end
            
             
            % 96 different proximity sensors
            obj.PROXSHELF_Number = 96;
           
            % Sensors for LEFT SHELF

            obj.ProxShelfNames{1} = 'Prox_R1_L1';
            obj.ProxShelfNames{2} = 'Prox_R1_L2';
            obj.ProxShelfNames{3} = 'Prox_R1_L3';
            obj.ProxShelfNames{4} = 'Prox_R1_L4';
            obj.ProxShelfNames{5} = 'Prox_R1_L5';
            obj.ProxShelfNames{6} = 'Prox_R1_L6';
            obj.ProxShelfNames{7} = 'Prox_R1_L7';
            obj.ProxShelfNames{8} = 'Prox_R1_L8';
            obj.ProxShelfNames{9} = 'Prox_R2_L1';
            obj.ProxShelfNames{10} = 'Prox_R2_L2';
            obj.ProxShelfNames{11} = 'Prox_R2_L3';
            obj.ProxShelfNames{12} = 'Prox_R2_L4';
            obj.ProxShelfNames{13} = 'Prox_R2_L5';
            obj.ProxShelfNames{14} = 'Prox_R2_L6';
            obj.ProxShelfNames{15} = 'Prox_R2_L7';
            obj.ProxShelfNames{16} = 'Prox_R2_L8';
            obj.ProxShelfNames{17} = 'Prox_R3_L1';
            obj.ProxShelfNames{18} = 'Prox_R3_L2';
            obj.ProxShelfNames{19} = 'Prox_R3_L3';
            obj.ProxShelfNames{20} = 'Prox_R3_L4';
            obj.ProxShelfNames{21} = 'Prox_R3_L5';
            obj.ProxShelfNames{22} = 'Prox_R3_L6';
            obj.ProxShelfNames{23} = 'Prox_R3_L7';
            obj.ProxShelfNames{24} = 'Prox_R3_L8';
            obj.ProxShelfNames{25} = 'Prox_R4_L1';
            obj.ProxShelfNames{26} = 'Prox_R4_L2';
            obj.ProxShelfNames{27} = 'Prox_R4_L3';
            obj.ProxShelfNames{28} = 'Prox_R4_L4';
            obj.ProxShelfNames{29} = 'Prox_R4_L5';
            obj.ProxShelfNames{30} = 'Prox_R4_L6';
            obj.ProxShelfNames{31} = 'Prox_R4_L7';
            obj.ProxShelfNames{32} = 'Prox_R4_L8';
            obj.ProxShelfNames{33} = 'Prox_R5_L1';
            obj.ProxShelfNames{34} = 'Prox_R5_L2';
            obj.ProxShelfNames{35} = 'Prox_R5_L3';
            obj.ProxShelfNames{36} = 'Prox_R5_L4';
            obj.ProxShelfNames{37} = 'Prox_R5_L5';
            obj.ProxShelfNames{38} = 'Prox_R5_L6';
            obj.ProxShelfNames{39} = 'Prox_R5_L7';
            obj.ProxShelfNames{40} = 'Prox_R5_L8';
            obj.ProxShelfNames{41} = 'Prox_R6_L1';
            obj.ProxShelfNames{42} = 'Prox_R6_L2';
            obj.ProxShelfNames{43} = 'Prox_R6_L3';
            obj.ProxShelfNames{44} = 'Prox_R6_L4';
            obj.ProxShelfNames{45} = 'Prox_R6_L5';
            obj.ProxShelfNames{46} = 'Prox_R6_L6';
            obj.ProxShelfNames{47} = 'Prox_R6_L7';
            obj.ProxShelfNames{48} = 'Prox_R6_L8';

            % Sensors for RIGHT SHELF

            obj.ProxShelfNames{49} = 'Prox_R1_R1';
            obj.ProxShelfNames{50} = 'Prox_R1_R2';
            obj.ProxShelfNames{51} = 'Prox_R1_R3';
            obj.ProxShelfNames{52} = 'Prox_R1_R4';
            obj.ProxShelfNames{53} = 'Prox_R1_R5';
            obj.ProxShelfNames{54} = 'Prox_R1_R6';
            obj.ProxShelfNames{55} = 'Prox_R1_R7';
            obj.ProxShelfNames{56} = 'Prox_R1_R8';
            obj.ProxShelfNames{57} = 'Prox_R2_R1';
            obj.ProxShelfNames{58} = 'Prox_R2_R2';
            obj.ProxShelfNames{59} = 'Prox_R2_R3';
            obj.ProxShelfNames{60} = 'Prox_R2_R4';
            obj.ProxShelfNames{61} = 'Prox_R2_R5';
            obj.ProxShelfNames{62} = 'Prox_R2_R6';
            obj.ProxShelfNames{63} = 'Prox_R2_R7';
            obj.ProxShelfNames{64} = 'Prox_R2_R8';
            obj.ProxShelfNames{65} = 'Prox_R3_R1';
            obj.ProxShelfNames{66} = 'Prox_R3_R2';
            obj.ProxShelfNames{67} = 'Prox_R3_R3';
            obj.ProxShelfNames{68} = 'Prox_R3_R4';
            obj.ProxShelfNames{69} = 'Prox_R3_R5';
            obj.ProxShelfNames{70} = 'Prox_R3_R6';
            obj.ProxShelfNames{71} = 'Prox_R3_R7';
            obj.ProxShelfNames{72} = 'Prox_R3_R8';
            obj.ProxShelfNames{73} = 'Prox_R4_R1';
            obj.ProxShelfNames{74} = 'Prox_R4_R2';
            obj.ProxShelfNames{75} = 'Prox_R4_R3';
            obj.ProxShelfNames{76} = 'Prox_R4_R4';
            obj.ProxShelfNames{77} = 'Prox_R4_R5';
            obj.ProxShelfNames{78} = 'Prox_R4_R6';
            obj.ProxShelfNames{79} = 'Prox_R4_R7';
            obj.ProxShelfNames{80} = 'Prox_R4_R8';
            obj.ProxShelfNames{81} = 'Prox_R5_R1';
            obj.ProxShelfNames{82} = 'Prox_R5_R2';
            obj.ProxShelfNames{83} = 'Prox_R5_R3';
            obj.ProxShelfNames{84} = 'Prox_R5_R4';
            obj.ProxShelfNames{85} = 'Prox_R5_R5';
            obj.ProxShelfNames{86} = 'Prox_R5_R6';
            obj.ProxShelfNames{87} = 'Prox_R5_R7';
            obj.ProxShelfNames{88} = 'Prox_R5_R8';
            obj.ProxShelfNames{89} = 'Prox_R6_R1';
            obj.ProxShelfNames{90} = 'Prox_R6_R2';
            obj.ProxShelfNames{91} = 'Prox_R6_R3';
            obj.ProxShelfNames{92} = 'Prox_R6_R4';
            obj.ProxShelfNames{93} = 'Prox_R6_R5';
            obj.ProxShelfNames{94} = 'Prox_R6_R6';
            obj.ProxShelfNames{95} = 'Prox_R6_R7';
            obj.ProxShelfNames{96} = 'Prox_R6_R8';
            
            %Get sensors Handle
            for a=1:obj.PROXSHELF_Number
                prox_name = ['/',obj.ProxShelfNames{a}];
                [res, obj.ProxShelfHandle{a}] = obj.vrep.simxGetObjectHandle(obj.clientID, prox_name, obj.vrep.simx_opmode_blocking);
                if (res ~= obj.vrep.simx_return_ok)
                    disp('ERROR: Failed getting proximity sensor shelf handle');
                    disp(a);
                    error = 1;
                    return;
                end
                [res,~,~,~,~]=obj.vrep.simxReadProximitySensor(obj.clientID, obj.ProxShelfHandle{a}, obj.vrep.simx_opmode_streaming);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed getting target position information');
                    error = 1;
                    return;
                end
            end
            
             %INTERMEDIATE POSITIONS
            obj.INTERM_Number = 18;
            obj.IntermPosNames{1} = 'KUKA_pos1';
            obj.IntermPosNames{2} = 'KUKA_pos2';
            obj.IntermPosNames{3} = 'KUKA_pos3';
            obj.IntermPosNames{4} = 'KUKA_pos4';
            obj.IntermPosNames{5} = 'KUKA_pos5';
            obj.IntermPosNames{6} = 'KUKA_pos6';
            obj.IntermPosNames{7} = 'KUKA_pos7';
            obj.IntermPosNames{8} = 'KUKA_pos8';
            obj.IntermPosNames{9} = 'KUKA_pos9';

            obj.IntermPosNames{10} = 'KUKA_pos10';
            obj.IntermPosNames{11} = 'KUKA_pos11';
            obj.IntermPosNames{12} = 'KUKA_pos12';
            obj.IntermPosNames{13} = 'KUKA_pos13';
            obj.IntermPosNames{14} = 'KUKA_pos14';
            obj.IntermPosNames{15} = 'KUKA_pos15';
            obj.IntermPosNames{16} = 'KUKA_pos16';
            obj.IntermPosNames{17} = 'KUKA_pos17';
            obj.IntermPosNames{18} = 'KUKA_pos18';
            
            % Get position handle
            for a=1:obj.INTERM_Number
                interm_name = ['/',obj.IntermPosNames{a}];
                [res, obj.IntermPosHandle{a}] = obj.vrep.simxGetObjectHandle(obj.clientID, interm_name, obj.vrep.simx_opmode_blocking);
                if (res ~= obj.vrep.simx_return_ok)
                    disp('ERROR: Failed getting Storage KUKA position handle');
                    disp(a);
                    error = 1;
                    return;
                end
                [res,~]=obj.vrep.simxGetObjectPosition(obj.clientID, obj.IntermPosHandle{a}, -1, obj.vrep.simx_opmode_streaming);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed getting Storage KUKA position information');
                    error = 1;
                    return;
                end
            end

            %Get ConveyorBelt Handle
            [res, obj.ConveyorBeltHandle] = obj.vrep.simxGetObjectHandle(obj.clientID, 'conveyor', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting conveyorbelt handle');
                error = 1;
                return;
            end

            [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocity', 0, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed set conveyorbelt velocity!');
                error=1;
                return;
            end

            %Get conveyors vision handles
            [res,obj.ConveyorVisionHandle]=obj.vrep.simxGetObjectHandle(obj.clientID,'Conveyor_vision',obj.vrep.simx_opmode_oneshot_wait);     %side view orthogonal vision sensor

            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed Conveyor vision sensor handle ');
                error = 1;
                return;
            end
            [res,obj.TopVisionHandle]=obj.vrep.simxGetObjectHandle(obj.clientID,'Top_vision',obj.vrep.simx_opmode_oneshot_wait);     %top view vision sensor

            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed Conveyor vision sensor handle ');
                error = 1;
                return;
            end

            %Get conveyor proximity sensor handle
            [res, obj.ConveyorProxHandle] = obj.vrep.simxGetObjectHandle(obj.clientID,'Conveyor_prox', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting conveyor handle');
                error = 1;
                return;
            end
            [res,~,~,~,~]=obj.vrep.simxReadProximitySensor(obj.clientID, obj.ConveyorProxHandle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting target position information');
                error = 1;
                return;
            end

             %% Setup data streaming
            % simulation time
            [res, ~] = obj.vrep.simxGetFloatSignal(obj.clientID,'SimulationTime', obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed to setup data streaming for simulation time');
                error = 1;
                return;
            end

            [res, ~] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTimeStep', obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed to setup data streaming for simulation time step');
                error = 1;
                return;
            end
        end

        function [vrep, clientID] = get_connection(obj)
            vrep = obj.vrep;
            clientID = obj.clientID;
        end

        % call before setting velocity and getting data
        function ensure_all_data(obj)
            obj.vrep.simxGetPingTime(obj.clientID);
        end

        % call just after all data have been collected.
        function trigger_simulation(obj)
            obj.vrep.simxSynchronousTrigger(obj.clientID);
        end

        %Function that allows you to get the position of the target
        function [error,targetPosition]=get_target_position(obj,iTarget)
            error = 0;
            [res,targetPosition] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.TargetHandle{iTarget},-1,obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting target position information');
                error = 1;
                return;
            end
        end

         %Function that allows you to get the position of the target
         function [error,objectPosition]=get_object_position(obj,iObject)
            error = 0;
            [res,objectPosition] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.ObjectHandle{iObject},-1,obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting object position information');
                error = 1;
                return;
            end
        end
        
        %Function that allows you to get the position of the target
        function [error,targetPosition]=get_KUKAtarget_position(obj,iTarget)
            error = 0;
            [res,tposition] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.PositionHandle{iTarget},-1,obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting target position information');
                error = 1;
                return;
            end
            targetPosition(1)=tposition(1)*100;      %cm
            targetPosition(2)=tposition(2)*100;      %cm
        end

        %Funtion that allows to know the value of the sensor
        function [error,detectionState,detectedPoint]=get_shelf_sensor_value(obj,iSensor)
            error = 0;
            [res,detectionState, detectedPoint,~,~] = obj.vrep.obj.vrep.simxReadProximitySensor(obj.clientID, obj.ProxShelfHandle{iSensor}, obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting object position information');
                error = 1;
                return;
            end
        end
        
        % -> Get intermediate Position for store cans
        function [error,store_pos]=get_intermediate_store_position(obj,iObject)
            error = 0;
            [res,store_pos] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.IntermPosHandle{iObject},-1,obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting object position information');
                error = 1;
                return;
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONVEYOR_FUNCTIONS
        
        %Function that allows you to put the conveyor belt stopped
        function error = stop_conveyorbelt(obj)
            value = 0;
            error = 0;
            [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocity', value, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok ) 
                disp('ERROR: Failed stopping belt!');
                error=1;
                return;
            end
        end

        %Function that allows you to put the conveyor belt in motion
        function error = move_conveyorbelt(obj)
            value = 0.06;
            error = 0;
            [res] = obj.vrep.simxSetFloatSignal(obj.clientID, 'BeltVelocity', value, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: Failed moving belt!');
                error = 1;
                return;
            end
        end

        %Function for vision sensor on the conveyor
        %-> vision Functions
        function [error,image] = innit_top_Image(obj)
            error = 0;
            [~, ~, image]=obj.vrep.simxGetVisionSensorImage2(obj.clientID, obj.TopVisionHandle, 0 , obj.vrep.simx_opmode_streaming);
        end

        function [error, image] = get_top_Image(obj)
            error = 0;
            [~, ~, image] = obj.vrep.simxGetVisionSensorImage2(obj.clientID, obj.TopVisionHandle, 0, obj.vrep.simx_opmode_buffer);
        end
        
        function [error,image] = innit_conv_Image(obj)
            error = 0;
            [~, ~, image]=obj.vrep.simxGetVisionSensorImage2(obj.clientID, obj.ConveyorVisionHandle, 0 , obj.vrep.simx_opmode_streaming);
        end

        function [error, image] = get_conv_Image(obj)
            error = 0;
            [~, ~, image] = obj.vrep.simxGetVisionSensorImage2(obj.clientID, obj.ConveyorVisionHandle, 0, obj.vrep.simx_opmode_buffer);
        end

        %Function for proximity sensor on the conveyor
        function [error,detectionState]=get_conveyor_sensor_value(obj)
            error = 0;
            [res,detectionState,~,~,~] = obj.vrep.simxReadProximitySensor(obj.clientID, obj.ConveyorProxHandle, obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting object position information');
                error = 1;
                return;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function [error,sim_time] = get_simulation_time(obj)
            error = 0;
            [res, sim_time] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTime', obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: The simulation stopped in CoppeliaSim');
                error = 1;
                return;
            end
        end

        function [error,sim_timestep] = get_simulation_timestep(obj)
            error = 0;
            [res, sim_timestep] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTimeStep', obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: Failed getting simulation time step!');
                error = 1;
                return;
            end
        end

        function error = terminate(obj)
            error = 0;
            res = obj.vrep.simxStopSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed stopping simulation!');
                error = 1;
                return;
            end
            obj.vrep.simxGetPingTime(obj.clientID);

            % Now close the connection to CoppeliaSim:
            obj.vrep.simxFinish(obj.clientID);

            obj.vrep.delete(); % call the destructor!
        end
    end

end

