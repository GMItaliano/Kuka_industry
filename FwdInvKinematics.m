%% Brief Explanation

%Forward kinematics -> 

% Inverse kinematics -> 




%% Code 

classdef FwdInvKinematics
    properties
        theta
        fwdPos
        invPos
        Link
    end

    methods
        function initialization(obj)
            % Constructor -> Responsible to enable all

            obj.theta = zeros(7,1);
            obj.fwdPos = zeros(3,1);
            obj.invPos = zeros(3,1);
            obj.Link(1)=0.360;
            obj.Link(2)=0.42;
            obj.Link(3)=0.4;
            obj.Link(4)=0.126;
        end
        
        function transf = denavitHartenberg(obj, link_twist, link_offset, joint_angle, link_size)
        
            transf = [cos(joint_angle), -sin(joint_angle)*cos(link_twist), sin(joint_angle)*sin(link_twist), link_size*cos(joint_angle);
                      sin(joint_angle), cos(joint_angle)*cos(link_twist), -cos(joint_angle)*sin(link_twist), link_size*sin(joint_angle);
                      0               , sin(link_twist)                 , cos(link_twist)                  , link_offset               ;       
                      0               , 0                               , 0                                , 1                          ];
        
        end

        function [error, pos_EE] = forwardKinematics(obj, alphai, di, thi)
            
            % trans_EE = T01*T12*T23*T34*T45*T56*T67
            trans_ee = 
        end

        function inverseKinematics()
        end
        
        function pathPlan()
        end
    end
end
