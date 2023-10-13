%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef ArmController
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties (Constant)
        DEFAULT_STEPS_PER_METRE = 200;
        DEFAULT_IK_ERROR_MAX = 10^-3;
        DEFAULT_VELOCITY_MAX = 0.1; % m/s
    end

    properties(Access = private)
        robot
        stepsPerMetre
        ikErrorMax
        velocityMax
        currentState
        currentPose
        nextPose
        errorCode
    end

    properties(Access = public)
        previousState
        previousPoses
    end

    properties(Dependent)
        jointPose
        nextState
        EStopFlag
    end

    methods
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%//Constructors///////////////////////////////////////////////////////////%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = ArmController(robot,stepsPerMetre,ikErrorMax,velocityMax,qInit)
            %Instantiate class object
            %Setting member attributes to default values
            self.errorCode = 0;
            self.stepsPerMetre = self.DEFAULT_STEPS_PER_METRE;
            self.ikErrorMax = self.DEFAULT_IK_ERROR_MAX;
            self.currentState = armState.Init;
        
            if nargin > 0
                % Just robot arm passed
                self.robot = robot;
                if nargin > 1
                    % Arm and steps per metre
                    self.stepsPerMetre = stepsPerMetre;
                end
                if nargin > 2
                    % Arm, steps and inv kinematic error max
                    self.ikErrorMax = ikErrorMax;
                end
                if nargin > 3
                    % Arm, steps, inv kinematic error max and velocity max
                    self.velocityMax = velocityMax;
                end
                if nargin > 4
                    % All elements passed
                    self.robot.model.animate(qInit);
                else
                    % No initial joint angles passed
                    self.robot.model.animate(zeros(1,self.robot.model.n));
                end
            else % Nothing passed!
                self.errorCode = 1;
                error('No robot passed to controller');
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%//Getters////////////////////////////////////////////////////////////////%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [jointPose,self] = GetJointPose(self,jointNumber)
            %Get the pose transform for the specified joint
            % (returns empty if outside of joint bounds)
            if 0 > jointNumber || jointNumber > self.robot.model.n
                %OUT OF BOUNDS
                self.errorCode = 2;
                error('Joint out of bounds');
            else %SELECTION WITHIN JOINT NUMBER
                if  floor((self.robot.model.n)/2) > jointNumber
                    %Joint closer to base
                    disp('Closer to base');
                    currentTransform = ...
                    self.robot.model.base.T;

                    for i = 1:jointNumber
                        theta = self.robot.model.links(i).theta;
                        d = self.robot.model.links(i).d;
                        a = self.robot.model.links(i).a;
                        alpha = self.robot.model.links(i).alpha;

                        if isempty(theta)
                            theta = 0;
                        end
                        if isempty(alpha)
                            alpha = 0;
                        end

                        Tr = eye(4) * ...
                            trotz(rad2deg(theta), 'deg') * ...
                            transl(a,0,d) * ...
                            trotx(rad2deg(alpha), 'deg');

                        currentTransform = currentTransform * Tr;
                    end
                else %Joint closer to end effector
                    disp('Closer to end');
                    currentTransform = ...
                    self.robot.model.fkine(self.robot.model.getpos).T;
                    for i = self.robot.model.n:-1:jointNumber
                        theta = self.robot.model.links(i).theta;
                        d = self.robot.model.links(i).d;
                        a = self.robot.model.links(i).a;
                        alpha = self.robot.model.links(i).alpha;

                        if isempty(theta)
                            theta = 0;
                        end
                        if isempty(alpha)
                            alpha = 0;
                        end

                        Tr = eye(4) * ...
                             trotz(rad2deg(theta)) * ...
                             transl(a,0,d) * ...
                             trotx(rad2deg(alpha));

                        currentTransform = currentTransform * inv(Tr);
                    end
                end
                jointPose = currentTransform;
            end
        end
%-------------------------------------------------------------------------%
        function errorCode = get.errorCode(self)
            errorCode = self.errorCode;
        end
%-------------------------------------------------------------------------%
        function previousState = get.previousState(self)
            previousState = self.previousState;
        end
%-------------------------------------------------------------------------%
        function currentState = get.currentState(self)
            currentState = self.currentState;
        end
%-------------------------------------------------------------------------%
        function stepsPerMetre = get.stepsPerMetre(self)
            stepsPerMetre = self.stepsPerMetre;
        end
%-------------------------------------------------------------------------%
        function ikErrorMax = get.ikErrorMax(self)
            ikErrorMax = self.ikErrorMax;
        end
%-------------------------------------------------------------------------%
        function velMax = get.velocityMax(self)
            velMax = self.velocityMax;
        end
%-------------------------------------------------------------------------%
        function previousPoses = get.previousPoses(self)
            previousPoses = self.previousPoses;
        end
%-------------------------------------------------------------------------%
        function currentPose = get.currentPose(self)
            currentPose = self.currentPose;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%//Setters////////////////////////////////////////////////////////////////%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = set.previousState(self,state)
          try isenum(state)
             self.previousState = state;
          catch ME
             error("previousState must be an item from armState")
          end
        end
%-------------------------------------------------------------------------%
        function self = set.currentState(self,state)
          try isenum(state)
             self.currentState = state;
          catch ME
             error("currentState must be an item from armState")
          end
        end
%-------------------------------------------------------------------------%
        function self = set.nextState(self,state)
            if self.currentState ~= state % ensuring state has changed
                try armState.contains(state)
                    self.previousState = self.currentState;
                    self.currentState = state;
                catch ME
                    error("nextState must be an item from armState")
                end
            end
        end
%-------------------------------------------------------------------------%
        function self = set.EStopFlag(self,stopBool)
            if stopBool
                self.currentState = armState.EStop;
                disp('ESTOP TRIGGERED, OPERATIONS CEASED');
            else
                self.currentState = self.previousState;
                disp('ESTOP RELEASED, OPERATIONS RESUMED');
            end
        end
%-------------------------------------------------------------------------%
        function self = set.previousPoses(self,previousPoses)
            self.previousPoses = previousPoses;
        end
%-------------------------------------------------------------------------%
        function self = set.currentPose(self,currentPose)
            self.currentPose = currentPose;
        end
%-------------------------------------------------------------------------%
function self = set.nextPose(self,nextPose)
            % Setting previous+current poses and next pose
            self.previousPoses{end+1} = self.currentPose;
            self.nextPose = nextPose;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%//Functions//////////////////////////////////////////////////////////////%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = EStop(self,stopBool)
            self.EStopFlag = stopBool;
        end
%-------------------------------------------------------------------------%
        function self = moveToNextPoint(self,qPath)
            for stepCurrent = 1:length(qPath)
                self.robot.model.animate(qPath(stepCurrent,:));
                drawnow();
            
                robotPos = self.robot.model.getpos;
                self.currentPose = self.robot.model.fkine(robotPos).T;
                pause(0);
            end
        end
%-------------------------------------------------------------------------%
%
        function qPath = calcPath(self,desiredTr)
            dist = norm(desiredTr-self.currentPose);
            steps = dist * self.stepsPerMetre;

            mask = [1,1,1,1,1,1];

            q0 = self.robot.model.getpos;
            q1 = self.robot.model.ikcon(desiredTr,mask);

            qPath = jtraj(q0,q1,steps);
        end
%-------------------------------------------------------------------------%
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%