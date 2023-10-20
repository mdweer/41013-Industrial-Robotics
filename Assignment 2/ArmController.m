%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef ArmController
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties (Constant)
        DEFAULT_STEPS_PER_METRE = 50;
        DEFAULT_IK_ERROR_MAX = 10^-3;
        DEFAULT_VELOCITY_MAX = 0.1; % m/s
        MEASURE_OF_MANIPULABILITY_MIN = 0.1;
    end

    properties(Access = private)
        robot
        stepsPerMetre
        ikErrorMax
        velocityMax
        manipMin
        previousState
        previousPoses
        currentState
        currentPose
        nextPose
        errorCode
        EStopButton
    end

    properties(Access = public)
        jointCount
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
        function self = ArmController( ...
                        robot,stepsPerMetre,ikErrorMax,velocityMax,qInit)
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
%-------------------------------------------------------------------------%
        function nJoints = get.jointCount(self)
            nJoints = self.robot.model.n;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%//Setters////////////////////////////////////////////////////////////////%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = set.previousState(self,state)
            disp('set.previousState')
            try isenum(state)
                self.previousState = state;
            catch ME
                error("previousState must be an item from armState")
            end
        end
%-------------------------------------------------------------------------%
        function self = set.currentState(self,state)
            disp('set.currentState')
            try isenum(state)
                self.currentState = state;
            catch ME
                error("currentState must be an item from armState")
            end
        end
%-------------------------------------------------------------------------%
        function self = set.nextState(self,state)
            disp('set.nextState')
            if ~(isequal(self.currentState,state)) % ensuring state has changed
                disp('set.nextState state has changed')
                try isenum(state)
                    self.previousState = self.currentState;
                    disp(self.previousState)
                    self.currentState = state;
                    disp(self.currentState)
                catch ME
                    error("nextState must be an item from armState")
                end
            end
        end
%-------------------------------------------------------------------------%
        function self = set.EStopFlag(self,stopBool)
            disp('set.EStopFlag')
            if stopBool
                self.nextState = armState.EStop;
                fprintf('ESTOP TRIGGERED, OPERATIONS CEASED');
                disp('Check currentState: ')
                disp(self.currentState)
            else
                self.nextState = self.previousState;
                fprintf('ESTOP RELEASED, OPERATIONS RESUMED');
                disp('Check currentState: ')
                disp(self.currentState)
            end
        end
%-------------------------------------------------------------------------%
        function self = EStop(self,stopBool)
            disp('EStop')
            self.EStopFlag = stopBool;
        end
%-------------------------------------------------------------------------%
        function self = set.previousPoses(self,previousPose)
            self.previousPoses{end+1} = previousPose;
        end
%-------------------------------------------------------------------------%
        function self = set.currentPose(self,currentPose)
            self.currentPose = currentPose;
        end
%-------------------------------------------------------------------------%
function self = set.nextPose(self,nextPose)
            % Adds current robot arm pose (translation and rotation) to the
            % previousPoses array (doesn't save nextPose here as that
            % wouldn't account for end-effector error)
            self.previousPoses = self.currentPose;
            % Assigns nextPose
            self.nextPose = nextPose;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%//Functions//////////////////////////////////////////////////////////////%
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
        function [goalReached,err] = moveToNextPoint(self,desiredTr,qPath)
            disp('moveToNextPoint')
            dT = GetClockSpeed() / length(qPath);
            disp('Check currentState: ')
            disp(self.currentState)
            if isequal(self.currentState,armState.EStop)
                disp('OPERATIONS CANNOT CONTINUE UNTIL ESTOP RELEASED')
                goalReached = 0;
                err = 0;
            else
                for stepCurrent = 1:length(qPath)
                    self.robot.model.animate(qPath(stepCurrent,:));
                    drawnow();

                    % self.nextPose = qPath(stepCurrent,:);
                
                    robotPos = self.robot.model.getpos;
                    self.currentPose = self.robot.model.fkine(robotPos).T;
                    
                    % Plot End-Effector position during steps
                    plot3(self.currentPose(1,4), ...
                          self.currentPose(2,4), ...
                          self.currentPose(3,4),'.-b');
                    pause(dT);
                end

                % Plot final End-Effector position (compare to desiredTr)
                plot3(self.currentPose(1,4), ...
                      self.currentPose(2,4), ...
                      self.currentPose(3,4),'o:r');
                % Goal flag, final pose has been reached
                goalReached = 1;
                
                err = norm(transl(desiredTr) - transl(self.currentPose));
            end
        end
%-------------------------------------------------------------------------%
%         function vect = genVector(self,desiredTr)
%             eeTr = self.robot.model.fkine(self.robot.model.getpos);
%             displ = norm(transl(desiredTr) - transl(eeTr));
%             v = self.velocityMax;
% 
%             dx = desiredTr(1,4) - eeTr(1,4);
%             dy = desiredTr(2,4) - eeTr(2,4);
%             dz = desiredTr(3,4) - eeTr(3,4);
% 
%             rx = trotx(destiredTr);
%             ry = troty(destiredTr);
%             rz = trotz(destiredTr);
% 
% 
%             maxMove = dx + dy + dz;
% 
%             vect = [(dx/maxMove) * v, ...
%                     (dy/maxMove) * v, ...
%                     (dz/maxMove) * v, ...
%                     rx,ry,rz];
%         end
% %-------------------------------------------------------------------------% 
%         function qPath = genRMRCPath(self,vector)
%             dT = GetClockSpeed() / length(qPath);
% 
%             manip = zeros(1,steps);
%             err = nan(2,steps);
%             for i = 1:length(qPath)
%                 xdot = (x(:,i+1) - x(:,i))/dT;                                      % Calculate velocity at discrete time step
%                 J = p2.jacob0(qMatrix(i,:));                                            % Get the Jacobian at the current state
%                 J = J(1:3,:);                                                           % Take only first 3 rows (3D)
%                 manip(:,i)= sqrt(det(J*J'));                                                % Measure of Manipulability
%                 if manip(:,i) < minManipMeasure
%                     qdot = inv(J'*J + 0.01*eye(2))*J'*xdot;
%                 else
%                     qdot = inv(J) * xdot;                                               % Solve velocitities via RMRC
%                 end
%                 error(:,i) = xdot - J*qdot;
%                 qMatrix(i+1,:) = qMatrix(i,:) + dT * qdot';                         % Update next joint state
%             end
%         end
%-------------------------------------------------------------------------%
        function qPath = genIKPath(self,desiredTr,profile)
            disp('genIKPath')
            %Calculating distance and required steps based on current
            %Clock Rate
            eeTr = self.robot.model.fkine(self.robot.model.getpos);
            displ = norm(transl(desiredTr) - transl(eeTr));
            steps = round(displ * self.stepsPerMetre);

            M = [1,1,1,1,1,1];

            q0 = self.robot.model.getpos;
            q1 = self.robot.model.ikcon(desiredTr,q0);

            % % Optimising q1 ikcon here
            % finalTr = self.robot.model.fkine(q1);
            % finalErr = norm(transl(desiredTr) - transl(finalTr));
            % qOpt(1,:) = q0;
            % for i = 1:steps
            %     if self.ikErrorMax > finalErr
            %         break
            %     else
            %         disp('finalErr:')
            %         disp(finalErr)
            %         q1 = self.robot.model.ikcon(desiredTr,qOpt(i));
            %         disp('q1:')
            %         disp(q1)
            % 
            %         dq(i) = inv(self.robot.model.jacob0(qOpt(i))) * finalErr;
            % 
            %         finalTr = self.robot.model.fkine(q1);
            %         finalErr = norm(transl(desiredTr) - transl(finalTr));
            % 
            %         qOpt(i+1) = qOpt(i) + dq(i);
            %     end
            % end

            switch profile
                case 'Quin'
                    qPath = jtraj(q0,q1,steps);
                case 'Trap'
                    s = lspb(0,1,steps); %interp scalar
                    qPath = nan(steps,self.jointCount);
                    for i = 1:steps
                        qPath(i,:) = (1-s(i))*q0 + s(i)*q1;
                    end
            end

            if self.currentState ~= armState.EStop
                self.currentState = armState.AssignedNextPose;
            end
        end
%-------------------------------------------------------------------------%
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%