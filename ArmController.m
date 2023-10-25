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
        qCurrent
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
            self.velocityMax = self.DEFAULT_VELOCITY_MAX;
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
        function qCurrent = get.qCurrent(self)
            qCurrent = self.robot.model.getpos;
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
%-------------------------------------------------------------------------%
        function nJoints = get.jointCount(self)
            nJoints = self.robot.model.n;
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
            if ~(isequal(self.currentState,state)) % ensuring state has changed
                self.previousState = self.currentState;
                self.currentState = state;
            end
        end
%-------------------------------------------------------------------------%
        function self = set.EStopFlag(self,stopBool)
            if stopBool
                self.nextState = armState.EStop;
                disp('ESTOP TRIGGERED, OPERATIONS CEASED');
            else
                self.nextState = self.previousState;
                disp('ESTOP RELEASED, OPERATIONS RESUMED');
            end
        end
%-------------------------------------------------------------------------%
        function self = EStop(self,stopBool)
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
                    currentTransform = ...
                    self.robot.model.base.T;

                    for i = 1:jointNumber
                        theta = self.robot.model.links(i).offset;
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
                jointPose = currentTransform;
            end
        end
%-------------------------------------------------------------------------%
        function [goalReached,err] = moveToNextPoint(self,desiredTr,qPath)
            delay = GetClockSpeed() / length(qPath);
            
            if isequal(self.currentState,armState.EStop)
                disp('OPERATIONS CANNOT CONTINUE UNTIL ESTOP RELEASED')
                goalReached = 0;
                err = 0;
            else
                for stepCurrent = 1:length(qPath)
                    self.robot.model.animate(qPath(stepCurrent,:));
                    drawnow();
                
                    robotPos = self.robot.model.getpos;
                    self.currentPose = self.robot.model.fkine(robotPos).T;
                    
                    % Plot End-Effector position during steps
                    plot3(self.currentPose(1,4), ...
                          self.currentPose(2,4), ...
                          self.currentPose(3,4),'.-b');
                    pause(delay);
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
        function qPath = genIKPath(self,desiredTr,profile)
            %Calculating distance and required steps based on current
            %Clock Rate
            eeTr = self.robot.model.fkine(self.robot.model.getpos);
            displ = norm(transl(desiredTr) - transl(eeTr));
            steps = round(displ * self.stepsPerMetre);

            M = [1,1,1,1,1,1];

            q0 = self.robot.model.getpos;
            q1 = self.robot.model.ikcon(desiredTr,q0);

            switch profile
                case 'Quin'
                    qPath = jtraj(q0,q1,steps);
                case 'Trap'
                    s = lspb(0,1,steps); %interp scalar
                    qPath = nan(steps,self.jointCount);
                    for i = 1:steps
                        qPath(i,:) = (1-s(i))*q0 + s(i)*q1;
                    end
                case 'RMRC'
                    qPath(1,:) = q0;
                    dT = self.velocityMax * GetClockSpeed();
                    disp(desiredTr)
                    eeTr = self.robot.model.fkine(self.robot.model.getpos).T;
        
                    manip = zeros(1,steps);
                    err = nan(self.jointCount,steps);
                    
                    x = linspace(eeTr(1,4),desiredTr(1,4),steps);
                    y = linspace(eeTr(2,4),desiredTr(2,4),steps);
                    z = linspace(eeTr(3,4),desiredTr(3,4),steps);
                    P = zeros(1,steps);
                    R = zeros(1,steps);
                    Y = zeros(1,steps);
        
                    X = [x',y',z',P',R',Y']';
        
                    for i = 1:steps-1
                        xd = (X(:,i+1) - X(:,i))/dT;
                        J = self.robot.model.jacob0(qPath(i,:));
                        J = J(1:self.jointCount,:);
                        manip(:,i)= sqrt(det(J*J'));
        
                        % Checking if close to singularity
                        if manip(:,i) < self.manipMin
                            qd = inv(J'*J + 0.01*eye(2))*J' * xd;
                        else
                            qd = inv(J) * xd;
                        end
                        
                        errVal = xd - J*qd;
                        err(:,i) = errVal;
                        qPath(i+1,:) = qPath(i,:) + dT * qd';
                    end
            end

            if self.currentState ~= armState.EStop
                self.currentState = armState.AssignedNextPose;
            end
        end
%-------------------------------------------------------------------------%
        function calcWorkEnvelope(self)
            eeArray = cell(1);
            q = zeros(self.jointCount);
            jIncr = 1;
            
            for i = 1:self.jointCount
                jLim = self.robot.model.links(i).qlim;
                jLimMin(i) = jLim(1);
                jLimMax(i) = jLim(2);

                for j = rad2deg(jLimMin(i)):1:rad2deg(jLimMax(i))
                    q(i) = j;
                    if i > 1
                        for k = 1:jIncr
                            qj = j*(k+1);
                            q(i-k) = qj;
                        end
                    end
                    self.robot.model.animate(q);
                    drawnow();

                    eeTr = self.robot.model.fkine(self.robot.model.getpos).T;

                    plot3(eeTr(1,4), ...
                          eeTr(2,4), ...
                          eeTr(3,4),'.-y');

                    eeArray{end+1} = eeTr;

                    jIncr = jIncr + 1;
                    pause(0);
                end
            end
        end
%-------------------------------------------------------------------------%
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%