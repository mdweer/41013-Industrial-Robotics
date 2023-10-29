%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef ArmController
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties (Constant)
        DEFAULT_STEPS_PER_METRE = 50;
        DEFAULT_IK_ERROR_MAX = 0.001;
        DEFAULT_VELOCITY_MAX = 0.1; % m/s
        MEASURE_OF_MANIPULABILITY_MIN = 0.1;
    end

    properties(Access = private)
        robot
        gripper
        gripTarget_h
        gripTargetVerts
        gripTargetTr
        stepsPerMetre
        ikErrorMax
        velocityMax
        manipMin
        previousState
        previousTr
        currentState
        currentTr
        nextTr
        errorCode
        EStopButton
    end

    properties(Access = public)
        jointCount
        qCurrent
        gripperCloseFlag
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
            self.gripperCloseFlag = false;
            % ArmState is initialised until constructor is complete
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
                    % self.robot.model.animate(zeros(1,self.robot.model.n));
                end
            else % Nothing passed!
                self.errorCode = 1;
                error('No robot passed to controller');
            end

            % Constructor is complete, arm is waiting to be assigned a pose
            self.nextState = armState.WaitingForNextTr;
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
        function previousTr = get.previousTr(self)
            previousTr = self.previousTr;
        end
%-------------------------------------------------------------------------%
        function currentTr = get.currentTr(self)
            currentTr = self.currentTr;
        end
%-------------------------------------------------------------------------%
        function nJoints = get.jointCount(self)
            nJoints = self.robot.model.n;
        end
%-------------------------------------------------------------------------%
        function gripObj_h = get.gripTarget_h(self)
            gripObj_h = self.gripTarget_h;
        end
%-------------------------------------------------------------------------%
        function gripObjVerts = get.gripTargetVerts(self)
            gripObjVerts = self.gripTargetVerts;
        end
%-------------------------------------------------------------------------%
        function gripObjTr = get.gripTargetTr(self)
            gripObjTr = self.gripTargetTr;
        end
%-------------------------------------------------------------------------%
        function gripperClose = get.gripperCloseFlag(self)
            gripperClose = self.gripperCloseFlag;
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
            % self.EStopFlag = stopBool;
            if stopBool
                self.nextState = armState.EStop;
                disp('ESTOP TRIGGERED, OPERATIONS CEASED');
            else
                self.nextState = self.previousState;
                disp('ESTOP RELEASED, OPERATIONS RESUMED');
            end
        end
%-------------------------------------------------------------------------%
        function self = set.previousTr(self,previousTr)
            self.previousTr{end+1} = previousTr;
        end
%-------------------------------------------------------------------------%
        function self = set.currentTr(self,currentTr)
            self.currentTr = currentTr;
        end
%-------------------------------------------------------------------------%
        function self = set.nextTr(self,nextTr)
            % Adds current robot arm pose (translation and rotation) to the
            % previousTr array (doesn't save nextTr here as that
            % wouldn't account for end-effector error)
            self.previousTr = self.currentTr;
            % Assigns nextTr
            self.nextTr = nextTr;
        end
%-------------------------------------------------------------------------%
        function self = set.gripTarget_h(self,gripObj_h)
            self.gripTarget_h = gripObj_h;
        end
%-------------------------------------------------------------------------%
        function self = set.gripTargetVerts(self,gripObjVerts)
            self.gripTargetVerts = gripObjVerts;
        end
%-------------------------------------------------------------------------%
        function self = set.gripTargetTr(self,gripObjTr)
            self.gripTargetTr = gripObjTr;
        end
%-------------------------------------------------------------------------%
        function self = set.gripperCloseFlag(self,gripperClose)
            self.gripperCloseFlag = gripperClose;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%//Functions//////////////////////////////////////////////////////////////%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = SetGrabTarget(self,gripObjTr,gripObj_h)
            disp('SetGrabTarget - Inputs:')
            disp('Transform:')
            disp(gripObjTr)
            disp('handle:')
            disp(gripObj_h)

            self.gripTarget_h = gripObj_h;
            self.gripTargetVerts = [get(self.gripTarget_h,'Vertices'),...
                ones(size(get(self.gripTarget_h,'Vertices'),1),1)];
            self.gripTargetTr = gripObjTr;
            
            disp('SetGrabTarget - Outputs:')
            disp('Transform:')
            disp(self.gripTargetTr)
            disp('handle:')
            disp(self.gripTarget_h)
            disp('vertices:')
            disp(self.gripTargetVerts)
        end
%-------------------------------------------------------------------------%
        function self = grabTarget(self,gripperClose)
            if gripperClose
                gripObjTr = self.currentTr * inv(self.gripTargetTr);
                
                nextVertices = (self.gripTargetVerts(:,1:3) * ...
                                gripObjTr(1:3,1:3)') + gripObjTr(1:3,4)';
        
                set(brickCurrent_h,'Vertices',nextVertices)
                drawnow();
            else
            end
        end
%-------------------------------------------------------------------------%
        function SetQ(self,q)
            self.robot.model.animate(q);
        end
%-------------------------------------------------------------------------%
        function [jointPose,self] = GetJointPose(self,jointNumber)
            %Get the pose transform for the specified joint
            % (returns empty if outside of joint bounds)
            if 0 > jointNumber || jointNumber > self.robot.model.n
                %OUT OF BOUNDS
                self.errorCode = 2;
                error('Joint out of bounds');
            else %SELECTION WITHIN JOINT NUMBER
                    eeTr = self.robot.model.base.T;
                    currentq = self.qCurrent;
                    jointArray = zeros(3,jointNumber);
                    isPris = self.robot.model.isprismatic;

                    for i = 1:jointNumber
                        if isPris(i)
                            disp('prismatic joint at:')
                            disp(i)
                            theta = self.robot.model.links(i).offset + pi;
                            d = self.robot.model.links(i).d + currentq(i);
                            a = self.robot.model.links(i).a;
                            alpha = self.robot.model.links(i).alpha;
                        else
                            theta = self.robot.model.links(i).offset + currentq(i);
                            d = self.robot.model.links(i).d;
                            a = self.robot.model.links(i).a;
                            alpha = self.robot.model.links(i).alpha;
    
                            if isempty(theta)
                                theta = 0;
                            end
                            if isempty(alpha)
                                alpha = 0;
                            end
                        end
    
                        Tr = eye(4) * ...
                            trotz(rad2deg(theta), 'deg') * ...
                            transl(a,0,d) * ...
                            trotx(rad2deg(alpha), 'deg');

                        eeTr = eeTr * Tr;
                        jointArray(1,i) = eeTr(1,4);
                        jointArray(2,i) = eeTr(2,4);
                        jointArray(3,i) = eeTr(3,4);
                    end
                jointPose = eeTr;

                plot3(jointArray(1,:), ...
                      jointArray(2,:), ...
                      jointArray(3,:),'*-w');
            end
        end
%-------------------------------------------------------------------------%
        function [goalReached,err] = moveToNextPoint(self,desiredTr,qPath)
            delay = GetClockSpeed() / length(qPath);
            goalReached = false;
            err = 0;
                    disp('moveToNextPoint - SetGrabTarget:')
                    disp('Transform:')
                    disp(self.gripTargetTr)
                    disp('handle:')
                    disp(self.gripTarget_h)
                    disp('vertices:')
                    disp(self.gripTargetVerts)
            
            if isequal(self.currentState,armState.EStop)
                disp('OPERATIONS CANNOT CONTINUE UNTIL ESTOP RELEASED')
            else
                for stepCurrent = 1:length(qPath(:,1))
                    self.robot.model.animate(qPath(stepCurrent,:));

                    self.grabTarget(self.gripperCloseFlag);

                    % if ~isempty(self.gripper)
                    %     self.gripper.model.base = self.currentTr * inv(self.gripper.model.base);
                    %     nextVertices = (brickVerts(:,1:3) * brickTf(1:3,1:3)') + brickTf(1:3,4)';
                    % 
                    %     set(brickCurrent_h,'Vertices',nextVertices)
                    % end
                    % 
                    % if ~isempty(self.gripTarget)
                    %     gripTarget = robotEE * inv(brickCurrentPose);
                    %     nextVertices = (brickVerts(:,1:3) * brickTf(1:3,1:3)') + brickTf(1:3,4)';
                    % 
                    %     set(brickCurrent_h,'Vertices',nextVertices)
                    % end
                    drawnow();
                
                    robotPos = self.robot.model.getpos;
                    self.currentTr = self.robot.model.fkine(robotPos).T;
                    
                    % Plot End-Effector position during steps
                    plot3(self.currentTr(1,4), ...
                          self.currentTr(2,4), ...
                          self.currentTr(3,4),'.-b');
                    pause(delay);
                end

                % Plot final End-Effector position (compare to desiredTr)
                plot3(self.currentTr(1,4), ...
                      self.currentTr(2,4), ...
                      self.currentTr(3,4),'o:r');
                
                err = norm(transl(desiredTr) - transl(self.currentTr));
                if self.ikErrorMax > err
                    % Goal flag, final pose has been reached within tolerance
                    disp('Result within IKine max')
                    goalReached = true;
                end
            end
        end
%-------------------------------------------------------------------------%
        function qPath = genIKPath(self,desiredTr,profile,coordinateFrame)
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
                    eeTr = self.robot.model.fkine(self.robot.model.getpos).T;
                    disp(desiredTr)
        
                    manip = zeros(1,steps);
                    err = nan(self.jointCount,steps);
                    
                    x = linspace(eeTr(1,4),desiredTr(1,4),steps);
                    y = linspace(eeTr(2,4),desiredTr(2,4),steps);
                    z = linspace(eeTr(3,4),desiredTr(3,4),steps);
                    P = zeros(1,steps);
                    R = zeros(1,steps);
                    Y = zeros(1,steps);
        
                    X = [x',y',z',P',R',Y']';%%%%%%%%%%%%%
                    X = X(1:self.jointCount,:);
        
                    for i = 1:steps-1
                        xd = (X(:,i+1) - X(:,i))/dT;
                        switch coordinateFrame
                            case 'Global'
                            J = self.robot.model.jacob0(qPath(i,:));
                            case 'Local'
                            J = self.robot.model.jacobe(qPath(i,:));
                        end
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
                self.currentState = armState.AssignedNextTr;
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