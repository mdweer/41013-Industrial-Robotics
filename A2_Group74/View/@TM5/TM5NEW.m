classdef TM5 < RobotBaseClass
    %% TM5

    properties(Access = public)
        plyFileNameStem = 'TM5';
        defaultRealQ  = [0,pi/2,0,-pi/2,0,0,0];
    end
    
    methods
%% Constructor
        function self = TM5(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                    
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.homeQ = self.defaultRealQ;
            self.PlotAndColourRobot();
            drawnow
        end

%% CreateModel
        function CreateModel(self)

            link(1) = Link('d',0.145198,'a',0,'alpha',pi/2,'qlim',deg2rad([-270,270]), 'offset', 0);
            link(2) = Link('d',0.0,'a',-0.32900,'alpha',0,'qlim' ,deg2rad([-180,180]), 'offset',pi);
            link(3) = Link('d',0.0,'a',-0.31150,'alpha',0,'qlim' ,deg2rad([-155,155]), 'offset', 0);
            link(4) = Link('d',0.10600,'a', 0,'alpha',pi/2,'qlim',deg2rad([-180,180]), 'offset', 0);
            link(5) = Link('d',0.10600,'a',0,'alpha',-pi/2,'qlim',deg2rad([-180,180]), 'offset', 0);
            link(6) = Link('d',0.11315,'a', 0,'alpha',0,'qlim'   ,deg2rad([-270,270]), 'offset', 0);
            link(7) = Link('d',0.04949,'a', 0,'alpha',0,'qlim'   ,deg2rad([-360,360]), 'offset', 0);

      
            self.model = SerialLink(link,'name',self.name);
        end          
    end
end