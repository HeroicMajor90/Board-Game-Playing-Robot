classdef uArmRobot < SerialLink
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        L1 = Link('d',0,'a',2,'alpha',-pi/2,'offset',0,'qlim', [-pi/2,pi/2]);
        L2 = Link('d',0,'a',15,'alpha',0,'offset',-pi/1.5,'qlim', [(-1.5/3*pi),(1.5/3*pi)]);
        L3 = Link('d',0,'a',16,'alpha',0,'offset',+pi/1.5,'qlim', [-pi/3,(2/3*pi)]);
        L4 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim', [-2*pi,2*pi]);
        
        QA = [0 0 0 0];
    end
    
    properties
        eStop = zeros(1, 8);
        uArm = SerialLink([uArmRobot.L1 uArmRobot.L2 uArmRobot.L3 uArmRobot.L4], ...
                           'name', 'uArm');
        arduinoBoard = arduino();
        collisionPoint = [-50,-50,-50];   
        
        endEffectorVertexCount;
        endEffectorVerts;
        midPointEndEffector = [-4,0,7];
        endEffector_h;
        
        tokenVertexCount;
        tokenVerts;
        tokenHandlers;
        
        tokenVertexCount2;
        tokenVerts2;
        tokenHandlers2;
    end
    
    methods
        function self = uArmRobot()
            self = self.initEndEffector();
            self.initGameBoard();
            self.initEnclosure();
            self.initTokens();
            self.uArm.plot(self.QA);
        end
        
        function self = initEndEffector(self)
            [f, v, data] = read_ply('EndEffector.ply', 'tri');
            self.endEffectorVertexCount = size(v,1);
            self.endEffectorVerts = v - repmat(self.midPointEndEffector, self.endEffectorVertexCount, 1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            self.endEffector_h = trisurf(f, self.endEffectorVerts(:,1)+10.5, self.endEffectorVerts(:,2)+0, ...
                                         self.endEffectorVerts(:,3) + 13 ,'FaceVertexCData', vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on;
        end
        
        function initGameBoard(self)
            [f, v, data] = read_ply('board.ply', 'tri');
            boardVertexCount = size(v,1);
            midPointboard = sum(v)/boardVertexCount;
            boardVerts = v - repmat(midPointboard,boardVertexCount,1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            trisurf(f,boardVerts(:,1)+20,boardVerts(:,2), boardVerts(:,3),'FaceVertexCData', vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on;
        end
        
        function initEnclosure(self)
            [f, v, data] = read_ply('enclosure.ply', 'tri');
            enclosureVertexCount = size(v, 1);
            midPointenclosure = sum(v)/enclosureVertexCount;
            enclosureVerts = v - repmat(midPointenclosure, enclosureVertexCount, 1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            trisurf(f,enclosureVerts(:,1)+30, enclosureVerts(:,2), enclosureVerts(:,3)+20, 'FaceVertexCData', vertexColours, 'EdgeColor','interp','EdgeLighting','flat');
            hold on;
        end
        
        function initTokens(self)
            [f, v, data] = read_ply('token.ply','tri');
            self.tokenVertexCount = size(v,1);
            midPointtoken = sum(v)/self.tokenVertexCount;
            self.tokenVerts = v - repmat(midPointtoken, self.tokenVertexCount, 1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            for idx = 1:5
                self.tokenHandlers{idx} = trisurf(f, self.tokenVerts(:,1)+20, self.tokenVerts(:,2)+12, self.tokenVerts(:,3)+ idx - 1,'FaceVertexCData', vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            end
            hold on;
            
            [f, v, data] = read_ply('token2.ply','tri');
            self.tokenVertexCount2 = size(v,1);
            midPointtoken = sum(v)/self.tokenVertexCount2;
            self.tokenVerts2 = v - repmat(midPointtoken, self.tokenVertexCount2, 1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            for idx = 1:5
                self.tokenHandlers2{idx} = trisurf(f, self.tokenVerts2(:,1)+20, self.tokenVerts2(:,2)-12, self.tokenVerts2(:,3)+ idx - 1,'FaceVertexCData', vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            end
            hold on;
        end
        
        function endEffector = getEndEffector(self, qValues)
            endEffector = self.uArm.fkine(qValues);
        end
        
        function pos = getPos(self)
            pos = self.uArm.getpos();
        end
        
        function eeAngle = plot(self, qValues)
            pos = self.uArm.fkine(qValues); 
            eeAngle = rotz(qValues(1,1));  %get angle current and rotation matrix to add to below
            endEffectorPose =[eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4);eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4);eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4);0,0,0,1;];
            updatedPoints =[endEffectorPose * [self.endEffectorVerts,ones(self.endEffectorVertexCount,1)]']';
            self.endEffector_h.Vertices = updatedPoints(:,1:3);
            
            self.uArm.plot(qValues);
            hold on;
        end
        
        function iseStop(self)
            
            function emergency = readEmergencyStops(self)
                releaseEStop = readDigitalPin(self.arduinoBoard, 'D7');
                if releaseEStop
                    self.eStop(1,:) = 0;
                end
                
                buttonEstop = readDigitalPin(self.arduinoBoard,'D5'); %e stop
                if buttonEstop <= 0
                    self.eStop(1,1) = 1;
                end

                reedEstop = readDigitalPin(self.arduinoBoard,'D4'); %reed switch
                if reedEstop > 0
                    self.eStop(1,3) = 1;
                end

                microEstop = readDigitalPin(self.arduinoBoard,'D6'); %e stop
                if microEstop > 0
                    self.eStop(1,2) = 1;
                end
                
                emergency = any(self.eStop);
            end
            
            %tests for estop conditions
            
            if nargin == 1
                releaseStop = false
            end
            
            writeDigitalPin(self.arduinoBoard, 'D9', 1); %safety light
            
            stop = readEmergencyStops(self);
            
            while stop ~= 0
                display('stopped')
                self.eStop
                stop = readEmergencyStops(self);
            end
            
            writeDigitalPin(self.arduinoBoard, 'D9', 0);
        end
        
        function makeMove(self, row, col, player, board)
            goalXYZ = [20 12 4-sum(board.board(:) == player)];
            
            if player == board.PLAYER_X
                goalXYZ(2) = -goalXYZ(2);
            end
            
            self.moveTo(goalXYZ);
            
            goalXYZ = [(30 - row*5), (-10 + col * 5), 1];
            self.moveTo(goalXYZ, player, board);
        end
        
        function moveTo(self, goalXYZ, player, board)
            
            function isInRange(reach)
                if reach > 30
                    disp('Out of range Far')
                end

                if reach < 10
                    disp('Out of range Near')
                end

                if goalXYZ(1,3) > 19
                    disp('Out of range Height')
                end

                if goalXYZ(1,3) < 0
                    disp('Out of range Depth')
                end
            end
            
            self.iseStop(); %check for safety
            %create bounding boxes
            %create square radius size R
            Z1= 4;Z2 = 3;Z3 = 17;Z4 = 18;Z5 = 8;
            R= 8;
            Zs= [Z1, Z2, Z3, Z4, Z5;];
            boxesA = self.recPris(R,Zs);
            %R= R/2;
            boxesB = self.recPris(R,Zs);
            %so to create a rectanglular prism we need 8 points, 4 at height 0 and 4 at
            %height Z
            boxesA = self.boxUpdate(boxesA);
            boxesB = self.boxUpdate(boxesB);
            %check bounding boxes
            self = self.isCollision(boxesA, self.collisionPoint, 1);    
            self = self.isCollision(boxesB, self.collisionPoint, 2);

            steps = 15;
            %get angles
            qa = self.uArm.getpos();
            endEffector = self.uArm.fkine(qa);
            qNow = self.uArm.getpos(); %current joint angles
            %required L1 rotation to get end effector pose
            angleGoal = radtodeg(atan((goalXYZ(1,2)/goalXYZ(1,1))))
            angleCurrent =radtodeg(atan((endEffector(2,4)/endEffector(1,4))))
            angleTotal = angleGoal %+ angleCurrent
            eNow = transl([(endEffector(1,4)-(3.5*cos(degtorad(angleTotal)))),(endEffector(2,4)-(3.5*sin(degtorad(angleTotal)))),5]);
            reach = (abs(goalXYZ(1,1))+abs(goalXYZ(1,2))); %of arm gives the amount of reach needed which then determines the
            %angle of L3 with regards to the height required.
            
            isInRange(reach);

            %end effector wanted orientation (L3 orientation)
            arg = rotz(angleTotal);
            %now goalXYZ offset to account for L4 and end effector
            %if angleTotal >=1
            offsetEE = [(goalXYZ(1,1)-(3.5*cos(degtorad(angleTotal)))),(goalXYZ(1,2)-(3.5*sin(degtorad(angleTotal)))),(goalXYZ(1,3)+6)]
            %else
            %   offsetEE = [(goalXYZ(1,1)-(3.5*cos(abs(angleTotal)))),(goalXYZ(1,2)+(3.5*sin(abs(angleTotal)))), (goalXYZ(1,3)+6)];
            %end
            %change goalXYZ to offsetEE when end effector is modeled in
            eEPoint = [arg(1,1),arg(1,2),arg(1,3),offsetEE(1,1); arg(2,1),arg(2,2),arg(2,3),offsetEE(1,2); arg(3,1),arg(3,2),arg(3,3),offsetEE(1,3);endEffector(4,1),endEffector(4,2),endEffector(4,3),endEffector(4,4);]
            goalPoint = [arg(1,1),arg(1,2),arg(1,3),goalXYZ(1,1); arg(2,1),arg(2,2),arg(2,3),goalXYZ(1,2); arg(3,1),arg(3,2),arg(3,3),goalXYZ(1,3);endEffector(4,1),endEffector(4,2),endEffector(4,3),endEffector(4,4);]

            qValues = self.uArm.ikcon(goalPoint); %goal joint values
            eValues = self.uArm.ikcon(eEPoint);  %hopefully same as goalPoint just offset to account for end effector
            qMatrix = jtraj(qNow,eValues,steps);
            eMatrix = ctraj(eNow,self.uArm.fkine(qValues),steps);
            
            while steps >= 1

                qCurrent= self.uArm.getpos();
                %create and move bounding boxes
                boxesB = self.boxUpdate(boxesB);
                self = self.isCollision(boxesB, self.collisionPoint, 1);
                self = self.isCollision(boxesB, self.collisionPoint, 2);
                self.iseStop();
                %plot end effector with arm
                qValues = qMatrix((16-steps), :);
                eeAngle = self.plot(qValues);
                pos = self.uArm.fkine(qValues);

                if nargin == 4
                    tokenSelector = 5 - sum(board.board(:) == player);
                    
                    if player == board.PLAYER_X
                        tokenPose = [eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4)+(3.5*cos((atan((pos(2,4)/pos(1,4))))));eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4)+(3.5*sin((atan((pos(2,4)/pos(1,4))))));eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4)-6.5;0,0,0,1;];
                        updatedPoints = [tokenPose * [self.tokenVerts2, ones(self.tokenVertexCount2, 1)]' ]';
                        self.tokenHandlers2{tokenSelector}.Vertices = updatedPoints(:, 1:3);
                    elseif player == board.PLAYER_O
                        tokenPose = [eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4)+(3.5*cos((atan((pos(2,4)/pos(1,4))))));eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4)+(3.5*sin((atan((pos(2,4)/pos(1,4))))));eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4)-6.5;0,0,0,1;];
                        updatedPoints = [tokenPose * [self.tokenVerts, ones(self.tokenVertexCount, 1)]' ]';
                        self.tokenHandlers{tokenSelector}.Vertices = updatedPoints(:, 1:3);
                    end
                end
                
                steps = steps - 1;
            end
            hold on;
            
        end
        
        function boxesB = recPris(self, R, Zs)
            %create square "radius" size R
            square1 = [-R, -R; -R, +R; +R, +R; +R, -R;]; % 4 corners

            rectangularPrism1 = [square1(1,1), square1(1,2), 0;     square1(2,1), square1(2,2),0;   square1(3,1), square1(3,2),0;   square1(4,1), square1(4,2),0;   square1(1,1), square1(1,2), Zs(1,1);     square1(2,1), square1(2,2),Zs(1,1);    square1(3,1), square1(3,2),Zs(1,1);     square1(4,1), square1(4,2),Zs(1,1);];
            rectangularPrism2 = [square1(1,1), square1(1,2), 0;     square1(2,1), square1(2,2),0;   square1(3,1), square1(3,2),0;   square1(4,1), square1(4,2),0;   square1(1,1), square1(1,2), Zs(1,2);     square1(2,1), square1(2,2),Zs(1,2);    square1(3,1), square1(3,2),Zs(1,2);     square1(4,1), square1(4,2),Zs(1,2);];
            rectangularPrism3 = [square1(1,1), square1(1,2), 0;     square1(2,1), square1(2,2),0;   square1(3,1), square1(3,2),0;   square1(4,1), square1(4,2),0;   square1(1,1), square1(1,2), Zs(1,3);     square1(2,1), square1(2,2),Zs(1,3);    square1(3,1), square1(3,2),Zs(1,3);     square1(4,1), square1(4,2),Zs(1,3);];
            rectangularPrism4 = [square1(1,1), square1(1,2), 0;     square1(2,1), square1(2,2),0;   square1(3,1), square1(3,2),0;   square1(4,1), square1(4,2),0;   square1(1,1), square1(1,2), Zs(1,4);     square1(2,1), square1(2,2),Zs(1,4);    square1(3,1), square1(3,2),Zs(1,4);     square1(4,1), square1(4,2),Zs(1,4);];
            rectangularPrism5 = [square1(1,1), square1(1,2), 0;     square1(2,1), square1(2,2),0;   square1(3,1), square1(3,2),0;   square1(4,1), square1(4,2),0;   square1(1,1), square1(1,2), Zs(1,5);     square1(2,1), square1(2,2),Zs(1,5);    square1(3,1), square1(3,2),Zs(1,5);     square1(4,1), square1(4,2),Zs(1,5);];

            boxesB = [rectangularPrism1, rectangularPrism2, rectangularPrism3, rectangularPrism4, rectangularPrism5;];
        end
        
        function boxesB = boxUpdate(self, boxesB)
            qCurrent= self.uArm.getpos();
            combinedMatrix = boxesB;
            aBox= combinedMatrix(:,1:3); %bounding box matrix of base
            bBox= combinedMatrix(:,4:6); %bounding box matrix of L1
            cBox= combinedMatrix(:,7:9); %bounding box matrix of L2
            dBox= combinedMatrix(:,10:12); %bounding box matrix of L3
            eBox= combinedMatrix(:,13:15); %bounding box matrix of end effector
            %move bounding boxes to locations
            aBoxPose = self.uArm.base;
            aUpdatedPoints =[aBoxPose * [aBox,ones(8,1)]']';
            aBox = aUpdatedPoints(:,1:3);

            bBoxPose = self.uArm.A([1], qCurrent);
            bUpdatedPoints =[bBoxPose * [bBox,ones(8,1)]']';
            bBox = bUpdatedPoints(:,1:3);

            cBoxPose = self.uArm.A([1,2], qCurrent);
            cUpdatedPoints =[cBoxPose * [cBox,ones(8,1)]']';
            cBox = cUpdatedPoints(:,1:3);

            dBoxPose = self.uArm.A([1,2,3], qCurrent);
            dUpdatedPoints =[dBoxPose * [dBox,ones(8,1)]']';
            dBox = dUpdatedPoints(:,1:3);

            eBoxPose = self.uArm.fkine(qCurrent);
            eUpdatedPoints =[eBoxPose * [eBox,ones(8,1)]']';
            eBox = eUpdatedPoints(:,1:3);

            boxesB = [aBox,bBox,cBox,dBox,eBox,];
        end

        function self = isCollision(self, boxes, collisionPoint, bound)


            plot3(collisionPoint(1,1), collisionPoint(1,2), collisionPoint(1,3), 'go'); hold on;
            %arbitrary values for now
            %test each axis against collisions with inpolygon()
            %[in] = inpolygon(xq,yq,xv,yv) v = bounding box q = point query
            %xvAX is the points of concern with the xy axis check for box A
            xvAX = boxes(:,1);
            yvAX = boxes(:,2);
            %xvAY is the 2nd axis points for check yz axis
            xvAY = boxes(:,2);
            yvAY = boxes(:,3);
            %xvAZ is the final axis points for check xz axis
            xvAZ = boxes(:,1);
            yvAZ = boxes(:,3);
            %b box points
            xvBX = boxes(:,4);
            yvBX = boxes(:,5);
            xvBY = boxes(:,5);
            yvBY = boxes(:,6);
            xvBZ = boxes(:,4);
            yvBZ = boxes(:,6);
            %c box points
            xvCX = boxes(:,7);
            yvCX = boxes(:,8);
            xvCY = boxes(:,8);
            yvCY = boxes(:,9);
            xvCZ = boxes(:,7);
            yvCZ = boxes(:,9);
            %d box points
            xvDX = boxes(:,10);
            yvDX = boxes(:,11);
            xvDY = boxes(:,11);
            yvDY = boxes(:,12);
            xvDZ = boxes(:,10);
            yvDZ = boxes(:,12);
            %e box points
            xvEX = boxes(:,13);
            yvEX = boxes(:,14);
            xvEY = boxes(:,14);
            yvEY = boxes(:,15);
            xvEZ = boxes(:,13);
            yvEZ = boxes(:,15);
            %checks if the point overlaps the 2D polygon created by the
            %bounding box. If the point overlaps in all three planes then there
            %is collision.
            %A box collision test
            if (inpolygon(collisionPoint(1,1),collisionPoint(1,2),xvAX,yvAX)&&inpolygon(collisionPoint(1,2),collisionPoint(1,3),xvAY,yvAY)&&inpolygon(collisionPoint(1,1),collisionPoint(1,3),xvAZ,yvAZ));
                %if bound == 2;
                    self.eStop(1,8) = 1;
                %else
                    %change velocity profile of arm to reduce potential collision
                %end
            end
            %B box collision test
            if (inpolygon(collisionPoint(1,1),collisionPoint(1,2),xvBX,yvBX)&&inpolygon(collisionPoint(1,2),collisionPoint(1,3),xvBY,yvBY)&&inpolygon(collisionPoint(1,1),collisionPoint(1,3),xvBZ,yvBZ));
                %if bound == 2
                    self.eStop(1,8) = 1;
                %else
                    %change vel
                %end
            end
            %C box collision test
            if (inpolygon(collisionPoint(1,1),collisionPoint(1,2),xvCX,yvCX)&&inpolygon(collisionPoint(1,2),collisionPoint(1,3),xvCY,yvCY)&&inpolygon(collisionPoint(1,1),collisionPoint(1,3),xvCZ,yvCZ));
                %if bound == 2
                    self.eStop(1,8) = 1;
                %else
                    %change vel
                %end
            end
            %D box collision test
            if (inpolygon(collisionPoint(1,1),collisionPoint(1,2),xvDX,yvDX)&&inpolygon(collisionPoint(1,2),collisionPoint(1,3),xvDY,yvDY)&&inpolygon(collisionPoint(1,1),collisionPoint(1,3),xvDZ,yvDZ));
                %if bound == 2
                    self.eStop(1,8) = 1;
                %else
                    %change vel
                %end
            end
            %E box collision test
            if (inpolygon(collisionPoint(1,1),collisionPoint(1,2),xvEX,yvEX)&&inpolygon(collisionPoint(1,2),collisionPoint(1,3),xvEY,yvEY)&&inpolygon(collisionPoint(1,1),collisionPoint(1,3),xvEZ,yvEZ));
                if bounds == 2;
                    self.eStop(1,8) = 1;
                else
                    %change vel
                end
            end
            self.iseStop();
        end
        
    end
    
end

