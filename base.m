function base()
clf;
clear all;
clc
goalXYZ= zeros(3,1);
eStop =zeros(1,8); %1 estop button, 2 microswitch,  3 reed switch,  4 empty for lightgate,  5 empty,    6 empty,    7 empty,    8 collisions
%%%%%%%%%  Initialise myR1 robot
L1 = Link('d',0,'a',2,'alpha',-pi/2,'offset',0,'qlim', [-pi/2,pi/2]);
L2 = Link('d',0,'a',15,'alpha',0,'offset',-pi/1.5,'qlim', [(-1.5/3*pi),(1.5/3*pi)]);
L3 = Link('d',0,'a',16,'alpha',0,'offset',+pi/1.5,'qlim', [-pi/3,(2/3*pi)]);
L4 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim', [-2*pi,2*pi]);

uARM = SerialLink([L1 L2 L3 L4], 'name', 'uARM')
uARM.base

uARM.base = transl(0, 0, 0) ;%Base position uARM
q1 =0; q2 =0; q3 =0; q4 =0; %initialise q states
qa = [q1,q2,q3,q4];

%initialise arduino
a = arduino();
gamestate = -1;
progress = 0; %number of pieces that have been placed
collisionPoint = [-50,-50,-50;];
%initialise end effector
[f,v,data] = read_ply('EndEffector.ply','tri');
EndEffectorVertexCount = size(v,1);
midPointEndEffector = [-4,0,7];%offset to part so it sits correctly on simulation
EndEffectorVerts = v - repmat(midPointEndEffector,EndEffectorVertexCount,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
EndEffector_h = trisurf(f,EndEffectorVerts(:,1)+10.5,EndEffectorVerts(:,2)+0, EndEffectorVerts(:,3)+13 ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on;
%initialise game board 
%center position (20,0) Top right (25,-5) bottom left (15,5)
[f,v,data] = read_ply('board.ply','tri');
boardVertexCount = size(v,1);
midPointboard = sum(v)/boardVertexCount;
boardVerts = v - repmat(midPointboard,boardVertexCount,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
board_h = trisurf(f,boardVerts(:,1)+20,boardVerts(:,2), boardVerts(:,3),'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on;
%initialise enclosure
[f,v,data] = read_ply('enclosure.ply','tri');
enclosureVertexCount = size(v,1);
midPointenclosure = sum(v)/enclosureVertexCount;
enclosureVerts = v - repmat(midPointenclosure,enclosureVertexCount,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
enclosure_h = trisurf(f,enclosureVerts(:,1)+30,enclosureVerts(:,2), enclosureVerts(:,3)+20,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on;
%initialise tokens player 1 tokens stored in stack at (20,+12)
%stack top at height 5 width of 1
[f,v,data] = read_ply('token.ply','tri');
tokenVertexCount = size(v,1);
midPointtoken = sum(v)/tokenVertexCount;
tokenVerts = v - repmat(midPointtoken,tokenVertexCount,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

tokenA_h = trisurf(f,tokenVerts(:,1)+20,tokenVerts(:,2)+12, tokenVerts(:,3)+0,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
tokenB_h = trisurf(f,tokenVerts(:,1)+20,tokenVerts(:,2)+12, tokenVerts(:,3)+1,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
tokenC_h = trisurf(f,tokenVerts(:,1)+20,tokenVerts(:,2)+12, tokenVerts(:,3)+2,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
tokenD_h = trisurf(f,tokenVerts(:,1)+20,tokenVerts(:,2)+12, tokenVerts(:,3)+3,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
tokenE_h = trisurf(f,tokenVerts(:,1)+20,tokenVerts(:,2)+12, tokenVerts(:,3)+4,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

%initialise tokens player 2 tokens stored in stack at (20,-12) 
%stack top at height 5 width of 1
[f,v,data] = read_ply('token2.ply','tri');
token2VertexCount = size(v,1);
midPointtoken2 = sum(v)/token2VertexCount;
token2Verts = v - repmat(midPointtoken2,token2VertexCount,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

token2A_h = trisurf(f,token2Verts(:,1)+20,token2Verts(:,2)-12, token2Verts(:,3)+0,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
token2B_h = trisurf(f,token2Verts(:,1)+20,token2Verts(:,2)-12, token2Verts(:,3)+1,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
token2C_h = trisurf(f,token2Verts(:,1)+20,token2Verts(:,2)-12, token2Verts(:,3)+2,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
token2D_h = trisurf(f,token2Verts(:,1)+20,token2Verts(:,2)-12, token2Verts(:,3)+3,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
token2E_h = trisurf(f,token2Verts(:,1)+20,token2Verts(:,2)-12, token2Verts(:,3)+4,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
%program commands
uARM.plot(qa);
hold on;
% TURN 1
%  goalXYZ = [20,12,5];%stack position height minus 1
%  moveTo(goalXYZ,eStop);
%  gamestate = 0;%advance gamestate for black
% goalXYZ = [15,5,1]; %move token here
%  moveTo(goalXYZ,eStop);
%  gamestate = -1; %null gamestate to stop carrying token
%  goalXYZ = [20,-12,5];%stack position height minus 1
%  moveTo(goalXYZ,eStop);
%  gamestate = 5;%advance gamestate for white
%  goalXYZ = [20,0,1];
%  moveTo(goalXYZ,eStop);
%  gamestate = -1;%null gamestate to stop carrying token
%  %TURN 2
%   goalXYZ = [20,12,4];%stack position height minus 1
%  moveTo(goalXYZ,eStop);
%  gamestate = 1;%advance gamestate for black
% goalXYZ = [25,-5,1];
%  moveTo(goalXYZ,eStop);
%  gamestate = -1;%null gamestate to stop carrying token
%  goalXYZ = [20,-12,4];%stack position height minus 1
%  moveTo(goalXYZ,eStop);
%  gamestate = 6;%advance gamestate for white
%  goalXYZ = [25,5,1];
%  moveTo(goalXYZ,eStop);
%  gamestate = -1;%null gamestate to stop carrying token
%  %TURN 3
%    goalXYZ = [20,12,3];%stack position height minus 1
%  moveTo(goalXYZ,eStop);
%  gamestate = 2;%advance gamestate for black
% goalXYZ = [15,-5,1];
%  moveTo(goalXYZ,eStop);
%  gamestate = -1;%null gamestate to stop carrying token
%  goalXYZ = [20,-12,3];%stack position height minus 1
%  moveTo(goalXYZ,eStop);
%  gamestate = 7; %advance gamestate for white
%  goalXYZ = [20,-5,1];
%  moveTo(goalXYZ,eStop);
%  gamestate = -1;%null gamestate to stop carrying token
%  %TURN 4
%     goalXYZ = [20,12,2]; %stack position height minus 1
%  moveTo(goalXYZ,eStop);
%  gamestate = 3; %advance gamestate for black
% goalXYZ = [15,0,1];  
%  moveTo(goalXYZ,eStop);
%  gamestate = -1;%null gamestate to stop carrying token

%incrementMove(4);       %case 0+X 1-X 2+Y 3-Y 4+Z 5-Z



    function iseStop(eStop)
        %tests for estop conditions
         buttonEstop = readDigitalPin(a,'D4');
        pause(1/100);           %allow time for pin to be read
         if buttonEstop < 1 
        eStop(1,1) = 1;
        end
        stopped = any(eStop);
        while stopped ~= 0
            display('stopped')
            eStop
            pause(2.0);
             buttonEstop = readDigitalPin(a,'D4');
             pause(1/100);           %allow time for pin to be read 10ms
             if buttonEstop > 0      %check for button to be manually reset
                 eStop(1,1) = 0;
                stopped = any(eStop);
             end
        end
    end

    function incrementQ(QGUI)
        qa = uARM.getpos(); %get current positions of arm
        incrementQ = degtorad(4);
        switch(QGUI)%case 0+X 1-X 2+Y 3-Y 4+Z 5-Z
            case 0 %x+
                qCurrent = qa;
                qa(1,1) = qa(1,1)+incrementQ; %correctly gets required joint angles
                %plot end effector with arm
                pos = uARM.fkine(qa);
                
                eeAngle = rotz(qCurrent(1,1));  %get angle current and rotation matrix to add to below
                EndEffectorPose =[eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4);eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4);eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4);0,0,0,1;];
                updatedPoints =[EndEffectorPose * [EndEffectorVerts,ones(EndEffectorVertexCount,1)]']';
                EndEffector_h.Vertices = updatedPoints(:,1:3);
                uARM.plot(qa);
            case 1 %x-
                qCurrent = qa;
                qa(1,1) = qa(1,1)-incrementQ; %correctly gets required joint angles
                %plot end effector with arm
                pos = uARM.fkine(qa);
                
                eeAngle = rotz(qCurrent(1,1));  %get angle current and rotation matrix to add to below
                EndEffectorPose =[eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4);eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4);eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4);0,0,0,1;];
                updatedPoints =[EndEffectorPose * [EndEffectorVerts,ones(EndEffectorVertexCount,1)]']';
                EndEffector_h.Vertices = updatedPoints(:,1:3);
                uARM.plot(qa);
            case 2 %y+
                qCurrent = qa;
                qa(1,2) = qa(1,2)+incrementQ; %correctly gets required joint angles
                %plot end effector with arm
                pos = uARM.fkine(qa);
                
                eeAngle = rotz(qCurrent(1,1));  %get angle current and rotation matrix to add to below
                EndEffectorPose =[eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4);eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4);eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4);0,0,0,1;];
                updatedPoints =[EndEffectorPose * [EndEffectorVerts,ones(EndEffectorVertexCount,1)]']';
                EndEffector_h.Vertices = updatedPoints(:,1:3);
                uARM.plot(qa);
            case 3  %y-
                qCurrent = qa;
                qa(1,2) = qa(1,2)-incrementQ; %correctly gets required joint angles
                %plot end effector with arm
                pos = uARM.fkine(qa);
                
                eeAngle = rotz(qCurrent(1,1));  %get angle current and rotation matrix to add to below
                EndEffectorPose =[eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4);eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4);eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4);0,0,0,1;];
                updatedPoints =[EndEffectorPose * [EndEffectorVerts,ones(EndEffectorVertexCount,1)]']';
                EndEffector_h.Vertices = updatedPoints(:,1:3);
                uARM.plot(qa);
            case 4  %z+
                qCurrent = qa;
                qa(1,3) = qa(1,3)+incrementQ; %correctly gets required joint angles
                %plot end effector with arm
                pos = uARM.fkine(qa);
                
                eeAngle = rotz(qCurrent(1,1));  %get angle current and rotation matrix to add to below
                EndEffectorPose =[eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4);eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4);eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4);0,0,0,1;];
                updatedPoints =[EndEffectorPose * [EndEffectorVerts,ones(EndEffectorVertexCount,1)]']';
                EndEffector_h.Vertices = updatedPoints(:,1:3);
                uARM.plot(qa);
            case 5  %z-
                qCurrent = qa;
                qa(1,3) = qa(1,3)-incrementQ; %correctly gets required joint angles
                %plot end effector with arm
                pos = uARM.fkine(qa);
                
                eeAngle = rotz(qCurrent(1,1));  %get angle current and rotation matrix to add to below
                EndEffectorPose =[eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4);eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4);eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4);0,0,0,1;];
                updatedPoints =[EndEffectorPose * [EndEffectorVerts,ones(EndEffectorVertexCount,1)]']';
                EndEffector_h.Vertices = updatedPoints(:,1:3);
                uARM.plot(qa);
            end
    end
    function incrementMove(XYZ)  %case 0+X 1-X 2+Y 3-Y 4+Z 5-Z
        increment = 2;
        switch(XYZ)
            case 0 %move +1 increment in X
                %get current end effector position
                qa = uARM.getpos();
                L4Position = uARM.fkine(qa);
                
                angleCurrent =radtodeg(atan((L4Position(2,4)/L4Position(1,4))));
                
                if angleCurrent >=1
                    offsetEE = [(L4Position(1,4)+(3.5*sin(abs(angleCurrent)))),(L4Position(2,4)+(3.5*cos((angleCurrent)))),(L4Position(3,4)-6)];
                else
                    offsetEE = [(L4Position(1,4)+(3.5*cos(abs(angleCurrent)))),(L4Position(2,4)+(3.5*sin((angleCurrent)))), (L4Position(3,4)-6)];
                end
                %offsetEE is now the actual end of the robot
                offsetEE(1,1) = offsetEE(1,1)+increment; %add the increment
                goalXYZ = [offsetEE(1,1),offsetEE(1,2),offsetEE(1,3);];
                moveTo(goalXYZ,eStop);
            case 1 %move -1 increment in X
                %get current end effector position
                qa = uARM.getpos();
                L4Position = uARM.fkine(qa);
                
                angleCurrent =radtodeg(atan((L4Position(2,4)/L4Position(1,4))));
                
                if angleCurrent >=1
                    offsetEE = [(L4Position(1,4)+(3.5*sin(abs(angleCurrent)))),(L4Position(2,4)+(3.5*cos((angleCurrent)))),(L4Position(3,4)-6)];
                else
                    offsetEE = [(L4Position(1,4)+(3.5*cos(abs(angleCurrent)))),(L4Position(2,4)+(3.5*sin((angleCurrent)))), (L4Position(3,4)-6)];
                end
                %offsetEE is now the actual end of the robot
                offsetEE(1,1) = offsetEE(1,1)-increment; %add the increment
                goalXYZ = [offsetEE(1,1),offsetEE(1,2),offsetEE(1,3);];
                moveTo(goalXYZ,eStop);
            case 2 %move +1 increment in Y
                %get current end effector position
                qa = uARM.getpos();
                L4Position = uARM.fkine(qa);
                
                angleCurrent =radtodeg(atan((L4Position(2,4)/L4Position(1,4))));
                
                if angleCurrent >=1
                    offsetEE = [(L4Position(1,4)+(3.5*sin(abs(angleCurrent)))),(L4Position(2,4)+(3.5*cos((angleCurrent)))),(L4Position(3,4)-6)];
                else
                    offsetEE = [(L4Position(1,4)+(3.5*cos(abs(angleCurrent)))),(L4Position(2,4)+(3.5*sin((angleCurrent)))), (L4Position(3,4)-6)];
                end
                %offsetEE is now the actual end of the robot
                offsetEE(1,2) = offsetEE(1,2)+increment; %add the increment
                goalXYZ = [offsetEE(1,1),offsetEE(1,2),offsetEE(1,3);];
                moveTo(goalXYZ,eStop);
            case 3 %move -1 increment in Y
                %get current end effector position
                qa = uARM.getpos();
                L4Position = uARM.fkine(qa);
                
                angleCurrent =radtodeg(atan((L4Position(2,4)/L4Position(1,4))));
                
                if angleCurrent >=1
                    offsetEE = [(L4Position(1,4)+(3.5*sin(abs(angleCurrent)))),(L4Position(2,4)+(3.5*cos((angleCurrent)))),(L4Position(3,4)-6)];
                else
                    offsetEE = [(L4Position(1,4)+(3.5*cos(abs(angleCurrent)))),(L4Position(2,4)+(3.5*sin((angleCurrent)))), (L4Position(3,4)-6)];
                end
                %offsetEE is now the actual end of the robot
                offsetEE(1,2) = offsetEE(1,2)-increment; %add the increment
                goalXYZ = [offsetEE(1,1),offsetEE(1,2),offsetEE(1,3);];
                moveTo(goalXYZ,eStop);
            case 4 %move +1 increment in Z
                %get current end effector position
                qa = uARM.getpos();
                L4Position = uARM.fkine(qa);
                
                angleCurrent =radtodeg(atan((L4Position(2,4)/L4Position(1,4))));
                
                if angleCurrent >=1
                    offsetEE = [(L4Position(1,4)+(3.5*sin(abs(angleCurrent)))),(L4Position(2,4)+(3.5*cos((angleCurrent)))),(L4Position(3,4)-6)];
                else
                    offsetEE = [(L4Position(1,4)+(3.5*cos(abs(angleCurrent)))),(L4Position(2,4)+(3.5*sin((angleCurrent)))), (L4Position(3,4)-6)];
                end
                %offsetEE is now the actual end of the robot
                offsetEE(1,3) = offsetEE(1,3)+increment; %add the increment
                goalXYZ = [offsetEE(1,1),offsetEE(1,2),offsetEE(1,3);];
                moveTo(goalXYZ,eStop);
            case 5 %move -1 increment in Z
                %get current end effector position
                qa = uARM.getpos();
                L4Position = uARM.fkine(qa);
                
                angleCurrent =radtodeg(atan((L4Position(2,4)/L4Position(1,4))));
                
                if angleCurrent >=1
                    offsetEE = [(L4Position(1,4)+(3.5*sin(abs(angleCurrent)))),(L4Position(2,4)+(3.5*cos((angleCurrent)))),(L4Position(3,4)-6)];
                else
                    offsetEE = [(L4Position(1,4)+(3.5*cos(abs(angleCurrent)))),(L4Position(2,4)+(3.5*sin((angleCurrent)))), (L4Position(3,4)-6)];
                end
                %offsetEE is now the actual end of the robot
                offsetEE(1,3) = offsetEE(1,3)-increment; %add the increment
                goalXYZ = [offsetEE(1,1),offsetEE(1,2),offsetEE(1,3);];
                moveTo(goalXYZ,eStop);
        end
    end

    function moveTo(goalXYZ,eStop)
        iseStop(eStop); %check for safety
        %create bounding boxes
        %create square radius size R
        Z1= 4;Z2 = 3;Z3 = 17;Z4 = 18;Z5 = 8;
        R= 4;
        Zs= [Z1, Z2, Z3, Z4, Z5;];
        
        boxesB = recPris(R,Zs);
        %so to create a rectanglular prism we need 8 points, 4 at height 0 and 4 at
        %height Z
        boxesB = boxUpdate(boxesB);
        %check bounding boxes
        isCollision(boxesB,collisionPoint);
        
        steps = 15;
        %get angles
        qa= uARM.getpos();
        endEffector = uARM.fkine(qa);
        qNow = uARM.getpos(); %current joint angles
        %required L1 rotation to get end effector pose
        angleGoal = radtodeg(atan((goalXYZ(1,2)/goalXYZ(1,1))))
        angleCurrent =radtodeg(atan((endEffector(2,4)/endEffector(1,4))))
        angleTotal = angleGoal %+ angleCurrent
        eNow = transl([(endEffector(1,4)-(3.5*cos(degtorad(angleTotal)))),(endEffector(2,4)-(3.5*sin(degtorad(angleTotal)))),5]);
        reach = (abs(goalXYZ(1,1))+abs(goalXYZ(1,2))); %of arm gives the amount of reach needed which then determines the
        %angle of L3 with regards to the height required.
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
        
        qValues = uARM.ikcon(goalPoint); %goal joint values
        eValues = uARM.ikcon(eEPoint);  %hopefully same as goalPoint just offset to account for end effector
        qMatrix = jtraj(qNow,eValues,steps);
        eMatrix = ctraj(eNow,uARM.fkine(qValues),steps);
        while steps >= 1
            
            qCurrent= uARM.getpos();
            %create and move bounding boxes
            boxUpdate(boxesB);
            isCollision(boxesB,collisionPoint);
            iseStop(eStop);
            %plot end effector with arm
            pos = uARM.fkine(qMatrix((16-steps),:));
            epos = eMatrix(:,:,16-steps);
            eeAngle = rotz(qCurrent(1,1));  %get angle current and rotation matrix to add to below
            EndEffectorPose =[eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4);eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4);eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4);0,0,0,1;];
            updatedPoints =[EndEffectorPose * [EndEffectorVerts,ones(EndEffectorVertexCount,1)]']';
            EndEffector_h.Vertices = updatedPoints(:,1:3);
            uARM.plot(qMatrix((16-steps),:));
            switch (gamestate)       %moving tokens with arm needs end effector position
                case 0
                    tokenEPose = [eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4)+(3.5*cos((atan((pos(2,4)/pos(1,4))))));eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4)+(3.5*sin((atan((pos(2,4)/pos(1,4))))));eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4)-6.5;0,0,0,1;];
                    updatedPoints = [tokenEPose * [tokenVerts,ones(tokenVertexCount,1)]']';
                    tokenE_h.Vertices = updatedPoints(:,1:3);
                case 1
                    tokenDPose = [eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4)+(3.5*cos((atan((pos(2,4)/pos(1,4))))));eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4)+(3.5*sin((atan((pos(2,4)/pos(1,4))))));eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4)-6.5;0,0,0,1;];
                    updatedPoints = [tokenDPose * [tokenVerts,ones(tokenVertexCount,1)]']';
                    tokenD_h.Vertices = updatedPoints(:,1:3);
                case 2
                    tokenCPose = [eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4)+(3.5*cos((atan((pos(2,4)/pos(1,4))))));eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4)+(3.5*sin((atan((pos(2,4)/pos(1,4))))));eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4)-6.5;0,0,0,1;];
                    updatedPoints = [tokenCPose * [tokenVerts,ones(tokenVertexCount,1)]']';
                    tokenC_h.Vertices = updatedPoints(:,1:3);
                case 3
                    tokenBPose = [eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4)+(3.5*cos((atan((pos(2,4)/pos(1,4))))));eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4)+(3.5*sin((atan((pos(2,4)/pos(1,4))))));eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4)-6.5;0,0,0,1;];
                    updatedPoints = [tokenBPose * [tokenVerts,ones(tokenVertexCount,1)]']';
                    tokenB_h.Vertices = updatedPoints(:,1:3);
                case 4
                    tokenAPose = [eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4)+(3.5*cos((atan((pos(2,4)/pos(1,4))))));eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4)+(3.5*sin((atan((pos(2,4)/pos(1,4))))));eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4)-6.5;0,0,0,1;];
                    updatedPoints = [tokenAPose * [tokenVerts,ones(tokenVertexCount,1)]']';
                    tokenA_h.Vertices = updatedPoints(:,1:3);
                case 5
                    token2EPose = [eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4)+(3.5*cos((atan((pos(2,4)/pos(1,4))))));eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4)+(3.5*sin((atan((pos(2,4)/pos(1,4))))));eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4)-6.5;0,0,0,1;];
                    updatedPoints = [token2EPose * [token2Verts,ones(token2VertexCount,1)]']';
                    token2E_h.Vertices = updatedPoints(:,1:3);
                case 6
                    token2DPose = [eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4)+(3.5*cos((atan((pos(2,4)/pos(1,4))))));eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4)+(3.5*sin((atan((pos(2,4)/pos(1,4))))));eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4)-6.5;0,0,0,1;];
                    updatedPoints = [token2DPose * [token2Verts,ones(token2VertexCount,1)]']';
                    token2D_h.Vertices = updatedPoints(:,1:3);
                case 7
                    token2CPose = [eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4)+(3.5*cos((atan((pos(2,4)/pos(1,4))))));eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4)+(3.5*sin((atan((pos(2,4)/pos(1,4))))));eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4)-6.5;0,0,0,1;];
                    updatedPoints = [token2CPose * [token2Verts,ones(token2VertexCount,1)]']';
                    token2C_h.Vertices = updatedPoints(:,1:3);
                case 8
                    token2BPose = [eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4)+(3.5*cos((atan((pos(2,4)/pos(1,4))))));eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4)+(3.5*sin((atan((pos(2,4)/pos(1,4))))));eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4)-6.5;0,0,0,1;];
                    updatedPoints = [token2BPose * [token2Verts,ones(token2VertexCount,1)]']';
                    token2B_h.Vertices = updatedPoints(:,1:3);
                case 9
                    token2APose = [eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4)+(3.5*cos((atan((pos(2,4)/pos(1,4))))));eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4)+(3.5*sin((atan((pos(2,4)/pos(1,4))))));eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4)-6.5;0,0,0,1;];
                    updatedPoints = [token2APose * [token2Verts,ones(token2VertexCount,1)]']';
                    token2A_h.Vertices = updatedPoints(:,1:3);
            end
            steps = steps - 1;
        end
        hold on;
    end

    function boxesB = recPris(R,Zs)
        %create square "radius" size R
        R= 4;
        square1 = [-R, -R; -R, +R; +R, +R; +R, -R;]; % 4 corners
        
        rectangularPrism1 = [square1(1,1), square1(1,2), 0;     square1(2,1), square1(2,2),0;   square1(3,1), square1(3,2),0;   square1(4,1), square1(4,2),0;   square1(1,1), square1(1,2), Zs(1,1);     square1(2,1), square1(2,2),Zs(1,1);    square1(3,1), square1(3,2),Zs(1,1);     square1(4,1), square1(4,2),Zs(1,1);];
        rectangularPrism2 = [square1(1,1), square1(1,2), 0;     square1(2,1), square1(2,2),0;   square1(3,1), square1(3,2),0;   square1(4,1), square1(4,2),0;   square1(1,1), square1(1,2), Zs(1,2);     square1(2,1), square1(2,2),Zs(1,2);    square1(3,1), square1(3,2),Zs(1,2);     square1(4,1), square1(4,2),Zs(1,2);];
        rectangularPrism3 = [square1(1,1), square1(1,2), 0;     square1(2,1), square1(2,2),0;   square1(3,1), square1(3,2),0;   square1(4,1), square1(4,2),0;   square1(1,1), square1(1,2), Zs(1,3);     square1(2,1), square1(2,2),Zs(1,3);    square1(3,1), square1(3,2),Zs(1,3);     square1(4,1), square1(4,2),Zs(1,3);];
        rectangularPrism4 = [square1(1,1), square1(1,2), 0;     square1(2,1), square1(2,2),0;   square1(3,1), square1(3,2),0;   square1(4,1), square1(4,2),0;   square1(1,1), square1(1,2), Zs(1,4);     square1(2,1), square1(2,2),Zs(1,4);    square1(3,1), square1(3,2),Zs(1,4);     square1(4,1), square1(4,2),Zs(1,4);];
        rectangularPrism5 = [square1(1,1), square1(1,2), 0;     square1(2,1), square1(2,2),0;   square1(3,1), square1(3,2),0;   square1(4,1), square1(4,2),0;   square1(1,1), square1(1,2), Zs(1,5);     square1(2,1), square1(2,2),Zs(1,5);    square1(3,1), square1(3,2),Zs(1,5);     square1(4,1), square1(4,2),Zs(1,5);];
        
        boxesB = [rectangularPrism1, rectangularPrism2, rectangularPrism3, rectangularPrism4, rectangularPrism5;];
    end

    function boxesB = boxUpdate(boxesB)
        qCurrent= uARM.getpos();
        combinedMatrix = boxesB;
        aBox= combinedMatrix(:,1:3); %bounding box matrix of base
        bBox= combinedMatrix(:,4:6); %bounding box matrix of L1
        cBox= combinedMatrix(:,7:9); %bounding box matrix of L2
        dBox= combinedMatrix(:,10:12); %bounding box matrix of L3
        eBox= combinedMatrix(:,13:15); %bounding box matrix of end effector
        %move bounding boxes to locations
        aBoxPose = uARM.base;
        aUpdatedPoints =[aBoxPose * [aBox,ones(8,1)]']';
        aBox = aUpdatedPoints(:,1:3);
        
        bBoxPose = uARM.A([1], qCurrent);
        bUpdatedPoints =[bBoxPose * [bBox,ones(8,1)]']';
        bBox = bUpdatedPoints(:,1:3);
        
        cBoxPose = uARM.A([1,2], qCurrent);
        cUpdatedPoints =[cBoxPose * [cBox,ones(8,1)]']';
        cBox = cUpdatedPoints(:,1:3);
        
        dBoxPose = uARM.A([1,2,3], qCurrent);
        dUpdatedPoints =[dBoxPose * [dBox,ones(8,1)]']';
        dBox = dUpdatedPoints(:,1:3);
        
        eBoxPose = uARM.fkine(qCurrent);
        eUpdatedPoints =[eBoxPose * [eBox,ones(8,1)]']';
        eBox = eUpdatedPoints(:,1:3);
        
        boxesB = [aBox,bBox,cBox,dBox,eBox,];
    end

    function isCollision(boxesB,collisionPoint)
        
        
        plot3(collisionPoint(1,1),collisionPoint(1,2),collisionPoint(1,3),'go'); hold on;
        %arbitrary values for now
        %test each axis against collisions with inpolygon()
        %[in] = inpolygon(xq,yq,xv,yv) v = bounding box q = point query
        %xvAX is the points of concern with the xy axis check for box A
        xvAX = boxesB(:,1);
        yvAX = boxesB(:,2);
        %xvAY is the 2nd axis points for check yz axis
        xvAY = boxesB(:,2);
        yvAY = boxesB(:,3);
        %xvAZ is the final axis points for check xz axis
        xvAZ = boxesB(:,1);
        yvAZ = boxesB(:,3);
        %b box points
        xvBX = boxesB(:,4);
        yvBX = boxesB(:,5);
        xvBY = boxesB(:,5);
        yvBY = boxesB(:,6);
        xvBZ = boxesB(:,4);
        yvBZ = boxesB(:,6);
        %c box points
        xvCX = boxesB(:,7);
        yvCX = boxesB(:,8);
        xvCY = boxesB(:,8);
        yvCY = boxesB(:,9);
        xvCZ = boxesB(:,7);
        yvCZ = boxesB(:,9);
        %d box points
        xvDX = boxesB(:,10);
        yvDX = boxesB(:,11);
        xvDY = boxesB(:,11);
        yvDY = boxesB(:,12);
        xvDZ = boxesB(:,10);
        yvDZ = boxesB(:,12);
        %e box points
        xvEX = boxesB(:,13);
        yvEX = boxesB(:,14);
        xvEY = boxesB(:,14);
        yvEY = boxesB(:,15);
        xvEZ = boxesB(:,13);
        yvEZ = boxesB(:,15);
        %checks if the point overlaps the 2D polygon created by the
        %bounding box. If the point overlaps in all three planes then there
        %is collision.
        %A box collision test
        if (inpolygon(collisionPoint(1,1),collisionPoint(1,2),xvAX,yvAX)&&inpolygon(collisionPoint(1,2),collisionPoint(1,3),xvAY,yvAY)&&inpolygon(collisionPoint(1,1),collisionPoint(1,3),xvAZ,yvAZ));
            eStop(1,8) = 1;
        end
        %B box collision test
        if (inpolygon(collisionPoint(1,1),collisionPoint(1,2),xvBX,yvBX)&&inpolygon(collisionPoint(1,2),collisionPoint(1,3),xvBY,yvBY)&&inpolygon(collisionPoint(1,1),collisionPoint(1,3),xvBZ,yvBZ));
            eStop(1,8) = 1;
        end
        %C box collision test
        if (inpolygon(collisionPoint(1,1),collisionPoint(1,2),xvCX,yvCX)&&inpolygon(collisionPoint(1,2),collisionPoint(1,3),xvCY,yvCY)&&inpolygon(collisionPoint(1,1),collisionPoint(1,3),xvCZ,yvCZ));
            eStop(1,8) = 1;
        end
        %D box collision test
        if (inpolygon(collisionPoint(1,1),collisionPoint(1,2),xvDX,yvDX)&&inpolygon(collisionPoint(1,2),collisionPoint(1,3),xvDY,yvDY)&&inpolygon(collisionPoint(1,1),collisionPoint(1,3),xvDZ,yvDZ));
            eStop(1,8) = 1;
        end
        %E box collision test
        if (inpolygon(collisionPoint(1,1),collisionPoint(1,2),xvEX,yvEX)&&inpolygon(collisionPoint(1,2),collisionPoint(1,3),xvEY,yvEY)&&inpolygon(collisionPoint(1,1),collisionPoint(1,3),xvEZ,yvEZ));
            eStop(1,8) = 1;
        end
        iseStop(eStop);
    end
    end