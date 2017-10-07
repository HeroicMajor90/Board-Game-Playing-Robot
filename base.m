function base()
clf;
clear all;
clc
goalXYZ= zeros(3,1);
eStop =zeros(1,8);
%%%%%%%%%  Initialise myR1 robot
L1 = Link('d',0,'a',2,'alpha',-pi/2,'offset',0,'qlim', [-pi/2,pi/2]);
L2 = Link('d',0,'a',15,'alpha',0,'offset',-pi/1.5,'qlim', [(-1/3*pi),(1/3*pi)]);
L3 = Link('d',0,'a',16,'alpha',0,'offset',+pi/1.5,'qlim', [-pi/3,(2/3*pi)]);
L4 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim', [-2*pi,2*pi]);

uARM = SerialLink([L1 L2 L3 L4], 'name', 'uARM')
uARM.base

uARM.base = transl(0, 0, 0) ;%Base position uARM
q1 =0; q2 =0; q3 =0; q4 =0; %initialise q states
qa = [q1,q2,q3,q4];
positionPart = zeros (10, 10);
%initialise end effector
[f,v,data] = read_ply('EndEffector.ply','tri');
EndEffectorVertexCount = size(v,1);
midPointEndEffector = [-4,0,7];%offset to part so it sits correctly on simulation
EndEffectorVerts = v - repmat(midPointEndEffector,EndEffectorVertexCount,1);
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
EndEffector_h = trisurf(f,EndEffectorVerts(:,1)+10.5,EndEffectorVerts(:,2)+0, EndEffectorVerts(:,3)+13 ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on;

%create bounding boxes
%Base
boxSize= 4; %size of bounding boxes in collision detection. Creates square with sides twice as long as boxSize
%baseSquare= [uARM.base(1,4)-R,uARM.base(2,4)-R; uARM.base(1,4)-R,uARM.base(2,4)+R;uARM.base(1,4)+R,uARM.base(2,4)+R;uARM.base(1,4)-R,uARM.base(2,4)-R;]
%baseInnerSquare = [uARM.base(1,4)-(R/2),uARM.base(2,4)-(R/2); uARM.base(1,4)-(R/2),uARM.base(2,4)+(R/2);uARM.base(1,4)+(R/2),uARM.base(2,4)+(R/2);uARM.base(1,4)-(R/2),uARM.base(2,4)-(R/2);]
squareL1 = uARM.base ;%gives location of sqaure start points will need to move into collision detect function
squareL2 = uARM.A([1], qa);
squareL3 = uARM.A([1,2], qa);
squareL4 = uARM.A([1,2,3], qa);
squareEndE = uARM.fkine(qa);
uARM.plot(qa);
hold on;
% plot3(squareL1(1,4),squareL1(2,4),squareL1(3,4),'go'); hold on;
% plot3(squareL2(1,4),squareL2(2,4),squareL2(3,4),'go'); hold on;
% plot3(squareL3(1,4),squareL3(2,4),squareL3(3,4),'go'); hold on;
% plot3(squareL4(1,4),squareL4(2,4),squareL4(3,4),'go'); hold on;
goalXYZ = [5,15,5];
moveTo(goalXYZ,eStop);
uARM.teach



    function iseStop(eStop)
        stopped = any(eStop);
        while stopped ~= 0
            display('stopped')
            pause(2.0);
        end
    end





    function moveTo(goalXYZ,eStop)
        iseStop(eStop);  %check for safety
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
        isCollision(boxesB);
        steps = 15;
        %get angles
        qa= uARM.getpos();
        endEffector = uARM.fkine(qa);
        qNow = uARM.getpos(); %current joint angles
        %required L1 rotation to get end effector pose
        angleGoal = radtodeg(atan((goalXYZ(1,2)/goalXYZ(1,1))));
        angleCurrent =radtodeg(atan((endEffector(2,4)/endEffector(1,4))));
        angleTotal = angleGoal + angleCurrent;
        
        reach = (abs(goalXYZ(1,1))+abs(goalXYZ(1,2))); %of arm gives the amount of reach needed which then determines the
        %angle of L3 with regards to the height required.
        if reach > 30
            disp('Out of range')
        end
        if reach < 10
            disp('Out of range')
        end
        if goalXYZ(1,3) > 19
            disp('Out of range')
        end
        if goalXYZ(1,3) < 0
            disp('Out of range')
        end
        
        %end effector wanted orientation (L3 orientation)
        arg = rotz(angleTotal);%*rotx(16.3-((reach/33)*(goalXYZ(1,3)/25)*16.3)) %(16.3/((goalXYZ(1,3)/6.5)*(reach/10.5)))
        %now goalXYZ offset to account for L4 and end effector
        if angleTotal >=1
            offsetEE = [(goalXYZ(1,1)-(3.5*sin(abs(angleTotal)))),(goalXYZ(1,2)-(3.5*cos((angleTotal)))),(goalXYZ(1,3)+6)];
        else
            offsetEE = [(goalXYZ(1,1)-(3.5*cos(abs(angleTotal)))),(goalXYZ(1,2)-(3.5*sin((angleTotal)))), (goalXYZ(1,3)+6)];
        end
        %change goalXYZ to offsetEE when end effector is modeled in
        eEPoint = [arg(1,1),arg(1,2),arg(1,3),offsetEE(1,1); arg(2,1),arg(2,2),arg(2,3),offsetEE(1,2); arg(3,1),arg(3,2),arg(3,3),offsetEE(1,3);endEffector(4,1),endEffector(4,2),endEffector(4,3),endEffector(4,4);];
        goalPoint = [arg(1,1),arg(1,2),arg(1,3),goalXYZ(1,1); arg(2,1),arg(2,2),arg(2,3),goalXYZ(1,2); arg(3,1),arg(3,2),arg(3,3),goalXYZ(1,3);endEffector(4,1),endEffector(4,2),endEffector(4,3),endEffector(4,4);];
        
        qValues = uARM.ikcon(goalPoint); %goal joint values
        qValues = uARM.ikcon(eEPoint);  %hopefully same as goalPoint just offset to account for end effector
        qMatrix = jtraj(qNow,qValues,steps);
        eMatrix = jtraj(qNow,qValues,steps);
        while steps >= 1
            
            qCurrent= uARM.getpos();
            %create and move bounding boxes
            boxUpdate(boxesB);
            isCollision(boxesB);
            iseStop(eStop);
            %plot end effector with arm
            pos = uARM.fkine(eMatrix((16-steps),:));
            
            eeAngle = rotz(qCurrent(1,1));  %get angle current and rotation matrix to add to below
            EndEffectorPose =[eeAngle(1,1),eeAngle(1,2),eeAngle(1,3),pos(1,4);eeAngle(2,1),eeAngle(2,2),eeAngle(2,3),pos(2,4);eeAngle(3,1),eeAngle(3,2),eeAngle(3,3),pos(3,4);0,0,0,1;];
            updatedPoints =[EndEffectorPose * [EndEffectorVerts,ones(EndEffectorVertexCount,1)]']';
            EndEffector_h.Vertices = updatedPoints(:,1:3);
            uARM.plot(qMatrix((16-steps),:));
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

    function isCollision(boxesB)
        QUERYPOINTX = 0; 
        QUERYPOINTY = 0;
        QUERYPOINTZ = 0;
        plot3(QUERYPOINTX,QUERYPOINTY,QUERYPOINTZ,'go'); hold on;
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
        if (inpolygon(QUERYPOINTX,QUERYPOINTY,xvAX,yvAX)&&inpolygon(QUERYPOINTY,QUERYPOINTZ,xvAY,yvAY)&&inpolygon(QUERYPOINTX,QUERYPOINTZ,xvAZ,yvAZ));
             eStop(1,8) = 1;
        end
        %B box collision test
        if (inpolygon(QUERYPOINTX,QUERYPOINTY,xvBX,yvBX)&&inpolygon(QUERYPOINTY,QUERYPOINTZ,xvBY,yvBY)&&inpolygon(QUERYPOINTX,QUERYPOINTZ,xvBZ,yvBZ));
             eStop(1,8) = 1;
        end
        %C box collision test
        if (inpolygon(QUERYPOINTX,QUERYPOINTY,xvCX,yvCX)&&inpolygon(QUERYPOINTY,QUERYPOINTZ,xvCY,yvCY)&&inpolygon(QUERYPOINTX,QUERYPOINTZ,xvCZ,yvCZ));
             eStop(1,8) = 1;
        end
        %D box collision test
        if (inpolygon(QUERYPOINTX,QUERYPOINTY,xvDX,yvDX)&&inpolygon(QUERYPOINTY,QUERYPOINTZ,xvDY,yvDY)&&inpolygon(QUERYPOINTX,QUERYPOINTZ,xvDZ,yvDZ));
             eStop(1,8) = 1;
        end
        %E box collision test
        if (inpolygon(QUERYPOINTX,QUERYPOINTY,xvEX,yvEX)&&inpolygon(QUERYPOINTY,QUERYPOINTZ,xvEY,yvEY)&&inpolygon(QUERYPOINTX,QUERYPOINTZ,xvEZ,yvEZ));
             eStop(1,8) = 1;
        end
        iseStop(eStop);
    end
end