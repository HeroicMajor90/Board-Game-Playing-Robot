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
squareL1 = uARM.base %gives location of sqaure start points will need to move into collision detect function
squareL2 = uARM.A([1], qa)
squareL3 = uARM.A([1,2], qa)
squareL4 = uARM.A([1,2,3], qa)
squareEndE = uARM.fkine(qa)
uARM.plot(qa);
hold on;
% plot3(squareL1(1,4),squareL1(2,4),squareL1(3,4),'go'); hold on;
% plot3(squareL2(1,4),squareL2(2,4),squareL2(3,4),'go'); hold on;
% plot3(squareL3(1,4),squareL3(2,4),squareL3(3,4),'go'); hold on;
% plot3(squareL4(1,4),squareL4(2,4),squareL4(3,4),'go'); hold on;
goalXYZ = [12,0,10];
moveTo(goalXYZ,eStop,qa);
uARM.teach



function iseStop(eStop)
stopped = any(eStop);
 while stopped ~= 0
    pause(2.0);
 end
end


function [baseSquare, baseInnerSquare] = squareBounds(point,boxSize)
baseSquare= [point(1,4)-boxSize,point(2,4)-boxSize,point(3,4); point(1,4)-boxSize,point(2,4)+boxSize,point(3,4);point(1,4)+boxSize,point(2,4)+boxSize,point(3,4);point(1,4)-boxSize,point(2,4)-boxSize,point(3,4);]
baseInnerSquare = [point(1,4)-(boxSize/2),point(2,4)-(boxSize/2),point(3,4); point(1,4)-(boxSize/2),point(2,4)+(boxSize/2),point(3,4);point(1,4)+(boxSize/2),point(2,4)+(boxSize/2),point(3,4);point(1,4)-(boxSize/2),point(2,4)-(boxSize/2),point(3,4);]
end


function moveTo(goalXYZ,eStop,qa)
iseStop(eStop);  %check for safety
%create bounding boxes

%move bounding boxes to locations

%check bounding boxes

steps = 15;
%get angles
endEffector = uARM.fkine(qa)
qNow = uARM.ikcon(uARM.fkine(qa)) %current joint angles
%required L1 rotation to get end effector pose
angleGoal = radtodeg(atan((goalXYZ(1,2)/goalXYZ(1,1))))
angleCurrent =radtodeg(atan((endEffector(2,4)/endEffector(1,4))))
angleTotal = angleGoal + angleCurrent

reach = (abs(goalXYZ(1,1))+abs(goalXYZ(1,2))) %of arm gives the amount of reach needed which then determines the
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
arg = rotz(angleTotal)%*rotx(16.3-((reach/33)*(goalXYZ(1,3)/25)*16.3)) %(16.3/((goalXYZ(1,3)/6.5)*(reach/10.5)))
%now goalXYZ offset to account for L4 and end effector
if angleTotal >=1
    offsetEE = [(goalXYZ(1,1)-(3.5*sin(abs(angleTotal)))),(goalXYZ(1,2)-(3.5*cos((angleTotal)))),(goalXYZ(1,3)+6)]
else
    offsetEE = [(goalXYZ(1,1)-(3.5*cos(abs(angleTotal)))),(goalXYZ(1,2)-(3.5*sin((angleTotal)))), (goalXYZ(1,3)+6)]
end
%change goalXYZ to offsetEE when end effector is modeled in
eEPoint = [arg(1,1),arg(1,2),arg(1,3),offsetEE(1,1); arg(2,1),arg(2,2),arg(2,3),offsetEE(1,2); arg(3,1),arg(3,2),arg(3,3),offsetEE(1,3);endEffector(4,1),endEffector(4,2),endEffector(4,3),endEffector(4,4);]
goalPoint = [arg(1,1),arg(1,2),arg(1,3),goalXYZ(1,1); arg(2,1),arg(2,2),arg(2,3),goalXYZ(1,2); arg(3,1),arg(3,2),arg(3,3),goalXYZ(1,3);endEffector(4,1),endEffector(4,2),endEffector(4,3),endEffector(4,4);]

qValues = uARM.ikcon(goalPoint) %goal joint values
qValues = uARM.ikcon(eEPoint)  %hopefully same as goalPoint just offset to account for end effector
qMatrix = jtraj(qNow,qValues,steps)
eMatrix = jtraj(qNow,qValues,steps)
while steps >= 1
    iseStop(eStop);
    uARM.plot(qMatrix((16-steps),:));
    %plot end effector with arm
    pos = uARM.fkine(eMatrix((16-steps),:));
    if angleTotal >= 0
        EndEffectorPose =transl(pos(1,4),pos(2,4),pos(3,4));
    else
        EndEffectorPose = transl(pos(1,4),pos(2,4),pos(3,4));
    end
    updatedPoints =[EndEffectorPose * [EndEffectorVerts,ones(EndEffectorVertexCount,1)]']';
    EndEffector_h.Vertices = updatedPoints(:,1:3);
    steps = steps - 1;
end
hold on;
end
function isCollision()
end
end