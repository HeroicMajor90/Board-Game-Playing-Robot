function base()
clf;
clear all;
clc
goalXYZ= zeros(3,1);
eStop =0; 
%%%%%%%%%  Initialise myR1 robot
L1 = Link('d',0,'a',2,'alpha',3*pi/2,'offset',0,'qlim', [-pi/2,pi/2]);
L2 = Link('d',0,'a',15,'alpha',0,'offset',-pi/1.5,'qlim', [(-1/3*pi),(1/3*pi)]);
L3 = Link('d',0,'a',16,'alpha',0,'offset',+pi/1.5,'qlim', [-pi/3,(2/3*pi)]);
L4 = Link('d',0,'a',6.95,'alpha',0,'offset',degtorad(59.7),'qlim', [-pi/2,pi/2]);

uARM = SerialLink([L1 L2 L3 L4], 'name', 'uARM')
uARM.base = transl(0.5, 0, -2); %Base position uARM

q1 =0; q2 =0; q3 =0; q4 =0; %initialise q states
qa = [q1,q2,q3,q4];
uARM.plot(qa);
hold on;
goalXYZ = [10,10,0];
moveTo(goalXYZ,eStop,qa);


uARM.teach
% qValues = uARM.fkine(qa)
%guess = uARM.ikcon(transl(goalXYZ),qa,[0,0,0,0,0,1])
%degguess = radtodeg(guess)
%End effector is 30.3 degrees clockwise from L4 position so that it is
%normal to the xy = 0 plane
function iseStop(eStop)
 while eStop ~= 0
    pause(2.0);
 end
end

function moveTo(goalXYZ,eStop,qa)
iseStop(eStop);  %check for safety
steps = 15;
%get angles


endEffector = uARM.fkine(qa)
qNow = uARM.ikcon(uARM.fkine(qa)) %current joint angles
goalPoint = [endEffector(1,1),endEffector(1,2),endEffector(1,3),goalXYZ(1,1); endEffector(2,1),endEffector(2,2),endEffector(2,3),goalXYZ(1,2); endEffector(3,1),endEffector(3,2),endEffector(3,3),goalXYZ(1,3);endEffector(4,1),endEffector(4,2),endEffector(4,3),endEffector(4,4);]
qValues = uARM.ikcon(goalPoint) %goal joint values
%get values for each link joint
%qL1 = %rotation of arm about the base
qL1 = qValues(1,1)
%qL2 = %rotation of side arm 
qL2 = qValues(1,2)
%qL3 = %rotation of top arm function of other arms
qL3 = qValues(1,3)%540-((210+radtodeg(qL2))+330) %angle of L2, plus angles of 5 sided shape
%qL4 = %rotation of end effector with 30.3 degree ofset so that it points normal to ground function of other arms
qL4 = -(radtodeg(qL2)+qL3)    %qValues(1,4)    %effector at normal to ground, undo rotations of other arms
%Actual move part
qGoal = [qL1,qL2,degtorad(qL3),degtorad(qL4)]
qMatrix = jtraj(qNow,qGoal,steps)
%tMatrix = ctraj(endEffector,goalPoint,15) %give point by point movement in transforms
while steps >= 1
            iseStop(eStop);
            uARM.plot(qMatrix((16-steps),:));
            steps = steps - 1;
end 
hold on;
end
function moveIncrement()
end
function isCollision()
end
end