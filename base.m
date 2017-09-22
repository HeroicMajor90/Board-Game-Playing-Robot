function base()
clf;
clear all;
clc
goalXYZ= zeros(3,1);
eStop =0; 
%%%%%%%%%  Initialise myR1 robot
L1 = Link('d',0,'a',2,'alpha',pi/2,'offset',0,'qlim', [-pi/2,pi/2]);
L2 = Link('d',0,'a',15,'alpha',0,'offset',pi/1.5,'qlim', [(-2/3*pi),(1/3*pi)]);
L3 = Link('d',0,'a',16,'alpha',0,'offset',-pi/1.5,'qlim', [-pi/3,(2/3*pi)]);
L4 = Link('d',0,'a',6.95,'alpha',-pi/2,'offset',degtorad(-59.7),'qlim', [-pi/3,pi/3]);

uARM = SerialLink([L1 L2 L3 L4], 'name', 'uARM')
uARM.base = transl(0.5, 0, -2); %Base position uARM

q1 =0; q2 =0; q3 =0; q4 =0; %initialise q states
qa = [q1,q2,q3,q4];
uARM.plot(qa);
hold on;
goalXYZ = [18,10,7];
moveTo(goalXYZ,eStop,qa);
uARM.plot(qMatrix);hold on;
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
goalPoint = transl(goalXYZ)

qNow = uARM.ikcon(uARM.fkine(qa)) %current joint angles
qValues = uARM.ikcon(uARM.fkine(goalPoint)) %goal joint values
%get values for each link joint
%qL1 = %rotation of arm about the base
qL1 = qValues(1,1)
%qL2 = %rotation of side arm 
qL2 = qValues(1,2)
%qL3 = %rotation of top arm function of other arms
qL3 = 540-((210+radtodeg(qL2))+239.7) %angle of L2, plus angles of 5 sided shape
%qL4 = %rotation of end effector with 30.3 degree ofset so that it points normal to ground function of other arms
qL4 = -(radtodeg(qL2)+qL3) %effector at normal to ground, undo rotations of other arms
%Actual move part
qGoal = [qL1,qL2,degtorad(qL3),degtorad(qL4)]
qMatrix = jtraj(qNow,qGoal,steps)

end
function moveIncrement()
end
function isCollision()
end
end