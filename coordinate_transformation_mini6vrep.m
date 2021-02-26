
%---使用matlab与vrep联合仿真：在matlab中发送关节控制指令，实现vrep中机械臂对目标的抓取
disp('Program started');
%---机械臂数值解方法的逆解函数自由度掩码
MASK = [1 1 1 1 1 1];
%---关节角度反馈值初始化
currentJoints = zeros(1,6);
%---使用机器人工具箱The Robotics Toolbox for MATLAB (RTB)
%---建立DH模型
%           theta    d       a     alpha     offset
 L1 = Link([    0,0.1128,      0,    0],'modified');%0.11448
 L2 = Link([-pi/2,      0,0.02969,-pi/2],'modified');
 L3 = Link([    0,      0,  0.1076,    0],'modified');
 L4 = Link([    0,0.12867,0.02038,-pi/2],'modified');%0.12867
 L5 = Link([    0,      0,      0, pi/2],'modified');
 L6 = Link([    0,-0.03028,      0, pi/2],'modified');%0.03028

%L2=Link([0, 0  ,  0.115, 0 ],'standard');
%L3=Link([0, 0  ,  0.025, -pi/2 ],'standard');
%L4=Link([0, 0.128, 0 ,pi/2  ],'standard');
%L5=Link([0, 0  ,  0 ,-pi/2  ],'standard');
%L6=Link([0, 0  ,  0 ,0  ],'standard');

s_robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'DOF-6');
%---加载vrep远程库
vrep=remApi('remoteApi');
%---先关闭所以vrep通信
vrep.simxFinish(-1); 
%---定义vrep中各关节句柄名称
JointNames={'Joint1','Joint2','Joint3','Joint4','Joint5','Joint6'};
%---返回值有效性判断
res2=1;
res3=1;
res4=1;
res5=1;
res7=1;
res8=1;
%---建立与vrep的通信
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
%---判断通信是否成功
if (clientID>-1)   
    disp('Connected to remote API server'); 
    %---启动vrep的场景仿真
    res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait);
    %---读取各个关节句柄值
     for i = 1:6
         [res, sixJoints(i)] = vrep.simxGetObjectHandle(clientID, ...
             JointNames{i}, vrep.simx_opmode_oneshot_wait);
     end

    %---获得vrep中相应名称的对象句柄值
    %---'target_matlab'为抓取目标对象
    [res,handle_target] = vrep.simxGetObjectHandle(clientID,'target_matlab',vrep.simx_opmode_blocking);
    [res,handle_base_o] = vrep.simxGetObjectHandle(clientID,'Base_o',vrep.simx_opmode_blocking);
   % [res,handle_base] = vrep.simxGetObjectHandle(clientID,'base_frame',vrep.simx_opmode_blocking);
   % [res,handle_table] = vrep.simxGetObjectHandle(clientID,'workshop_frame',vrep.simx_opmode_blocking);
    %---锟叫讹拷matlab锟斤拷vrep锟斤拷锟斤拷锟斤拷锟角凤拷锟斤拷效 
    if(vrep.simxGetConnectionId(clientID) ~= -1) % while v-rep connection is still active
        while (res2==1||res3==1)
            [res2,T_targetPosition]=vrep.simxGetObjectPosition(clientID,handle_target,handle_base_o,vrep.simx_opmode_streaming);
            [res3,T_targetOrientation]=vrep.simxGetObjectOrientation(clientID,handle_target,handle_base_o,vrep.simx_opmode_streaming);
        end
        T_targetPosition
        T_targetOrientation
         %---转换为双精度浮点数
        T_targetOrientation=double(T_targetOrientation);
        %---转换为T矩阵中的位置向量
        target2baseposition=transl(T_targetPosition);
        %---转换为T矩阵中的姿态
        target2baseOrientation=trotx(T_targetOrientation(1))*troty(T_targetOrientation(2))*trotz(T_targetOrientation(3));%看来获得的原始姿态是用RPY表示的！ xyz绕旋转轴欧拉角
        %---得到工件坐标系在基座标系的位姿T
        target2basePos=target2baseposition*target2baseOrientation%把只有位置的矩阵和只有姿态的矩阵拼起来！！！
       
       %---读取vrep中四个关节当前的角度值
        for i = 1:6
            [returnCode,currentJoints(i)] = vrep.simxGetJointPosition(clientID, sixJoints(i),...
                    vrep.simx_opmode_oneshot_wait);
            currentJoints(i) = rad2deg(currentJoints(i));
        end
        currentJoints
        
%         g11 =round( target2basePos (1,1),3);%加round元整数据到小数点后三位，否则，有的应该为0的数据显示为很小的正数
%         g12 =round( target2basePos (1,2),3);
%         g13 =round( target2basePos (1,3),3);
%         g14 =round( target2basePos (1,4),3);
%         g21 =round( target2basePos (2,1),3);
%         g22 =round( target2basePos (2,2),3);
%         g23 =round( target2basePos (2,3),3);%加round元整数据到小数点后三位，否则，有的应该为0的数据显示为很小的正数
%         g24 =round( target2basePos (2,4),3);
%         g31 =round( target2basePos (3,1),3);
%         g32 =round( target2basePos (3,2),3);
%         g33 =round( target2basePos (3,3),3);
%         g34 =round( target2basePos (3,4),3);
%         g41 =round( target2basePos (4,1),3);
%         g42 =round( target2basePos (4,2),3);
%         g43 =round( target2basePos (4,3),3);
%         g44 =round( target2basePos (4,4),3);
%         
%         target2basePos_2 = [g11,g12,g13,g14;
%                             g21,g22,g23,g24;
%                             g31,g32,g33,g34;
%                             g41,g42,g43,g44]
        
        position_cmd_joints=s_robot.ikine(target2basePos,'rlimit',500)
        %---锟斤拷锟斤拷锟截节角讹拷指锟斤拷锟斤拷莘锟斤拷锟斤拷锟絭rep锟斤拷锟斤拷锟?
        vrep.simxPauseCommunication(clientID,1); 
        for i = 1:6
          % res = vrep.simxSetJointPosition(clientID, sixJoints(i),position_cmd_joints(i),vrep.simx_opmode_oneshot);
          res = vrep.simxSetJointTargetPosition(clientID, sixJoints(i),position_cmd_joints(i),vrep.simx_opmode_oneshot);
        end
        vrep.simxPauseCommunication(clientID,0);
        %---锟斤拷锟斤拷锟斤拷锟?
 
    end
    pause(8);
    %---停止vrep锟斤拷锟斤拷锟斤拷锟?
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
   
end
