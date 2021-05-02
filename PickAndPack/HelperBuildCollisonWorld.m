function BuildCollisonWorld()
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
     bench = collisionBox(0.25, 0.35, 0.025);
    belt1 = collisionBox(0.65, 0.2, 0.025);
    belt2 = collisionBox(0.65, 0.2, 0.025);

    TBench = trvec2tform([0.2 0 0.1]);
    TBelt1 = trvec2tform([0 -0.3 0.1]);
    TBelt2 = trvec2tform([0 0.3 0.1]);

    bench.Pose = TBench;
    belt1.Pose = TBelt1;
    belt2.Pose = TBelt2;
    
    coordinator.World = {bench, belt1, belt2};
    
    obs1 = collisionSphere(0.065);
    Tobs1 = trvec2tform([0.2 0.19 0.2]);
    obs1.Pose = Tobs1;
    
    obs2 = collisionSphere(0.065);
    Tobs2 = trvec2tform([0.2 -0.19 0.2]);
    obs2.Pose = Tobs2;

    coordinator.Obstacles = {obs1, obs2};    

    % Add the parts, which are only used for visualization and
    % simulation. A separate tool ensures that when a part is
    % gripped, it is included in the collision detection stage of
    % the trajectory optimization workflow.
    box2 = collisionBox(0.03, 0.03, 0.05);
    box3 = collisionBox(0.03, 0.03, 0.05);
    box1 = collisionBox(0.03, 0.03, 0.05);

    % Move the parts into position
    TBox2 = trvec2tform([0.25 -0.07 0.13]);
    TBox3 = trvec2tform([0.25 0 0.13]);
    TBox1 = trvec2tform([0.2 -0.05 0.13]);

    box2.Pose = TBox2;
    box3.Pose = TBox3;
    box1.Pose = TBox1;

    % Set the part mesh and color
    part1.mesh = box2;
    part2.mesh = box3;
    part3.mesh = box1;

    part1.color = 'y';
    part2.color = 'y';
    part3.color = 'g';

    part1.centerPoint = tform2trvec(part1.mesh.Pose);
    part2.centerPoint = tform2trvec(part2.mesh.Pose);
    part3.centerPoint = tform2trvec(part3.mesh.Pose);

    part1.plot = [];
    part2.plot = [];
    part3.plot = [];

    %coordinator.Parts = {part1, part2, part3};

    % Visualize world and parts
    %visualizeWorld()
    %visualizeParts()

   % Trigger Stateflow chart Event
   %coordinator.FlowChart.worldBuilt;


    end
    
    

