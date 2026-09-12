package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;


import java.util.Collections;
import java.util.List;


@Autonomous(name = "Blue Side Auto Close 6",  preselectTeleOp = "MainTeleOp")
public class test extends OpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose poseStart = poseFactory.of(8, 8,90);
    private final Pose pose1 = poseFactory.of(56, 64, 90);
    private final Pose pose2 = poseFactory.of(56, 8, 90);
    private final Pose pose3 = poseFactory.of(56, 64, 0);
    private final Pose pose3dot8 = poseFactory.of(112, 64, 0);
    private final Pose pose5 = poseFactory.of(112, 136, 0);
    private final Pose pose6 = poseFactory.of(112, 112, 180);
    private final Pose pose7 = poseFactory.of(56, 80, 180);
    private final Pose pose8 = poseFactory.of(8, 8, 270);
    private Path pathStart() {return line(poseStart, pose1).linear(poseStart, pose1);}
    private Path path1() {
        return line(pose1, pose2).linear(pose1, pose2);
    }

    private Path path2() {
        return line(pose2, pose3).linear(pose2, pose3);
    }

    private Path path3() {
        return line(pose3, pose3dot8).linear(pose3, pose3dot8);
    }

    private Path path3dot8() {
        return line(pose3dot8, pose5).linear(pose3dot8, pose5);
    }
    private Path path5() {
        return line(pose5, pose6).linear(pose5, pose6);
    }

    private Path path6(){
        return line(pose6, pose7).linear(pose6, pose7);
    }

    private Path path7() {
        return line(pose7, pose8).linear(pose7, pose8);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, pathStart()),
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path3dot8()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);

        follower.setPose(poseStart);

        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();

        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
    }

}


