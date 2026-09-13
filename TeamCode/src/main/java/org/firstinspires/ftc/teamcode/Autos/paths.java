package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.intake;

@Autonomous(name = "Test AUTO - RED")
public class paths extends OpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Follower follower;
    private intake intake = new intake();
    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(55.9936, 19.2415, 270);
    private final Pose path1Control1 = poseFactory.of(55.9936, 19.8479, 0);
    private final Pose path1Segment1Start = poseFactory.of(55.9936, 19.2415, 90);
    private final Pose path1Segment1End = poseFactory.of(55.9936, 19.2415, 90);
    private final Pose path1Segment2Start = poseFactory.of(55.9936, 19.2415, 90);
    private final Pose path1Segment2End = poseFactory.of(55.9936, 19.2415, 270);
    private final Pose point2 = poseFactory.of(8.4079, 23.5486, -90);
    private final Pose point2Control1 = poseFactory.of(28.5686, 21.5407, 0);
    private final Pose point2Control2 = poseFactory.of(5.7521, 42.4993, 0);
    private final Pose point2Segment1Heading = poseFactory.of(8.4079, 23.5486, 270);
    private final Pose point3 = poseFactory.of(8.7907, 7.6743, 270);
    private final Pose point4 = poseFactory.of(57.8129, 116.4722, 90);
    private final Pose point4Control1 = poseFactory.of(56.6, 46.1265, 0);
    private final Pose point4Control2 = poseFactory.of(5.5364, 112.525, 0);
    private final Pose point4Segment1Start = poseFactory.of(57.8129, 116.4722, 270);
    private final Pose point4Segment1End = poseFactory.of(57.8129, 116.4722, 270);
    private final Pose point4Segment2Start = poseFactory.of(57.8129, 116.4722, 270);
    private final Pose point4Segment2End = poseFactory.of(57.8129, 116.4722, 90);

    public Path path1() {
        return curve(start, path1Control1, path1).heading(Interpolator.piecewise().until(0.0681, Interpolator.linear(path1Segment1Start, path1Segment1End)).until(1, Interpolator.linear(path1Segment2Start, path1Segment2End)));
    }

    public Path path2() {
        return curve(path1, point2Control1, point2Control2, point2).heading(Interpolator.piecewise().until(1, Interpolator.constant(point2Segment1Heading)));
    }

    public Path path3() {
        return line(point2, point3).constant(point3);
    }

    public Path path4() {
        return curve(point3, point4Control1, point4Control2, point4).heading(Interpolator.piecewise().until(0.3186, Interpolator.linear(point4Segment1Start, point4Segment1End)).until(1, Interpolator.linear(point4Segment2Start, point4Segment2End)));
    }

    public Command intakeToggle(){
        return intake.toggle();
    }

    private Command autoRoutine() {
        return sequential(
                sequential(follow(follower, path1())),
                sequential(follow(follower, path2())),
                intakeToggle(),
                sequential(follow(follower, path3())),
                intakeToggle(),
                sequential(follow(follower, path4()))
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);

        intake.init(hardwareMap);

        follower.setPose(start);

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

        intake.tick();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
    }

}