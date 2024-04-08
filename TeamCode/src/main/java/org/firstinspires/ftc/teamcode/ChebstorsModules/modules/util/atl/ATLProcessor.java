package org.firstinspires.ftc.teamcode.ChebstorsModules.modules.util.atl;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import dev.narlyx.ftc.tweetybird.TweetyBirdProcessor;

/**
 * Processor for odometry via AprilTags
 */
public class ATLProcessor {

    /**
     * Reference to the current LinearOpMode
     */
    protected LinearOpMode opMode;

    /**
     * Reference to the current AprilTagProcessor
     */
    protected AprilTagProcessor aprilTagProcessor;

    /**
     * Reference to the current TweetyBirdProcessor
     */
    protected TweetyBirdProcessor tweetyBirdProcessor;

    private ATLWorker workerThread;

    //public FtcDashboard ftcDashboard = FtcDashboard.getInstance();

    public void scan() {
        workerThread.scan();
    }

    public void turnOnAutoRun() {
        workerThread.auto = true;
        workerThread.start();
    }

    public void turnOffAutoRun() {
        workerThread.auto = false;
    }

    /**
     * Constructor
     * @param builder The builder method MUST be used to start the class
     */
    public ATLProcessor(Builder builder) {
        //Pulling builder configurations
        this.opMode = builder.opMode;
        this.aprilTagProcessor = builder.aprilTagProcessor;
        this.tweetyBirdProcessor = builder.tweetyBirdProcessor;

        //Ensure all required imports have been set
        if ( this.opMode==null || this.aprilTagProcessor==null || this.tweetyBirdProcessor==null) {
            //return;
        }

        workerThread = new ATLWorker(this);

    }

    /**
     * Used to create/start this class
     */
    public static class Builder {
        //Variables
        private LinearOpMode opMode = null;
        public Builder setOpMode(LinearOpMode opMode) {
            this.opMode = opMode;
            return this;
        }

        private AprilTagProcessor aprilTagProcessor = null;
        public Builder setAprilTagProcessor(AprilTagProcessor aprilTagProcessor) {
            this.aprilTagProcessor = aprilTagProcessor;
            return this;
        }

        private TweetyBirdProcessor tweetyBirdProcessor  = null;
        public Builder setTweetyBirdProcessor(TweetyBirdProcessor tweetyBirdProcessor) {
            this.tweetyBirdProcessor = tweetyBirdProcessor;
            return this;
        }


        public ATLProcessor build() {
            return new ATLProcessor(this);
        }
    }
}