package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class QuestNav {
        NetworkTable quest = NetworkTableInstance.getDefault().getTable("questnav");

        IntegerSubscriber miso = quest.getIntegerTopic("miso").subscribe(0);
        IntegerPublisher mosi = quest.getIntegerTopic("mosi").publish();

        DoubleSubscriber timestamp = quest.getDoubleTopic("timestamp").subscribe(0.0f);
        IntegerSubscriber frames = quest.getIntegerTopic("frameCount").subscribe(0);
        DoubleSubscriber battery = quest.getDoubleTopic("device/batteryPercent").subscribe(0.0f);

        FloatArraySubscriber position = quest.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
        FloatArraySubscriber rotation = quest.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f});

        final DoubleSubscriber request = quest.getDoubleTopic("heartbeat/quest_to_robot").subscribe(0.0);
        final DoublePublisher response = quest.getDoubleTopic("heartbeat/robot_to_quest").publish();
        double lastProcessed = 0;

        Pose2d resetPose = new Pose2d();

        public boolean connected() {
                return ((Timer.getFPGATimestamp() - battery.getLastChange()) / 1000) < 250;
        }

        public boolean initialized() {
                return Utilities.isValidPose(resetPose);
        }

        public void heartbeat() {
                double requestId = request.get();

                if (requestId > 0 && requestId != lastProcessed) {
                        response.set(requestId);
                        lastProcessed = requestId;
                }
        }

        public double getTime() {
                return timestamp.getAtomic().serverTime;
        }

        public double getBattery() {
                return battery.get();
        }

        public int getFrames() {
                return (int)frames.get();
        }

        Translation2d getTranslation() {
                float[] questnavPosition = position.get();
                return new Translation2d(questnavPosition[2], -questnavPosition[0]);
        }

        Rotation2d getRotation() {
                float[] rotation = this.rotation.get();
                double yaw = (rotation[1] - Constants.Quest.rotationOffset) % 360;

                if (yaw < 0) yaw += 360;
                return new Rotation2d(yaw);
        }

        public Pose2d getPose() {
                return new Pose2d(
                        getTranslation().minus(Constants.Quest.positionOffset).minus(resetPose.getTranslation()),
                        getRotation()
                );
        }

        public void resetPose(Pose2d pose) {
                resetPose = pose;

                if (miso.get() != 99) {
                        mosi.set(1);
                }
        }
}
