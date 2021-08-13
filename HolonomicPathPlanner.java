package frc.libs;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Motion Of The Ocean, the Spiritual Successor to Shark Macro
 *
 * @author Varun Rajesh
 */
public class MotionOfTheOcean {

    private static Map<String, Runnable> usableCommands = new HashMap<>();

    /**
     * Add key-command pair to map for recording and executing
     *
     * @param commandName string identifier of command
     * @param commandRunnable runnable command
     */
    public static void addCommand(String commandName, Runnable commandRunnable) {
        usableCommands.put(commandName, commandRunnable);
    }

    public static class Recorder {
        private static ArrayList<State> recording = new ArrayList<>();
        private static ArrayList<String> currentCommands = new ArrayList<>();

        /**
         * Add command to list of ran commands during current iteration
         * 
         * @param command String identifier of command to be recorded
         */
        public static void recordCommand(String command) {
            currentCommands.add(command);
        }

        /**
         * Record state, pose and ran commands, of robot
         * 
         * @param poseSupplier Supplier to get pose of robot in an x, y, theta triplet
         */
        public static void recordState(Supplier<double[]> poseSupplier) {
            recording.add(new State(poseSupplier.get(), currentCommands));
            currentCommands = new ArrayList<>();
        }

        /**
         * Save recording to specified file name
         * 
         * @param fileName String of file name recording to be saved to
         */
        public static void writeOutRecording(String fileName) {
            try {
                FileWriter fileWriter = new FileWriter(fileName);
                for (State state : recording) {
                    fileWriter.write(state.toString() + "\n");
                }
                fileWriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        /**
         * Reset recorder
         * 
         * @param reset Runnable of the specified reset method
         */
        public static void resetRecorder(Runnable reset) {
            recording = new ArrayList<>();
            currentCommands = new ArrayList<>();
            reset.run();
        }
    }

    public static class Executor {
        private static Map<String, ArrayList<State>> loadedRecordings = new HashMap<>();

        private static ArrayList<State> selectedRecording;
        private static int executorIndex;

        /**
         * Load all specified recordings to a map for selection later
         * 
         * @param fileNames Array of strings of file names to be loaded
         */
        public static void loadRecordings(String... fileNames) {
            for (String fileName : fileNames) {
                ArrayList<State> currentLoadingRecording = new ArrayList<>();
                try {
                    BufferedReader fileReader = new BufferedReader(new FileReader(fileName));

                    String line;
                    while ((line = fileReader.readLine()) != null) {
                        double x, y, theta;
                        ArrayList<String> recordedCommands = new ArrayList<String>();

                        String[] split = line.split(",");
                        x = Double.parseDouble(split[0]);
                        y = Double.parseDouble(split[1]);
                        theta = Double.parseDouble(split[2]);

                        if (split.length == 4) {
                            String[] splitCommands = split[3].split("#");
                            recordedCommands.addAll(Arrays.asList(splitCommands));
                        }

                        currentLoadingRecording.add(new State(x, y, theta, recordedCommands));
                    }

                    loadedRecordings.put(fileName, currentLoadingRecording);
                    fileReader.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }

        /**
         * Select a recording from the loaded recording for execution
         * 
         * @param fileName String of file name of recording to be selected
         */
        public static void selectRecording(String fileName) {
            selectedRecording = loadedRecordings.get(fileName);
        }

        /**
         * Checks if the executor still has a states to be executed
         * 
         * @return boolean of aforementioned condition
         */
        public static boolean hasNextState() {
            return !(executorIndex >= selectedRecording.size() - 1);
        }

        /**
         * Get the state that the robot should go to
         * 
         * @return State that the robot should go to
         */
        public static State getState() {
            return selectedRecording.get(executorIndex);
        }

        /**
         * Executes recorded commands and calls supplied toPose method
         * 
         * @param toPose Runnable of the robot's toPose method
         */
        public static void executeRecording(Runnable toPose) {
            boolean runDrivetrain = true;

            for (String command : getState().getCommands()) {
                System.out.println(command);
                Runnable runnable;
                if(command.charAt(0) == '!') runDrivetrain = false;
                if ((runnable = usableCommands.get(command)) != null) {
                    runnable.run();
                }
            }
            if(runDrivetrain) toPose.run();
            if (hasNextState()) executorIndex++;
        }

        /**
         * Reset the executor
         * 
         * @param reset Runnable of the specified reset method
         */
        public static void resetExecutor(Runnable reset) {
            selectedRecording = new ArrayList<>();
            executorIndex = 0;
            reset.run();
        }
    }

    public static class State {
        private double x;
        private double y;
        private double theta;
        private ArrayList<String> commands;

        /**
         * Constructor
         * 
         * @param x x-coordinate of robot
         * @param y y-coordinate of robot
         * @param theta angle of robot
         * @param commands ArrayList of commands to be executed with pose
         */
        public State(double x, double y, double theta, ArrayList<String> commands) {
            this.x = x;
            this.y = y;
            this.theta = theta;
            this.commands = commands;
        }

        /**
         * Constructor
         * 
         * @param pose x, y, theta triplet of robot
         * @param commands ArrayList of commands to be executed with pose
         */
        public State(double[] pose, ArrayList<String> commands) {
            this(pose[0], pose[1], pose[2], commands);
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getTheta() {
            return theta;
        }

        public ArrayList<String> getCommands() {
            return commands;
        }

        public double[] getPose() {
            return new double[]{x, y, theta};
        }

        public String toString() {
            return x + "," + y + "," + theta + "," + String.join("#", commands);
        }
    }
}
