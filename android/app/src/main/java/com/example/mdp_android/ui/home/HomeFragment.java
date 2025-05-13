package com.example.mdp_android.ui.home;

import android.annotation.SuppressLint;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.Observer;
import androidx.lifecycle.ViewModelProvider;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;
import androidx.recyclerview.widget.GridLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.example.mdp_android.R;
import com.example.mdp_android.controllers.BluetoothController;
import com.example.mdp_android.controllers.BluetoothControllerSingleton;
import com.example.mdp_android.controllers.DeviceSingleton;
import com.example.mdp_android.controllers.RpiController;
import com.example.mdp_android.databinding.FragmentHomeBinding;
import com.example.mdp_android.ui.grid.Map;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

public class HomeFragment extends Fragment {
    private static final String TAG = "HomeFragment";
    private static final int COLUMNS_OF_OBSTACLES = 2;
    private FragmentHomeBinding binding;
    private HomeViewModel homeViewModel;
    private String connectedDevice = "";
    DeviceSingleton deviceSingleton;

    private boolean pathGenerated = false;


    public Map map;
    private TextView robotStatus, targetStatus, bluetoothTextView, status;

    private RecyclerView obsListView;
    private static RecyclerAdapter obsItems;

    private Toast currentToast;

    private BluetoothController bluetoothController;

    private ToggleButton startBtn;



    @Override
    public View onCreateView(
            LayoutInflater inflater,
            ViewGroup container,
            Bundle savedInstanceState
    ) {
        homeViewModel = new ViewModelProvider(this).get(HomeViewModel.class);

        binding = FragmentHomeBinding.inflate(inflater, container, false);
        View root = binding.getRoot();

        root.setBackgroundResource(R.drawable.background); // Set the background image to adapt this pattern
        bluetoothTextView = binding.textViewBluetooth;

//        homeViewModel.setStatus("Not Ready");

        // register receiver for connected devices
        LocalBroadcastManager.getInstance(requireActivity()).registerReceiver(
                mNameReceiver,
                new IntentFilter("getConnectedDevice")
        );

        // register receiver for robot status
        LocalBroadcastManager.getInstance(requireActivity()).registerReceiver(
                mTextReceiver,
                new IntentFilter("getReceived")
        );

        // register receiver for initial robot position
        LocalBroadcastManager.getInstance(requireActivity()).registerReceiver(
                initialStatusReceiver,
                new IntentFilter("getStatus")
        );

        bluetoothController = BluetoothControllerSingleton.getInstance(new Handler());

        return root;
    }

    // Update status whenever connection changes
    private BroadcastReceiver mNameReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String deviceName = intent.getStringExtra("name");
            if (deviceName.equals("")) {
                connectedDevice = "";
                deviceSingleton.setDeviceName(connectedDevice);
                updateBluetoothStatus();
            } else {
                connectedDevice = deviceName;
                deviceSingleton.setDeviceName(connectedDevice);
                Log.d(TAG, "onReceive: -msg- " + connectedDevice);
                updateBluetoothStatus();
            }
        }
    };

    // update robot coordinates whenever new coordinates are received
    private BroadcastReceiver mTextReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            Log.d(TAG, "receiving messages");
            String status;
            String textReceived = intent.getStringExtra("received");
            log("TEXT RECEIVED IS: " + textReceived);
            JSONObject response = RpiController.readRpiMessages(textReceived);
            String messageType = RpiController.getRpiMessageType(textReceived);
            log("MESSAGE TYPE IS:" + messageType);

            if (textReceived.startsWith("ROBOT")) { // THIS STATEMENT IS TO HANDLE THE C10 SCENARIO ("ROBOT, <x>, <y>, <dir>")
                toast("C10 SCENARIO DETECTED");
                // Takes the string, format it into the desirable JSON format then pass it into updateRobotPosition
                String[] parts = textReceived.split(",");
                JSONObject responseC10 = new JSONObject();
                if (parts.length == 4) {
                    try {
                        int x = Integer.parseInt(parts[1].trim());
                        int y = Integer.parseInt(parts[2].trim());
                        String direction = parts[3].trim();

                        try {
                            responseC10.put("x", x);
                            responseC10.put("y", y);
                            responseC10.put("dir", direction);
                        } catch (JSONException e) {
                            e.printStackTrace();
                        }
                    } catch (NumberFormatException e) {
                        log("Invalid number format");
                    }
                }

                updateRobotPosition(responseC10);
                status = RpiController.getRobotStatus(responseC10);
                homeViewModel.setRobotStatus(status);
            } else if (messageType.equals("path")) { //ACCOUNT FOR THE CASE WHEN IT IS JUST PATH
                Log.d(TAG, "JUST PATH");
                pathGenerated = true;
                startBtn.setEnabled(true);
                startBtn.setChecked(false);


                try {
                    homeViewModel.setStatus("Looking for target");
                    ArrayList<ArrayList<Integer>> path = RpiController.getPath(response);
                    map.setExploredPath(path);
//                    map.animateRobotPath(path);
                    Log.d(TAG, "path: " + path.get(0));
                } catch (Exception e) {
                    log("empty path received: " + e);
                }
            } else if (messageType.equals("robot")) {
                Log.d(TAG, "JUST ROBOT");
                status = RpiController.getRobotStatus(response);
                homeViewModel.setRobotStatus(status);

                ArrayList<Map.Robot> robotPath = new ArrayList<>();

                try {
                    JSONArray jsonArray = response.getJSONArray("value");
                    for (int i = 0; i < jsonArray.length(); i++) {
                        JSONObject obj = jsonArray.getJSONObject(i);

                        // offset +1 cos api returns center of robot but our code is top right of robot
                        int x = obj.getInt("x") + 1;
                        int y = obj.getInt("y") + 1;
                        String direction = obj.getString("direction");

                        Map.Robot tempRobot = new Map.Robot(x, y);
                        // api returns the name of the direction instead of char
                        tempRobot.setDirection(Character.toString(direction.charAt(0)));

                        robotPath.add(tempRobot);
                    }
                } catch (JSONException e) {
                    Log.e(TAG, "JSON Parsing Error: " + e.getMessage());
                }

//                new Handler(Looper.getMainLooper()).postDelayed(() -> {
//                    log("sendMapToRpi: sending start cmd BEFORE animation");
//                    RpiController.sendToRpi(RpiController.getStartMsg());
//                }, 2000); // Adjust timeout here if needed

                new Handler(Looper.getMainLooper()).postDelayed(() -> {
                    log("sendMapToRpi: animating now");
                    map.animateRobotPath(robotPath);
                    }, 2000); // Adjust timeout here if needed
            } else if (messageType.equals("image")) {
                int count = textReceived.length();
                Log.e(TAG, "Number of characters: " + count);
                //CHECK IF IS COMBINATION OF PATH AND IMAGE
                if (count < 67) {
                    Log.d(TAG, "JUST BOTH IMAGE AND PATH");
                    //SETTLE THE FIRST RESPONSE
                    status = RpiController.getTargetStatus(response);
                    updateObstacle(response);
                    try {
                        Map.Obstacle o = map.getObstacle(Integer.parseInt(response.getString("obs_id")));
                        if (o != null) {
                            int x = o.getObsXCoor() - 1;
                            int y = o.getObsYCoor() - 1;
                            status = status + " at (" + x + ", " + y + ") facing " + o.getDirection();
                        } else {
                            status = "Invalid ID received";
                            toast("Invalid Obstacle ID received");
                        }
                    } catch (Exception e) {
                        log("Failed to parse JSON: " + e);
                    }
                    homeViewModel.setTargetStatus(status);
                    homeViewModel.setStatus("Target detected");

                    //SETTLE THE SECOND RESPONSE
                    // Find the index of the first occurrence of '{'
                    int firstIndex = textReceived.indexOf('{');

                    // Find the index of the second occurrence, starting search just after the firstIndex
                    int secondIndex = textReceived.indexOf('{', firstIndex + 1);

                    // Find the index of the second occurrence, starting search just after the firstIndex
                    int thirdIndex = textReceived.indexOf('{', secondIndex + 1);

                    // THIS IS THE SECOND JSON
                    String json2String = textReceived.substring(thirdIndex); // BUG: String IndexOutOfbounds exception
                    JSONObject response2 = RpiController.readSecondJSONMessages(json2String);
                    try {
                        homeViewModel.setStatus("Looking for target");
                        ArrayList<ArrayList<Integer>> path = RpiController.getPath(response2);
                        map.setExploredPath(path);
//                        map.animateRobotPath(path);
                        Log.d(TAG, "path: " + path.get(0));
                    } catch (Exception e) {
                        log("empty path received: " + e);
                    }
                } else {//IF NOT THEN IS JUST IMAGE
//                    Log.d(TAG, "JUST IMAGE");
//                    status = RpiController.getTargetStatus(response);
//                    updateObstacle(response);
//                    try {
//                        Map.Obstacle o = map.getObstacle(Integer.parseInt(response.getString("obs_id")));
//                        if (o != null) {
//                            int x = o.getObsXCoor() - 1;
//                            int y = o.getObsYCoor() - 1;
//                            status = status + " at (" + x + ", " + y + ") facing " + o.getDirection();
//                        } else {
//                            status = "Invalid ID received";
//                            toast("Invalid Obstacle ID received");
//                        }
//                    } catch (Exception e) {
//                        log("Failed to parse JSON: " + e);
//                    }
//                    homeViewModel.setTargetStatus(status);
//                    homeViewModel.setStatus("Target detected");
                    Log.d(TAG, "JUST IMAGE");

                    try {
                        JSONObject detection = response.getJSONObject("largest_detection");

                        // Extract new IDs correctly
                        int obstacleID = Integer.parseInt(response.getString("obstacle_id"));
                        String imageID = detection.getString("symbol_map_id");

                        // Update obstacle with correct IDs
                        JSONObject obstacleUpdate = new JSONObject();
                        obstacleUpdate.put("obs_id", obstacleID);
                        obstacleUpdate.put("img_id", imageID);

                        updateObstacle(obstacleUpdate);

                        // Retrieve obstacle from map for updating status
                        Map.Obstacle o = map.getObstacle(obstacleID);
                        if (o != null) {
                            int x = o.getObsXCoor() - 1;
                            int y = o.getObsYCoor() - 1;
                            status = obstacleID + " -> " + imageID +
                                    " at (" + x + ", " + y + ") facing " + o.getDirection();
                        } else {
                            status = "Invalid ID received";
                            toast("Invalid Obstacle ID received");
                        }
                    } catch (JSONException e) {
                        log("Failed to parse JSON: " + e);
                        toast("Failed to parse image recognition data");
                        status = "Parsing error";
                    } catch (Exception e) {
                        status = "Unknown error";
                        log("HomeFragment: Some exception occured when receiving: " + e.getMessage());
                    }

                    homeViewModel.setTargetStatus(status);
                    homeViewModel.setStatus("Target detected");
                }
            }
        }
    };

    private BroadcastReceiver initialStatusReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String textReceived = intent.getStringExtra("robot");
            homeViewModel.setRobotStatus(textReceived);

        }
    };


    @Override
    public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        // remove receiver
        LocalBroadcastManager.getInstance(requireActivity()).unregisterReceiver(mNameReceiver);
        LocalBroadcastManager.getInstance(requireActivity()).unregisterReceiver(mTextReceiver);
        LocalBroadcastManager.getInstance(requireActivity()).unregisterReceiver(initialStatusReceiver);
    }

    @Override
    public void onViewCreated(View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        // Top text boxes
        robotStatus = getView().findViewById(R.id.textView_robotStatus);
        targetStatus = getView().findViewById(R.id.textView_targetCoor);

        // Map control UI
        Button resetBtn;
        ToggleButton setRobot, removeRobot, setDirection;

        map = getView().findViewById(R.id.mapView);
        resetBtn = getView().findViewById(R.id.button_reset);
        // unused variable
        // ImageView obsData = getView().findViewById(R.id.imageView_obsData);
        obsListView = getView().findViewById(R.id.recyclerView_obsList);
        setRobot = getView().findViewById(R.id.button_startpoint);
        removeRobot = getView().findViewById(R.id.button_remove_robot);
        setDirection = getView().findViewById(R.id.button_setDirection);
        status = getView().findViewById(R.id.textView_Status);
        Button resetRobotBtn = getView().findViewById(R.id.reset_robot_btn);

        Button sendObstacleBtn = getView().findViewById(R.id.obstacle_send);

        // Bottom part of the UI
        ToggleButton setTaskType, manualBtn;
        setTaskType = getView().findViewById(R.id.button_taskType);
        startBtn = getView().findViewById(R.id.button_start);

        startBtn.setChecked(false);

        manualBtn = getView().findViewById(R.id.manualMode);

        // Robot control UI
        ImageButton upBtn, downBtn, backLeftBtn, backRightBtn, snapBtn, forwardLeftBtn, forwardRightBtn, cameraBtn;
        upBtn = getView().findViewById(R.id.imageButton_up);
        downBtn = getView().findViewById(R.id.imageButton_down);
        forwardLeftBtn = getView().findViewById(R.id.forwardLeft);
        forwardRightBtn = getView().findViewById(R.id.forwardRight);
        backLeftBtn = getView().findViewById(R.id.imageButton_left);
        backRightBtn = getView().findViewById(R.id.imageButton_right);
        cameraBtn = getView().findViewById(R.id.camera);

        // This button is for the hardcoded path in rpi

        createObstacleList();

        // set listeners to send task,
        // send obstacles
        sendObstacleBtn.setOnClickListener(v -> {
            // This typically sends obstacles:
            map.setStart(true);
            map.sendMapToRpi();
            toast("sent obstacles");
        });
        // robot and obstacles coordinates to rpi
        startBtn.setOnCheckedChangeListener((compoundButton, isChecked) -> {
            map.sendStartToRpi();
            toast("sent start cmd");

            // If path is NOT generated yet, first click => generate path
//            if (!pathGenerated) {
//                if (isChecked) {
//                    // The user just toggled ON => "Generate Path" phase
//                    toast("Generating path...");
//
//                    // This typically sends obstacles:
//                    map.sendMapToRpi();
//
//                    // Optionally disable the toggle until we hear back from RPi
//                    startBtn.setEnabled(false);
//                }
//                else {
//                    // If user toggles OFF again (rare), we can ignore
//                    // or revert to some default. But usually they won't.
//                }
//            }
//            else {
//                // pathGenerated == true => second click => "Start Robot"
//                if (isChecked) {
//                    toast("Starting robot...");
//                    RpiController.sendToRpi(RpiController.getStartMsg());
//                    // Optionally, you might set startBtn.setEnabled(false) to
//                    // prevent repeated toggles.
//                }
//                else {
//                    // If they un-toggle after start, do nothing or show a Toast.
//                }
//            }
        });


        manualBtn.setOnClickListener(v -> {
            if (manualBtn.isChecked()) {
                bluetoothController.write("{\"cat\":\"mode\",\"value\":\"manual\"}".getBytes(StandardCharsets.UTF_8));
                toast("Manual Mode on: ");
                manualBtn.setBackgroundResource(R.drawable.icons8_steering_wheel_48);
            } else {
                bluetoothController.write("{\"cat\":\"mode\",\"value\":\"path\"}".getBytes(StandardCharsets.UTF_8));
                toast("Path Mode On: ");
                manualBtn.setBackgroundResource(R.drawable.icons8_robot_24);
            }
        });

        resetBtn.setOnClickListener(v -> {
            toast("map is cleared");
            map.clearGrid();
            obsItems.setAllVisibility(true);
            map.setStart(false);
            robotStatus.setText("");
            targetStatus.setText("");
        });

        resetRobotBtn.setOnClickListener(v -> {
            toast("reset everything");
            ArrayList<String> commands = new ArrayList<>();
//            commands.add("SF050");
            //RpiController.sendToRpi(RpiController.getNavDetails(commands));
            bluetoothController.write(
                    "{\"cat\":\"control\",\"value\":\"reset\"}".getBytes(StandardCharsets.UTF_8)
            );
        });

        cameraBtn.setOnClickListener(v -> {
            if (!manualBtn.isChecked()) {
                toast("remember to set manual mode");
                return;
            }
            ArrayList<String> commands = new ArrayList<>();
            bluetoothController.write(
                    "{\"cat\":\"manual\",\"value\":\"MANSNAP\"}".getBytes(StandardCharsets.UTF_8)
            );        });

        upBtn.setOnClickListener(v -> {
            if (!manualBtn.isChecked()) {
                toast("remember to set manual mode");
                return;
            }
            toast("move forward");
            ArrayList<String> commands = new ArrayList<>();
//            commands.add("SF050");
            //RpiController.sendToRpi(RpiController.getNavDetails(commands));
            bluetoothController.write("{\"cat\":\"manual\",\"value\":\"F50|050a\"}".getBytes(StandardCharsets.UTF_8));
            map.moveRobot("N");
        });

        downBtn.setOnClickListener(v -> {
            if (!manualBtn.isChecked()) {
                toast("remember to set manual mode");
                return;
            }
            toast("move backwards");
            ArrayList<String> commands = new ArrayList<>();
//            commands.add("SB050");
//            RpiController.sendToRpi(RpiController.getNavDetails(commands));
            bluetoothController.write("{\"cat\":\"manual\",\"value\":\"B50|150a\"}".getBytes(StandardCharsets.UTF_8));
            map.moveRobot("S");
        });

        forwardLeftBtn.setOnClickListener(v -> {
            if (!manualBtn.isChecked()) {
                toast("remember to set manual mode");
                return;
            }
            toast("turn left");
            ArrayList<String> commands = new ArrayList<>();
//            commands.add("LF090");
//            RpiController.sendToRpi(RpiController.getNavDetails(commands));
            bluetoothController.write(
                    "{\"cat\":\"manual\",\"value\":\"L50|090a\"}".getBytes(StandardCharsets.UTF_8)
            );
            map.moveRobot("W");
        });
        forwardRightBtn.setOnClickListener(v -> {
            if (!manualBtn.isChecked()) {
                toast("remember to set manual mode");
                return;
            }
            toast("turn left");
            ArrayList<String> commands = new ArrayList<>();
//            commands.add("LF090");
//            RpiController.sendToRpi(RpiController.getNavDetails(commands));
            bluetoothController.write(
                    "{\"cat\":\"manual\",\"value\":\"R50|090a\"}".getBytes(StandardCharsets.UTF_8)
            );
            map.moveRobot("W");
        });

        backLeftBtn.setOnClickListener(v -> {
            if (!manualBtn.isChecked()) {
                toast("remember to set manual mode");
                return;
            }
            toast("turn left");
            ArrayList<String> commands = new ArrayList<>();
//            commands.add("LF090");
//            RpiController.sendToRpi(RpiController.getNavDetails(commands));
            bluetoothController.write(
                    "{\"cat\":\"manual\",\"value\":\"l50|090a\"}".getBytes(StandardCharsets.UTF_8)
            );
            map.moveRobot("W");
        });

        backRightBtn.setOnClickListener(v -> {
            if (!manualBtn.isChecked()) {
                toast("remember to set manual mode");
                return;
            }
            toast("turn right");
            ArrayList<String> commands = new ArrayList<>();
//            commands.add("RF090");
//            RpiController.sendToRpi(RpiController.getNavDetails(commands));
            bluetoothController.write(
                    "{\"cat\":\"manual\",\"value\":\"r50|090a\"}".getBytes(StandardCharsets.UTF_8)
            );
            map.moveRobot("E");
        });

        setRobot.setOnCheckedChangeListener((compoundButton, b) -> {
            String message;
            if (b) {
                message = "Select a cell to place robot";
                setRobot.setBackgroundResource(R.drawable.icons8_cross_30);
            } else {
                message = "Cancel";
                setRobot.setBackgroundResource(R.drawable.robot_south);
            }
            toast(message);
            map.setCanDrawRobot(b);
            if (removeRobot.isChecked()) removeRobot.setChecked(!b);
            if (setDirection.isChecked()) setDirection.setChecked(!b);
        });

        removeRobot.setOnCheckedChangeListener((compoundButton, b) -> {
            toast("robot removed");
            map.setRobotCoor(-1, -1, "R");
            if (setRobot.isChecked()) setRobot.setChecked(!b);
            if (removeRobot.isChecked()) removeRobot.setChecked(!b);
            if (setDirection.isChecked()) setDirection.setChecked(!b);
        });

        setDirection.setOnCheckedChangeListener((compoundButton, b) -> {
            String message;
            if (b) {
                message = "Select object to change direction";
                setDirection.setBackgroundResource(R.drawable.icons8_cross_30);
            } else {
                message = "cancel";
                setDirection.setBackgroundResource(R.drawable.icons8_rotate_right_50);
            }

            toast(message);
            map.setCanSetDirection(b);
            if (setRobot.isChecked()) setRobot.setChecked(!b);
            if (removeRobot.isChecked()) removeRobot.setChecked(!b);
        });

        setTaskType.setOnCheckedChangeListener((compoundButton, b) -> {
            toast("You can just press start");

//            String message;
//            if (b) {
//                message = "Task type: Fastest Car";
//            } else {
//                message = "Task type: Image Recognition";
//            }
//            toast(message);
//            // set task type in map
//            map.setTaskType(b);
        });

    }

    // repopulate the ui when user returns
    @Override
    public void onResume() {
        super.onResume();

        bluetoothTextView.setText(homeViewModel.getReceivedText().getValue());
        updateObstacleList();
        homeViewModel.getReceivedText().observe(getViewLifecycleOwner(), s -> bluetoothTextView.setText(s));

        robotStatus.setText(homeViewModel.getRobotStatus().getValue());
        homeViewModel.getRobotStatus().observe(getViewLifecycleOwner(), s -> robotStatus.setText(s));

        targetStatus.setText(homeViewModel.getTargetStatus().getValue());
        homeViewModel.getTargetStatus().observe(getViewLifecycleOwner(), s -> targetStatus.setText(s));

        status.setText(homeViewModel.getStatus().getValue());
        homeViewModel.getStatus().observe(getViewLifecycleOwner(), s -> status.setText(s));
    }


    public void createObstacleList() {
        obsItems = new RecyclerAdapter(new String[]{"1", "2", "3", "4", "5", "6", "7", "8"});
        obsListView.setAdapter(obsItems);
        RecyclerView.LayoutManager obstacleLayoutManager = new GridLayoutManager(getContext(), COLUMNS_OF_OBSTACLES) {
            @Override
            public boolean canScrollVertically() {
                return false;
            }
        };
        obsListView.setLayoutManager(obstacleLayoutManager);
    }

    public void updateBluetoothStatus() {
        log("updating bluetooth status in home fragment...");
        deviceSingleton = DeviceSingleton.getInstance();

        if (!deviceSingleton.getDeviceName().equals("")) {
            connectedDevice = deviceSingleton.getDeviceName();
            homeViewModel.setReceivedText(getContext().getString(
                    R.string.bluetooth_device_connected) + connectedDevice);

            homeViewModel.setStatus("Ready to start");

        } else {
            homeViewModel.setReceivedText(getContext().getString(
                    R.string.bluetooth_device_connected_not));
            homeViewModel.setStatus("Not ready");
        }
    }

    public void updateRobotPosition(JSONObject robot) {
        try {
            int x = Integer.parseInt(robot.getString("x"));
            int y = Integer.parseInt(robot.getString("y"));
            String d = robot.getString("dir");
            if (map.isWithinCanvasRegion(x + 1, y + 1)) {
                map.setRobotCoor(x + 1, y + 1, d);
            } else {
                toast("Invalid coordinates received");
            }

        } catch (Exception e) {
            log("Failed to parse JSON: " + e);
        }
    }

    /**
     * Sets an image on an obstacle on the map
     *
     * @param target json containing the obstacle id obs_id and image id img_id
     */
    public void updateObstacle(JSONObject target) {
        try {
            toast("Image detected!");
            log("Target is" + target);
            //JSONObject data = target.getJSONObject("data");
            int obstacleID = Integer.parseInt(target.getString("obs_id"));
            log("OBS ID IS " + obstacleID);
            String imageID = target.getString("img_id");
            log("new image is " + imageID);
            if (imageID.equals("")) {
            } else {
                map.setObsTargetID(obstacleID, imageID);
            }
        } catch (Exception e) {
            log("Failed to parse JSON: " + e);
        }
    }

    public static void modifyObstacleVisibility(int position, boolean visible) {
        obsItems.setItemVisibility(position, visible);
        Log.d(TAG, "set obstacle " + position + " to " + visible);
    }

    public void log(String message) {
        Log.d(TAG, message);
    }

    // cancels the current toast and show the next one
    public void toast(String message) {
        if (currentToast != null) currentToast.cancel();
        currentToast = Toast.makeText(binding.getRoot().getContext(), message, Toast.LENGTH_SHORT);
        currentToast.show();
    }

    /**
     * Repopulate the obstacle list based on the map state
     * used to redraw the recycler view when switching screens
     */
    @SuppressLint("NotifyDataSetChanged")
    public void updateObstacleList() {
        // make sure that the variables of the home screen are set
        if (map == null || obsItems == null) return;

        List<Integer> placedObstacleIds = map.getPlacedObstacleIds();

        for (int i = 0; i < obsItems.getItemCount(); i++) {
            boolean isVisible = !placedObstacleIds.contains(i + 1);
            obsItems.setItemVisibility(i, isVisible);
        }
        // makes the recycler view refresh
        obsItems.notifyDataSetChanged();
    }
}
