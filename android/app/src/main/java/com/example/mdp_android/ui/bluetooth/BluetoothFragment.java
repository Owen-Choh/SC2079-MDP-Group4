package com.example.mdp_android.ui.bluetooth;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.app.ActivityCompat;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.Observer;
import androidx.lifecycle.ViewModelProvider;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import com.example.mdp_android.R;
import com.example.mdp_android.controllers.BluetoothController;
import com.example.mdp_android.controllers.BluetoothControllerSingleton;
import com.example.mdp_android.controllers.DeviceSingleton;
import com.example.mdp_android.databinding.FragmentBluetoothBinding;

import java.util.Set;

public class BluetoothFragment extends Fragment {
    private static final String TAG = "BluetoothFragment";

    private BluetoothViewModel bluetoothViewModel;
    private FragmentBluetoothBinding binding;

    // UI elements
    private ListView pairedDevicesListView, availableDevicesListView ;
    private ArrayAdapter<String> pairedDevicesAdapter, availableDevicesAdapter;
    private Button scanBtn;
    private ProgressBar progressAvail;
    private TextView bluetoothStatusTextView;

    // Bluetooth-related
    private BluetoothAdapter bluetoothAdapter;
    private BluetoothController bluetoothController =
            BluetoothControllerSingleton.getInstance(null);

    // Keep track of the currently connected device name
    private String connectedDevice = "";

    // Singleton to store global device info
    private DeviceSingleton deviceSingleton;

    //region Fragment Lifecycle
    @Override
    public View onCreateView(
            LayoutInflater inflater,
            ViewGroup container,
            Bundle savedInstanceState
    ) {
        bluetoothViewModel = new ViewModelProvider(this).get(BluetoothViewModel.class);
        binding = FragmentBluetoothBinding.inflate(inflater, container, false);
        View root = binding.getRoot();

        initViews(root);
        registerReceivers();
        setupBluetoothAdapter();
        setupListeners();

        return root;
    }

    @Override
    public void onResume() {
        super.onResume();
        bluetoothStatusTextView.setText(bluetoothViewModel.getDevice().getValue());

        // Observe changes from the ViewModel
        bluetoothViewModel.getDevice().observe(getViewLifecycleOwner(), new Observer<String>() {
            @Override
            public void onChanged(@Nullable String s) {
                bluetoothStatusTextView.setText(s);
            }
        });
    }

    //region Initialization
    private void initViews(View root) {
        pairedDevicesListView = root.findViewById(R.id.pairdDevices);
        availableDevicesListView = root.findViewById(R.id.availableDevices);

        scanBtn = root.findViewById(R.id.scanBtn);
        progressAvail = root.findViewById(R.id.progressAvailable);
        bluetoothStatusTextView = root.findViewById(R.id.textView_bluetooth);

        pairedDevicesAdapter = new ArrayAdapter<>(root.getContext(), R.layout.device_item);
        availableDevicesAdapter = new ArrayAdapter<>(root.getContext(), R.layout.device_item);

        pairedDevicesListView.setAdapter(pairedDevicesAdapter);
        availableDevicesListView.setAdapter(availableDevicesAdapter);

        // Initialize device singleton
        deviceSingleton = DeviceSingleton.getInstance();
    }

    private void setupListeners() {
        pairedDevicesListView.setOnItemClickListener(handleDeviceClicked);
        availableDevicesListView.setOnItemClickListener(handleDeviceClicked);

        scanBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                updateAvailableDevices();
            }
        });
    }

    private void setupBluetoothAdapter() {
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (bluetoothAdapter == null) {
            toast("Bluetooth not supported", Toast.LENGTH_SHORT);
        } else {
            // Check if Bluetooth is enabled
            if (!bluetoothAdapter.isEnabled()) {
                startActivity(new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE));
            } else {
                // If enabled, update both lists
                updatePairedDevices();
                updateAvailableDevices();
            }
        }
    }
    //endregion

    //region UI & Helper Methods
    public void toast(String message, int duration) {
        Toast.makeText(binding.getRoot().getContext(), message, duration).show();
    }

    /**
     * Update the Paired Devices list.
     */
    public void updatePairedDevices() {
        pairedDevicesAdapter.clear();
        checkBluetoothPermission();

        Set<BluetoothDevice> pairedDevices = bluetoothAdapter.getBondedDevices();
        if (pairedDevices.size() > 0) {
            for (BluetoothDevice d : pairedDevices) {
                pairedDevicesAdapter.add(d.getName() + "\n" + d.getAddress());
            }
        }
    }

    /**
     * Update the Available Devices list by starting discovery.
     */
    public void updateAvailableDevices() {
        availableDevicesAdapter.clear();
        progressAvail.setVisibility(View.VISIBLE);

        toast("Scanning for devices...", Toast.LENGTH_SHORT);

        checkBluetoothPermission();

        // if already discovering devices, stop
        if (bluetoothAdapter.isDiscovering()) {
            bluetoothAdapter.cancelDiscovery();
        }
        bluetoothAdapter.startDiscovery();
    }

    /**
     * Pass Bluetooth status to other fragments via LocalBroadcast.
     */
    private void sendBluetoothStatus(String msg) {
        updateDeviceName();
        Log.d(TAG, "Sending device name: " + msg);
        Intent intent = new Intent("getConnectedDevice");
        intent.putExtra("name", msg);
        LocalBroadcastManager.getInstance(binding.getRoot().getContext()).sendBroadcast(intent);
    }

    /**
     * Pass received text to messages fragment via LocalBroadcast.
     */
    private void sendReceived(String msg) {
        Log.d(TAG, "Received msg: " + msg);
        Intent intent = new Intent("getReceived");
        intent.putExtra("received", msg);
        LocalBroadcastManager.getInstance(binding.getRoot().getContext()).sendBroadcast(intent);
    }

    /**
     * Ensure `connectedDevice` is in sync with `deviceSingleton`.
     */
    private void updateDeviceName() {
        if (!deviceSingleton.getDeviceName().isEmpty()) {
            connectedDevice = deviceSingleton.getDeviceName();
        }
    }

    //endregion

    //region Device Connection
    private final AdapterView.OnItemClickListener handleDeviceClicked =
            (adapterView, view, i, l) -> {
                String info = ((TextView) view).getText().toString();
                // MAC address is last 17 chars
                String address = info.substring(info.length() - 17);
                String currentDevice = info.substring(0, info.length() - address.length());

                deviceSingleton.setDeviceName(currentDevice.trim());
                connectDevice(address, currentDevice.trim());
            };

    public void connectDevice(String address, String deviceName) {
        BluetoothDevice bDevice = bluetoothAdapter.getRemoteDevice(address);
        bluetoothController.setHandler(mHandler);
        bluetoothController.connect(bDevice);
        connectedDevice = deviceName;
    }
    //endregion

    //region Handler
    public final Handler mHandler = new Handler(new Handler.Callback() {
        @Override
        public boolean handleMessage(@NonNull Message message) {
            switch (message.what) {
                case BluetoothController.MessageConstants.MESSAGE_STATE_CHANGE:
                    handle_stateChange(message);
                    break;

                case BluetoothController.MessageConstants.MESSAGE_WRITE:
                    Log.d(TAG, "Handler Log: MESSAGE_WRITE");
                    break;

                case BluetoothController.MessageConstants.MESSAGE_READ:
                    Log.d(TAG, "Handler Log: MESSAGE_READ");
                    byte[] readBuf = (byte[]) message.obj;
                    // Construct string from valid bytes in the buffer
                    String readMessage = new String(readBuf, 0, message.arg1);
                    // Send message to messages fragment
                    sendReceived(readMessage);
                    Log.d(TAG, "Read Message: " + readMessage);
                    break;

                case BluetoothController.MessageConstants.MESSAGE_DEVICE_NAME:
                    Log.d(TAG, "Handler Log: MESSAGE_DEVICE_NAME");
                    // Save the connected device's name
                    connectedDevice = message.getData().getString("device_name");
                    if (connectedDevice != null) {
                        deviceSingleton.setDeviceName(connectedDevice);
                    }
                    break;

                case BluetoothController.MessageConstants.MESSAGE_TOAST:
                    Log.d(TAG, "Handler Log: MESSAGE_TOAST");
                    String error = message.getData().getString("toast");
                    toast(error, Toast.LENGTH_SHORT);
                    break;

                case BluetoothController.MessageConstants.MESSAGE_PICTURE:
                    Log.d(TAG, "Handler Log: MESSAGE_PICTURE");
                    break;
            }
            return false;
        }
    });

    private void handle_stateChange(@NonNull Message message){
        switch (message.arg1) {
            case BluetoothController.StateConstants.STATE_NONE:
                Log.d(TAG +" Handler Log: ", "STATE_NONE");
                // Set empty string since no device is connected currently
                connectedDevice = "";
                deviceSingleton.setDeviceName(connectedDevice);
                if (binding.getRoot().getContext() != null) {
                    bluetoothViewModel.setDevice(binding.getRoot().getContext().getString(R.string.bluetooth_device_connected_not));
                    // Send string to fragments that no devices connected currently
                    sendBluetoothStatus(connectedDevice);
                }
                break;
            case BluetoothController.StateConstants.STATE_LISTEN:
                Log.d(TAG + " Handler Log: ", "STATE_LISTEN");
                // Set empty string since no device is connected currently
                connectedDevice = "";
                deviceSingleton.setDeviceName(connectedDevice);
                if (binding.getRoot().getContext() != null) {
                    bluetoothViewModel.setDevice(binding.getRoot().getContext().getString(R.string.bluetooth_device_connected_not));
                    // Send string to fragments that no devices connected currently
                    sendBluetoothStatus(connectedDevice);
                }

                break;
            case BluetoothController.StateConstants.STATE_CONNECTING:
                Log.d(TAG+" Handler Log: ", "STATE_CONNECTING");
                // Set empty string since no device is connected currently
                toast("Connecting...Please wait", Toast.LENGTH_SHORT);
                connectedDevice = "";
                deviceSingleton.setDeviceName(connectedDevice);
                if (binding.getRoot().getContext() != null) {
                    bluetoothViewModel.setDevice(binding.getRoot().getContext().getString(R.string.bluetooth_device_connected_not));
                    // Send string to fragments that no devices connected currently
                    sendBluetoothStatus(connectedDevice);
                }

                break;
            case BluetoothController.StateConstants.STATE_CONNECTED:
                Log.d(TAG+" Handler Log: ", "STATE_CONNECTED");
                toast("connected to: "+connectedDevice, Toast.LENGTH_SHORT);
                if (binding.getRoot().getContext() != null) {
                    Log.d(TAG, "update bluetooth status");
                    bluetoothViewModel.setDevice(binding.getRoot().getContext().getString(R.string.bluetooth_device_connected)+connectedDevice);
                    // Send device name to other fragments
                    sendBluetoothStatus(connectedDevice);
                }
                break;
            case BluetoothController.StateConstants.STATE_DISCONNECTED:
                Log.d("Handler Log: ", "STATE_DISCONNECTED");
                Log.d(TAG, "Connection lost, attempting for reconnection...");
                break;
        }
    }

    //endregion

    //region Broadcast Receiver
    private BroadcastReceiver bluetoothBroadcastReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            BluetoothDevice bDevice;
            switch (action) {
                case BluetoothDevice.ACTION_FOUND:
                    bDevice = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                    String newDevice = bDevice.getName() + "\n" + bDevice.getAddress();
                    if (!checkNewDeviceExist(newDevice) && bDevice.getName() != null && bDevice.getBondState() != BluetoothDevice.BOND_BONDED) {
                        availableDevicesAdapter.add(newDevice);
                    }
                    break;
                case BluetoothAdapter.ACTION_DISCOVERY_FINISHED:
                    progressAvail.setVisibility(View.GONE);
                    Log.d(TAG, "scan complete");
                    break;
                case BluetoothDevice.ACTION_BOND_STATE_CHANGED:
                    Log.d(TAG, "bReceiver: ACTION_BOND_STATE_CHANGED");
                    bDevice = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                    if (bDevice.getBondState() == BluetoothDevice.BOND_BONDED) {
                        updatePairedDevices();
                        updateAvailableDevices();
                    }
                    break;
                case BluetoothDevice.ACTION_ACL_DISCONNECTED:
                    Log.d(TAG, "bReceiver: ACTION_ACL_DISCONNECTED");
                    break;
            }
        }
    };

    /**
     * Check if a newly found device already exists in the list.
     */
    private boolean checkNewDeviceExist(String newDevice) {
        for (int i = 0; i < availableDevicesAdapter.getCount(); i++) {
            if (newDevice.equals(availableDevicesAdapter.getItem(i))) {
                return true;
            }
        }
        return false;
    }
    //endregion

    //region Permission
    public void registerReceivers() {
        IntentFilter filter = new IntentFilter();
        filter.addAction(BluetoothDevice.ACTION_FOUND);
        filter.addAction(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
        filter.addAction(BluetoothDevice.ACTION_ACL_DISCONNECTED);
        filter.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
        binding.getRoot().getContext().registerReceiver(bluetoothBroadcastReceiver, filter);
    }

    /**
     * Check for required Bluetooth Permissions at runtime.
     */
    private void checkBluetoothPermission() {
        int fineLoc = requireContext().checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION);
        int coarseLoc = requireContext().checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION);
        int btConnect = requireContext().checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT);
        int btScan   = requireContext().checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN);

        if (fineLoc + coarseLoc + btConnect + btScan != 0) {
            ActivityCompat.requestPermissions(
                    requireActivity(),
                    new String[]{
                            Manifest.permission.ACCESS_FINE_LOCATION,
                            Manifest.permission.ACCESS_COARSE_LOCATION,
                            Manifest.permission.BLUETOOTH_CONNECT,
                            Manifest.permission.BLUETOOTH_SCAN
                    },
                    1001
            );
        }
    }
    //endregion
}
