package com.example.mdp_android.ui.messages;

import android.content.BroadcastReceiver;
import android.content.ClipData;
import android.content.ClipboardManager;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.database.DataSetObserver;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ListView;
import android.widget.Toast;

import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import com.example.mdp_android.R;
import com.example.mdp_android.controllers.BluetoothController;
import com.example.mdp_android.controllers.BluetoothControllerSingleton;
import com.example.mdp_android.controllers.DeviceSingleton;
import com.example.mdp_android.controllers.MessageRepository;
import com.example.mdp_android.databinding.FragmentMessagesBinding;
import com.example.mdp_android.controllers.RpiController;

import org.json.JSONObject;

import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

public class MessagesFragment extends Fragment {
    private static final String TAG = "MessagesFragment";

    private FragmentMessagesBinding binding;
    private MessagesViewModel messagesViewModel;

    // UI components with more descriptive names
    private Button sendButton;
    private ImageButton copyReceivedButton, copySentButton;
    private ImageButton clearReceivedButton, clearSentButton;
    private EditText messageEditText;
    private ListView sentMessagesListView;
    private ListView receivedMessagesListView;

    // Use descriptive adapter names
    private static ArrayAdapter<String> sentMessagesAdapter;
    private static ArrayAdapter<String> receivedMessagesAdapter;

    private BluetoothController bluetoothController;
    private DeviceSingleton deviceSingleton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        messagesViewModel = new ViewModelProvider(this).get(MessagesViewModel.class);
        binding = FragmentMessagesBinding.inflate(inflater, container, false);
        View root = binding.getRoot();

        // Initialize UI components via view binding
        sentMessagesListView = binding.listViewSent;
        receivedMessagesListView = binding.listviewReceived;
        messageEditText = binding.editTextSendMessage;
        sendButton = binding.sendBtn;
        copyReceivedButton = binding.imageButtonCopyReceived;
        clearReceivedButton = binding.imageButtonClearReceived;
        copySentButton = binding.imageButtonCopySent;
        clearSentButton = binding.imageButtonClearSent;

        deviceSingleton = DeviceSingleton.getInstance();

        // Initialize adapters if not already created
        if (sentMessagesAdapter == null || receivedMessagesAdapter == null) {
            sentMessagesAdapter = new ArrayAdapter<>(requireContext(), R.layout.message_item);
            receivedMessagesAdapter = new ArrayAdapter<>(requireContext(), R.layout.message_item);
        }

        sentMessagesListView.setAdapter(sentMessagesAdapter);
        sentMessagesAdapter.registerDataSetObserver(new DataSetObserver() {
            @Override
            public void onChanged() {
                super.onChanged();
                sentMessagesListView.setSelection(sentMessagesAdapter.getCount() - 1);
            }
        });

        receivedMessagesListView.setAdapter(receivedMessagesAdapter);
        receivedMessagesAdapter.registerDataSetObserver(new DataSetObserver() {
            @Override
            public void onChanged() {
                super.onChanged();
                receivedMessagesListView.setSelection(receivedMessagesAdapter.getCount() - 1);
            }
        });

        // register receiver for receiving messages from connected device
        LocalBroadcastManager.getInstance(requireActivity()).registerReceiver(
                messageReceiver, new IntentFilter("getReceived"));

        // Set up the send button click listener
        sendButton.setOnClickListener(view -> {
            String message = messageEditText.getText().toString();
            if (deviceSingleton.getDeviceName().isEmpty()) {
                showToast("Bluetooth not connected to any device");
            } else if (message.isEmpty()) {
                showToast("Please enter a message");
            } else {
                sendMessage(message);
                showToast("Message sent: " + message);
                messageEditText.setText("");
                appendSentMessage(message);
            }
        });

        // Set up clear and copy buttons for received and sent messages
        clearReceivedButton.setOnClickListener(view -> {
            receivedMessagesAdapter.clear();
            showToast("Messages cleared");
        });

        clearSentButton.setOnClickListener(view -> {
            sentMessagesAdapter.clear();
            showToast("Messages cleared");
        });

        copyReceivedButton.setOnClickListener(view -> copyMessagesToClipboard(receivedMessagesAdapter, "received_messages"));
        copySentButton.setOnClickListener(view -> copyMessagesToClipboard(sentMessagesAdapter, "sent_messages"));

        // Initialize the BluetoothController singleton
        bluetoothController = BluetoothControllerSingleton.getInstance(new Handler());

        return root;
    }

    @Override
    public void onResume() {
        super.onResume();

        // Call your method to update the UI with messages when the fragment resumes
        updateReceivedMessages();
        updateSentMessages();
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        // Unregister the local broadcast receiver
        LocalBroadcastManager.getInstance(requireActivity()).unregisterReceiver(messageReceiver);
    }

    // Broadcast receiver to handle incoming messages
    private BroadcastReceiver messageReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            Log.d(TAG, "receiving messages");
            String textReceived = intent.getStringExtra("received"); // LEFT OFF HERE: 23 Feb
            JSONObject response = RpiController.readRpiMessages(textReceived);
            if (response != null) {
                String responseString = response.toString();
                appendReceivedMessage(textReceived);
            } else {
                appendReceivedMessage(textReceived);
            }

        }
    };

    // Returns the current time as a formatted string
    public static String getCurrentTime() {
        SimpleDateFormat formatter = new SimpleDateFormat("dd/MM/yyyy HH:mm:ss");
        return formatter.format(new Date());
    }

    // Append a new sent message to the adapter and update the view
    public static void appendSentMessage(String message) {
        sentMessagesAdapter.insert(getCurrentTime() + ": " + message, 0);
        sentMessagesAdapter.notifyDataSetChanged();
    }

    // Append a new received message to the adapter and update the view
    public static void appendReceivedMessage(String message) {
        receivedMessagesAdapter.insert(getCurrentTime() + ": " + message, 0);
        receivedMessagesAdapter.notifyDataSetChanged();
    }

    // Sends a message through the BluetoothController
    private void sendMessage(String message) {
        bluetoothController.write(message.getBytes(StandardCharsets.UTF_8));
    }

    // Update received messages from the MessageRepository
    private void updateReceivedMessages() {
        List<String> messages = MessageRepository.getInstance().getReceivedMessages();
        if (receivedMessagesAdapter != null) {
            receivedMessagesAdapter.clear();
            receivedMessagesAdapter.addAll(messages);
            receivedMessagesAdapter.notifyDataSetChanged();
        }
    }

    // Update sent messages from the MessageRepository
    private void updateSentMessages() {
        List<String> messages = MessageRepository.getInstance().getSentMessages();
        if (sentMessagesAdapter != null) {
            sentMessagesAdapter.clear();
            sentMessagesAdapter.addAll(messages);
            sentMessagesAdapter.notifyDataSetChanged();
        }
    }

    // Copy all messages from the given adapter to the clipboard
    private void copyMessagesToClipboard(ArrayAdapter<String> adapter, String label) {
        // Get the ClipboardManager service by calling getSystemService
        ClipboardManager clipboard = (ClipboardManager) getActivity().getSystemService(Context.CLIPBOARD_SERVICE);

        // Get the ClipboardManager service by calling getSystemService
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < adapter.getCount(); i++) {
            builder.append(adapter.getItem(i)).append("\n");
        }

        // Convert the StringBuilder to a String
        String allMessages = builder.toString().trim();

        // Create a ClipData with the text to be copied
        ClipData clip = ClipData.newPlainText(label, allMessages);

        // Set the ClipData as the primary clip on the clipboard
        clipboard.setPrimaryClip(clip);
        showToast("Copied to clipboard!");
    }

    // A helper method for displaying Toast messages
    private void showToast(String message) {
        Toast.makeText(getContext(), message, Toast.LENGTH_SHORT).show();
    }
}
