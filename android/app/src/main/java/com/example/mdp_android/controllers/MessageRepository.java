package com.example.mdp_android.controllers;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class MessageRepository {
    private static MessageRepository instance;
    private final List<String> messagesReceived = Collections.synchronizedList(new ArrayList<>());
    private final List<String> messagesSent = Collections.synchronizedList(new ArrayList<>());

    private MessageRepository() {}

    public static synchronized MessageRepository getInstance() {
        if (instance == null) {
            instance = new MessageRepository();
        }
        return instance;
    }

    public void addReceivedMessage(String message) {
        messagesReceived.add(0, message); // Add new messages at the beginning of the list
    }

    public void addSentMessage(String message) {
        messagesSent.add(0, message); // Add new messages at the beginning of the list
    }

    public List<String> getReceivedMessages() {
        return new ArrayList<>(messagesReceived); // Return a copy to avoid concurrent modification
    }

    public List<String> getSentMessages() {
        return new ArrayList<>(messagesSent); // Return a copy to avoid concurrent modification
    }
}

