import json
import os
import socket
from typing import Optional
import bluetooth
from communication.link import Link
import time
# to install bluetooth
#pip install git+https://github.com/pybluez/pybluez.git#egg=pybluez

#Need to know the bluetooth device adapter in the RPI, Client side (Android) will have to input the (UUID) as well


class AndroidMessage:
    """
    Class for communicating with Android tablet over Bluetooth connection.
    """

    def __init__(self, cat: str, value: str):
        """
        Constructor for AndroidMessage.
        :param cat: Message category.
        :param value: Message value.
        """
        self._cat = cat
        self._value = value

    @property
    def cat(self):
        """
        Returns the message category.
        :return: String representation of the message category.
        """
        return self._cat

    @property
    def value(self):
        """
        Returns the message as a string.
        :return: String representation of the message.
        """
        return self._value
    
    @property
    def jsonify(self) -> str:
        """
        Returns the message as a JSON string.
        :return: JSON string representation of the message.
        """
        return json.dumps({'cat': self._cat, 'value': self._value})
    
class AndroidLink(Link):
    
    def __init__(self):
        """
        Initialize the Bluetooth connection.
        """
        super().__init__()
        self.client_sock = None
        self.server_sock = None

    def connect(self):
        """
        Connect to Andriod by Bluetooth
        """
            
        self.logger.info("Bluetooth connection started")
        try:
            # Set RPi to be discoverable
            os.system("sudo hciconfig hci0 piscan")
            os.system("sudo rfcomm bind 0 6C:2F:8A:38:0E:EE")
            time.sleep(2)

            # Initialize the server socket
            self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.server_sock.bind(("", bluetooth.PORT_ANY))
            self.server_sock.listen(1)

            # Get assigned port
            port = self.server_sock.getsockname()[1]
            uuid = '94f39d29-7d6d-437d-973b-fba39e49d4ee'

            # Advertise the service
            bluetooth.advertise_service(self.server_sock, "MDP-Group-RPi",
                                        service_id=uuid,
                                        service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                                        profiles=[bluetooth.SERIAL_PORT_PROFILE])

            self.logger.info(f"Awaiting Bluetooth connection on RFCOMM CHANNEL {port}")
            self.client_sock, client_info = self.server_sock.accept()
            self.logger.info(f"Accepted connection from: {client_info}")

        except Exception as e:
            self.logger.error(f"Error in Bluetooth link connection: {e}")
            self.cleanup()
    def disconnect(self):
        """Disconnect from Android Bluetooth connection and shutdown all the sockets established"""
        try:
            self.logger.debug("Disconnecting Bluetooth link")
            self.server_sock.shutdown(socket.SHUT_RDWR)
            self.client_sock.shutdown(socket.SHUT_RDWR)
            self.client_sock.close()
            self.server_sock.close()
            self.client_sock = None
            self.server_sock = None
            self.logger.info("Disconnected Bluetooth link")
        except Exception as e:
            self.logger.error(f"Failed to disconnect Bluetooth link: {e}")

    def send(self, message: AndroidMessage):
        """Send message to Android"""
        try:
            self.client_sock.send(f"{message.jsonify}\n".encode("utf-8"))
            self.logger.debug(f"Sent to Android: {message.jsonify}")
        except OSError as e:
            self.logger.error(f"Error sending message to Android: {e}")
            raise e

    def recv(self) -> Optional[str]:
        """Receive message from Android"""
        try:
            tmp = self.client_sock.recv(1024)
            self.logger.debug(tmp)
            message = tmp.strip().decode("utf-8")
            self.logger.debug(f"Received from Android: {message}")
            return message
        except OSError as e:  # connection broken, try to reconnect
            self.logger.error(f"Error receiving message from Android: {e}")
            raise e
