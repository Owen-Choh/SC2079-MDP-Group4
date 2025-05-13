from typing import Optional
import serial
from communication.link import Link
from settings import SERIAL_PORT, BAUD_RATE


#Need to get the serial port and the baud rate(set in the device)
# To know the serial port: Run python -m serial.tools.list_ports

class STMLink(Link):
    def __init__(self):
        """
        Constructor for STMLink.
        """
        super().__init__()
        self.serial_link = None

    def connect(self):
        """Connect to STM32 using serial UART connection, given the serial port and the baud rate"""
        self.serial_link = serial.Serial(SERIAL_PORT, BAUD_RATE)
        self.logger.info("Connected to STM32")

    def disconnect(self):
        """Disconnect from STM32 by closing the serial link that was opened during connect()"""
        self.serial_link.close()
        self.serial_link = None
        self.logger.info("Disconnected from STM32")

    def send(self, message: str) -> None:
        """Send a message to STM32, utf-8 encoded 

        Args:
            message (str): message to send
        """
        self.serial_link.write(f"{message}".encode())
        self.logger.debug(f"Sent to STM32: {message}")

    def recv(self) -> Optional[str]:
        """Receive a message from STM32, utf-8 decoded

        Returns:
            Optional[str]: message received
        """
        message = self.serial_link.readline().decode()
        self.logger.debug(f"Received from STM32: {message}")
        return message
