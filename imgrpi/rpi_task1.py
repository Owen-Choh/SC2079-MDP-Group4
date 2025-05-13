import io
import json
import queue
import time
from multiprocessing import Process, Manager
from typing import Optional, List
import os
from picamera import PiCamera
import requests

from communication.android import AndroidLink, AndroidMessage
from communication.board import STMLink
from logger import prepare_logger
from settings import API_IP, API_PORT, OUTDOOR_BIG_TURN

SYMBOL_MAP = {
    "NA": "NA",
    "10": "Bullseye",
    "11": "One",
    "12": "Two",
    "13": "Three",
    "14": "Four",
    "15": "Five",
    "16": "Six",
    "17": "Seven",
    "18": "Eight",
    "19": "Nine",
    "20": "A",
    "21": "B",
    "22": "C",
    "23": "D",
    "24": "E",
    "25": "F",
    "26": "G",
    "27": "H",
    "28": "S",
    "29": "T",
    "30": "U",
    "31": "V",
    "32": "W",
    "33": "X",
    "34": "Y",
    "35": "Z",
    "36": "Up Arrow",
    "37": "Down Arrow",
    "38": "Right Arrow",
    "39": "Left Arrow",
    "40": "Stop"
}

class PiAction:
    """
    Represents an action that the Pi is responsible for:
    - Changing the robot's mode (manual/path)
    - Requesting a path from the API
    - Snapping an image and requesting the image-rec result from the API
    """

    def __init__(self, cat, value):
        self._cat = cat
        self._value = value

    @property
    def cat(self):
        return self._cat

    @property
    def value(self):
        return self._value
    

class RaspberryPi:
    def __init__(self):
        # prepare logger
        self.logger = prepare_logger()
        
        self.last_class_name = None
        self.last_snap_time = None
        # communication links
        self.android_link = AndroidLink()
        self.stm_link = STMLink()

        # for sharing information between child processes
        manager = Manager()
        self.stm_messages_queue=manager.Queue()
        
        self.path_ready_event = manager.Event()
        self.path_ready_event.clear()
        

        # 0: manual, 1: path (default: 1)
        self.robot_mode = manager.Value('i',1)

        # events
        self.android_dropped = manager.Event()  # set when the android link drops
        self.unpause = manager.Event()  # commands will be retrieved from commands queue when this event is set

        # movement lock, commands will only be sent to STM32 if this is released
        self.movement_lock = manager.Lock()

        # queues
        self.android_queue = manager.Queue()
        self.rpi_action_queue = manager.Queue()
        self.command_queue = manager.Queue()
        self.path_queue = manager.Queue()

        # define processes
        self.proc_recv_android = None
        self.proc_recv_stm32 = None
        self.proc_android_sender = None
        self.proc_command_follower = None
        self.proc_rpi_action = None

    def start(self):
        try:
            # establish bluetooth connection with Android
            self.android_link.connect()
             #self.android_link.send(AndroidMessage('info', 'Robot is ready!hehehehe'))
            self.android_queue.put(AndroidMessage('info', 'You are connected to the RPi!'))

            # establish connection with STM32
            self.stm_link.connect()
            
            

            # check api status
            self.check_api()

            # define processes
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_recv_stm32 = Process(target=self.recv_stm)
            self.proc_android_sender = Process(target=self.android_sender)
            self.proc_command_follower = Process(target=self.command_follower)
            self.proc_rpi_action = Process(target=self.rpi_action)

            # start processes
            self.proc_recv_android.start()
            self.proc_recv_stm32.start()
            self.proc_android_sender.start()
            self.proc_command_follower.start()
            self.proc_rpi_action.start()

            self.logger.info("Child Processes started")
            self.android_queue.put(AndroidMessage('info', 'Robot is ready!'))
            self.android_queue.put(AndroidMessage('mode', 'path' if self.robot_mode.value == 1 else 'manual'))

            # buzz STM32 (2 times)
            #self.stm_link.send("BR00")
            

            # reconnect handler to watch over android connection
            self.reconnect_android()

        except KeyboardInterrupt:
            self.stop()
    def some_condition_is_met(self) -> bool:
            """
            If bullseye => keep going (return False), else => stop (return True).
            """
            self.logger.debug(f"bullseye check: {self.last_class_name}")
            if (self.last_class_name == "Bullseye"):
                return True
            else:
                return False
    def stop(self):
        self.android_link.disconnect()
        self.stm_link.disconnect()
        self.logger.info("Program exited!")
    def repeat_movements_until_done(self):
        commands = ["B50|030a", "R50|050a", "L50|050a", "F50|050a", "r50|050a", "L50|040a", "B50|020a", "SNAP5_Ca"]
        self.logger.info("Starting repeated movement sequence...")

        while True:
            if self.some_condition_is_met():
                self.logger.info("Condition met before new loop => stopping.")
                break

            for cmd in commands:
                if self.some_condition_is_met():
                    self.logger.info("Condition met mid-sequence => stopping.")
                    return

                self.logger.debug(f"Queueing command: {cmd}")
                self.command_queue.put(cmd)
                time.sleep(7)

        self.logger.info("repeat_movements_until_done() finished.")
    def reconnect_android(self):
        self.logger.info("Reconnection handler is watching...")

        while True:
            # wait for android connection to drop
            self.android_dropped.wait()

            self.logger.error("Android link is down!")

            # buzz STM32 (3 times)
            #self.stm_link.send("ZZ03")

            # kill child processes
            self.logger.debug("Killing android child processes")
            self.proc_android_sender.kill()
            self.proc_recv_android.kill()

            # wait for the child processes to finish
            self.proc_android_sender.join()
            self.proc_recv_android.join()
            assert self.proc_android_sender.is_alive() is False
            assert self.proc_recv_android.is_alive() is False
            self.logger.debug("Android child processes killed")

            # clean up old sockets
            self.android_link.disconnect()

            # reconnect
            self.android_link.connect()

            # recreate android processes
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_android_sender = Process(target=self.android_sender)

            # start processes
            self.proc_recv_android.start()
            self.proc_android_sender.start()

            self.logger.info("Android child processes restarted")
            self.android_queue.put(AndroidMessage("info", "You are reconnected!"))
            self.android_queue.put(AndroidMessage('mode', 'path' if self.robot_mode.value == 1 else 'manual'))

            # buzz STM32 (2 times)
            #self.stm_link.send("BL00")

            self.android_dropped.clear()

    def recv_android(self) -> None:
        while True:
            msg_str: Optional[str] = None
            try:
                msg_str = self.android_link.recv()
            except OSError:
                self.android_dropped.set()
                self.logger.debug("Event set: Android connection dropped")

            # if an error occurred in recv()
            if msg_str is None:
                continue

            message: dict = json.loads(msg_str)

            # change mode command
            if message['cat'] == "mode":
                self.rpi_action_queue.put(PiAction(**message))
                self.logger.debug(f"Change mode PiAction added to queue: {message}")

            # manual movement commands
            elif message['cat'] == "manual":
                if self.robot_mode.value == 0:  # robot must be in manual mode
                    self.command_queue.put(message['value'])
                    self.logger.debug(f"Manual Movement added to command queue: {message['value']}")
                else:
                    self.android_queue.put(AndroidMessage("error", "Manual movement not allowed in Path mode."))
                    self.logger.warning("Manual movement not allowed in Path mode.")

            # set obstacles
            elif message['cat'] == "obstacles":
                if self.robot_mode.value == 1:  # robot must be in path mode
                    self.rpi_action_queue.put(PiAction(**message))
                    self.logger.debug(f"Set obstacles PiAction added to queue: {message}")
                else:
                    self.android_queue.put(AndroidMessage("error", "Robot must be in Path mode to set obstacles."))
                    self.logger.warning("Robot must be in Path mode to set obstacles.")

            # control commands
            elif message['cat'] == "control":
                if message['value'] == "start2":
                    self.unpause.set()
                    self.command_queue.put("Z0000000")
                    self.command_queue.put("F50|050a")
                    self.command_queue.put("SNAP8_c")
                    self.repeat_movements_until_done()
                elif message['value'] == "start":
                    # robot must be in path mode
                    if self.robot_mode.value !=1:
                        self.android_queue.put(AndroidMessage("Error", "robot must be in path mode"))
                        return
                    if not self.path_ready_event.is_set():
                        self.logger.warning("User tried to start but path is not ready")
                        self.android_queue.put(AndroidMessage("Error", "path is not ready yet please wait"))
                        return
                    if self.command_queue.empty():
                        self.logger.warning("The command queue is empty. did u generate the path?")
                        self.android_queue.put(AndroidMessage("Error", "Command queue empty, did u set obstacles?"))
                        return
                    self.unpause.set()
                    self.logger.info("Start commands received, starting robot on path")
                    self.android_queue.put(AndroidMessage("info", "starting robot on path"))
                    self.android_queue.put(AndroidMessage("status", "running"))
                                                            

    def recv_stm(self) -> None:
        self.logger.info("Started waiting for message from STM")
        while True:
            message: str = self.stm_link.recv()
            if "ACK" in message:
                #try:
                    #self.movement_lock.release()
                self.logger.debug("ACK received, message send to command follower.")
                self.stm_messages_queue.put(message)
                #except Exception as e:
                    #self.logger.debug("ACK received but lock was not acquired.")  
            else:
                self.logger.debug(f"Ignored unknown message from STM: {message}")
                #else:
                    #self.logger.warning(f"Ignored unknown message from STM: {message}")


    def android_sender(self) -> None:
        """
        Responsible for retrieving messages from the outgoing message queue and sending them over the Android Link
        """
        while True:
            # retrieve from queue
            try:
                message: AndroidMessage = self.android_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            # send it over the android link
            try:
                self.android_link.send(message)
            except OSError:
                self.android_dropped.set()
                self.logger.debug("Event set: Android dropped")


    def command_follower(self):
        """
        Continuously pulls commands off self.command_queue, tries sending them to the STM32,
        and waits for 'ACK' within a timeout. If no ACK arrives, we retry sending the same command
        up to `max_retries` times.
        """
        stm32_prefixes = ("F", "B", "L", "R", "l", "r", "Z", "p", "P", "o", "O")
        max_retries = 3  # how many times we'll re-send if we see no ACK
        ack_timeout = 7.0  # seconds to wait for an ACK each time
        previous_command = None
#         
#         queue_snapshot = []
#         while not self.command_queue.empty():
#             queue_snapshot.append(self.command_queue.get_nowait())
#             self.command_queue.append(queue_snapshot[-1])
#             
            
        #self.logger.debug(f"[command_follower] before dequeuing: {queue_snapshot}")

        while True:
            # 1) Block until the next command arrives
            command: str = self.command_queue.get()
            self.logger.debug(f"[command_follower] Got command: {command!r}")
            
#             if previous_command == command:
#                 self.logger.debug("Duplicate command [command_follower]")
#             previous_command = command

            # 2) Wait until unpause event is set (the 'start' signal)
            self.unpause.wait()

            # 3) Acquire the movement lock so we only handle one command at a time
            self.movement_lock.acquire()
            self.logger.debug("[command_follower] Movement lock acquired.")

            try:
                if command == "FIN":
                    self.unpause.clear()
        
                    self.logger.info("[command_follower] Commands queue finished.")
                    self.android_queue.put(
                        AndroidMessage("info", "Commands queue finished.")
                    )

                    self.android_queue.put(AndroidMessage("status", "finished"))
                    self.rpi_action_queue.put(PiAction(cat="stitch", value=""))


                elif command.startswith(stm32_prefixes):
                    # -------------------------------------------------------------
                    # Movement commands that require ACK from the STM32
                    # -------------------------------------------------------------
                    got_ack = False
                    retries = 0

                    while (not got_ack) and (retries < max_retries):
                        # Send the command once
                        self.stm_link.send(command)
                        self.logger.info(
                            f"[command_follower] Sent to STM32 (try #{retries+1}): {command}"
                        )

                        time.sleep(4)

                        try:
                            # Wait for an 'ACK' from self.stm_messages_queue with a timeout
                            msg = self.stm_messages_queue.get(timeout=ack_timeout)
                            if msg.startswith("ACK"):
                                self.logger.info(
                                    f"[command_follower] Received ACK for {command!r}."
                                )
                                got_ack = True
                            else:
                                self.logger.warning(
                                    f"[command_follower] Unexpected STM32 msg (not ACK): {msg!r}. Retrying..."
                                )
                        except queue.Empty:
                            # Did not get any message from STM within ack_timeout seconds
                            self.logger.warning(
                                f"[command_follower] No ACK received within {ack_timeout}s for {command!r}, retrying..."
                            )
                        finally:
                            retries += 1

                    if not got_ack:
                        # We exhausted max_retries
                        self.logger.error(
                            f"[command_follower] Failed to get ACK after {max_retries} tries for {command!r}."
                        )
                        # Optionally, tell Android or do something else:
                        self.android_queue.put(
                            AndroidMessage(
                                "error", f"No ACK after {max_retries} tries: {command}."
                            )
                        )

                elif command.startswith("WN"):
                    # No ACK needed, or up to you if you want to do the same pattern with ack
                    self.stm_link.send(command)
                    self.android_queue.put(AndroidMessage("status", "running"))
                    self.android_queue.put(
                        AndroidMessage("info", "Starting robot on fastest car!")
                    )

                elif command.startswith("SNAP"):
                    if len(command) > 4:
                        obstacle_id = command[4]
                    else:
                        obstacle_id = "NA"
                    
                    self.logger.info("[command_follower] Issuing SNAP command and waiting for image processing.")
                    
                    self.rpi_action_queue.put(PiAction(cat="snap", value=obstacle_id))
                    time.sleep(2)
                    next_command = None
                    if not self.command_queue.empty():
                        try:
                            next_command = self.command_queue.get_nowait()
                            #self.command_queue.put(next_command)
                        except queue.Empty:
                            pass
                    
                    if next_command == "FIN":
                        
                    # ✅ Wait until an image result is ready
                        snap_timeout = 5  # Adjust this based on expected processing time
                        start_time = time.time()
                    
                        while time.time() - start_time < snap_timeout:
                            if self.last_class_name is not None:  # Image was processed
                                self.logger.info("[command_follower] Image processing completed.")
                                break
                            time.sleep(1)  # Avoid excessive CPU usage

                        if self.last_class_name is None:
                            self.logger.warning("[command_follower] Timeout waiting for image processing.")

                    self.logger.info("[command_follower] Proceeding after SNAP.")

                    if next_command is not None:
                        self.command_queue.put(next_command)
                elif command.strip() == "MANSNAP":
                    # ...
                    self.logger.debug("[command_follower] MANSNAP received.")
                    self.rpi_action_queue.put(PiAction(cat="snap", value="8"))

                else:
                    # ...
                    raise Exception(f"[command_follower] Unknown command: {command!r}")
#                 if previous_command == command:
#                     time.sleep(0.1)

            finally:
                self.movement_lock.release()
                self.logger.debug("[command_follower] Movement lock released.")

    
    def rpi_action(self):
        while True:
            action: PiAction = self.rpi_action_queue.get()
            self.logger.debug(f"PiAction retrieved from queue: {action.cat} {action.value}")

            if action.cat == "mode":
                self.change_mode(action.value)
            elif action.cat == "obstacles":
                self.request_algo(action.value)
            elif action.cat == "snap":
                #command_follower function call this here
                self.snap_and_rec(obstacle_id_with_signal=action.value)
            #elif action.cat == "single-obstacle":
                #self.add_navigate_path()
            elif action.cat == "stitch":
                self.request_stitch()


 
 
                       
    def snap_and_rec(self, obstacle_id_with_signal: str) -> None:
        self.logger.info(f"Capturing image for obstacle id: {obstacle_id_with_signal}")
        self.android_queue.put(AndroidMessage("info", f"Capturing image for obstacle id: {obstacle_id_with_signal}"))

        url = f"http://{API_IP}:6000/process_frame"
        self.logger.debug(f"{url}")
        filename = f"{int(time.time())}_{obstacle_id_with_signal}.jpg"
        retry_count = 0
        self.last_class_name = None

        try:
            camera = PiCamera()
            camera.resolution = (640, 640)
        except Exception as e:
            self.logger.error(f"Failed to initialize camera: {str(e)}")
            return

        try:
            try:
                camera.capture(filename)
                self.logger.info(f"Image captured and saved as {filename}")
            except Exception as e:
                self.logger.error(f"Error capturing image: {str(e)}")
                return

            self.logger.debug("[snap_and_rec] Sending image to API")

            while retry_count < 3:
                try:
                    with open(filename, 'rb') as img_file:
                        files = {'file': (filename, img_file, 'image/jpeg')}
                        response = requests.post(url, files=files, timeout=10)

                    if response.status_code == 200:
                        results = response.json()
                        symbol_map_id = results.get('symbol_map_id', 'NA')
                        largest = results.get("largest_detection")

                        if largest:
                            class_name = largest.get("class_name", None)
                            self.last_class_name = class_name
                            self.logger.info(f"[snap_and_rec] Detected class_name: {class_name}")
                        else:
                            self.last_class_name = None
                            self.logger.info("[snap_and_rec] No detection found.")

                        # ✅ **Wait until the result is available before proceeding**
                        self.logger.info("[snap_and_rec] Image processing complete, sending data to Android")
                        results["obstacle_id"] = obstacle_id_with_signal
                        self.android_queue.put(AndroidMessage("image-rec", results))
                        return  # ✅ Return after success

                    else:
                        self.logger.error(f"[snap_and_rec] Error from API (status {response.status_code}). Retrying...")
                        retry_count += 1

                except requests.exceptions.RequestException as e:
                    self.logger.error(f"[snap_and_rec] Request failed: {e}")
                    retry_count += 1
                    time.sleep(1)  # Wait before retrying

            self.logger.error("[snap_and_rec] Maximum retries reached, image recognition failed.")

        finally:
            try:
                camera.close()
                self.logger.info("[snap_and_rec] Camera successfully closed.")
            except Exception as e:
                self.logger.error(f"[snap_and_rec] Failed to close the camera: {str(e)}")
            if self.last_class_name is None:
                self.last_class_name = "NA"
    # Calling the algo
    def request_algo(self, data, robot_x=1, robot_y=1, robot_dir=0, retrying=False):
        """
        Requests for a series of commands and the path from the Algo API.
        The received commands and path are then queued in the respective queues.
        data is expected to be a dict: { "obstacles": [ { "id": "...", "x": .., "y": .., "dir": "N" }, ... ] }
        """

        self.logger.info("Requesting path from Algo API...")
        self.android_queue.put(AndroidMessage("info", "Requesting path from Algo API..."))

        # 1) Convert your obstacles to the format required by the API
        #    The API expects "direction" in {"NORTH", "EAST", "SOUTH", "WEST"}
        #    If your Android sends "N"/"E"/"S"/"W", do a mapping:
        dir_map = {
            'N': 'NORTH',
            'E': 'EAST',
            'S': 'SOUTH',
            'W': 'WEST'
        }

        obstacles = data.get("obstacles", [])
        body = {"obstacles": []}

        for obs in obstacles:
            # Convert single-letter direction to "NORTH"/"SOUTH"/"EAST"/"WEST" if needed
            obs_dir = obs["dir"].upper()
            api_dir = dir_map.get(obs_dir, 'NORTH')  # default to NORTH if missing

            body["obstacles"].append({
                "id": obs["id"],
                "x": obs["x"],
                "y": obs["y"],
                "direction": api_dir
            })

        # 2) Call /generate_path
        generate_path_url = f"http://{API_IP}:{API_PORT}/generate_path"
        resp_gp = requests.post(generate_path_url, json=body, timeout=40)
        if resp_gp.status_code != 200:
            self.logger.error("Something went wrong when requesting /generate_path.")
            self.android_queue.put(AndroidMessage("error", "Algo API /generate_path call failed."))
            return

        gp_result = resp_gp.json()  # e.g. { "optimal_path": [...], "cost": 12, "visited_obstacle_order": [...] }

        # The returned path is in gp_result["optimal_path"],
        # which is a list of objects: e.g. [ { "x":1, "y":1, "direction":"NORTH" }, ... ]
        optimal_path = gp_result["optimal_path"]
       

        # 3) Call /commands (no request body, if your code doesn't require one)
        commands_url = f"http://{API_IP}:{API_PORT}/commands"
        resp_cmd = requests.post(commands_url)
        if resp_cmd.status_code != 200:
            self.logger.error("Something went wrong when requesting /commands.")
            self.android_queue.put(AndroidMessage("error", "Algo API /commands call failed."))
            return

        cmd_result = resp_cmd.json()  # e.g. { "commands": [...], "visited_obstacle_order": [...] }
        commands = cmd_result["commands"]
        self.logger.info(f"Command received from API: {commands}")
        
        # 4) Clear existing commands/paths, then load new ones
        self.clear_queues()

        # Put each command from /commands into the command queue
        for c in commands:
            self.command_queue.put(c)

        # Put each cell from the "optimal_path" into the path queue.
        # If your path-follower code expects a dictionary { x, y, direction },
        # we can do something like:
        for p in optimal_path[1:]:  # ignoring index 0 if it's the robot's starting position
            path_dict = {
                "x": p["x"],
                "y": p["y"],
                "direction": p["direction"]  # "NORTH"/"SOUTH"/"EAST"/"WEST"
            }
            self.path_queue.put(path_dict)

        self.path_ready_event.set()
        self.logger.info("Path is fully loaded, command queue is populated.")
        self.android_queue.put(AndroidMessage(
            "info", "Commands and path received from Algo API. Robot is ready to move."
        ))
        self.logger.info("Commands and path received from Algo API. Robot is ready to move.")
        self.logger.info(f"Sending optimal path to android: {optimal_path}")
        self.android_queue.put(AndroidMessage("COORDINATES", optimal_path))
        

        
    def change_mode(self, new_mode):
        # if robot already in correct mode
        if new_mode == "manual" and self.robot_mode.value == 0:
            self.android_queue.put(AndroidMessage('error', 'Robot already in Manual mode.'))
            self.logger.warning("Robot already in Manual mode.")
        elif new_mode == "path" and self.robot_mode.value == 1:
            self.android_queue.put(AndroidMessage('error', 'Robot already in Path mode.'))
            self.logger.warning("Robot already in Path mode.")
        else:
            # change robot mode
            self.robot_mode.value = 0 if new_mode == 'manual' else 1

            # clear command, path queues
            self.clear_queues()

            # set unpause event, so that robot can freely move
            if new_mode == "manual":
                self.unpause.set()
            else:
                self.unpause.clear()

            # release movement lock, if it was previously acquired
            #try:
                #self.movement_lock.release()
            #except Exception:
                #self.logger.warning("Tried to release a released lock!")

            # notify android
            self.android_queue.put(AndroidMessage('info', f'Robot is now in {new_mode.title()} mode.'))
            self.logger.info(f"Robot is now in {new_mode.title()} mode.")

            # buzz stm32 (1 time)
            #self.stm_link.send("ZZ01")
    def request_stitch(self):
        url = f"http://{API_IP}:6000/stitch_images"
        response = requests.get(url)

        # error encountered at the server, return early
        if response.status_code != 200:
            # notify android
            self.android_queue.put(AndroidMessage("error", "Something went wrong when requesting stitch from the API."))
            self.logger.error("Something went wrong when requesting stitch from the API.")
            return

        self.logger.info("Images stitched!")
        self.android_queue.put(AndroidMessage("info", "Images stitched!"))

    def clear_queues(self):
        while not self.command_queue.empty():
            self.command_queue.get()
        while not self.path_queue.empty():
            self.path_queue.get()

    def check_api(self) -> bool:
        url = f"http://{API_IP}:{API_PORT}/status"
        try:
            response = requests.get(url, timeout=5)
            if response.status_code == 200:
                self.logger.debug("API is up!")
                return True
        except ConnectionError:
            self.logger.warning("API Connection Error")
            return False
        except requests.Timeout:
            self.logger.warning("API Timeout")
            return False
        except Exception as e:
            self.logger.warning(f"API Exception: {e}")
            return False
        
if __name__ == "__main__":
    rpi = RaspberryPi()
    rpi.start()




   


