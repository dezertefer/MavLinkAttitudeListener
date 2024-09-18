# Reconnection loop
while True:
    try:
        # Open WebSocket connection
        ws = websocket.WebSocket()
        ws.connect(ws_url)
        print(f"Connected to WebSocket server at {ws_url}")

        while True:
            try:
                # Receive the MAVLink message
                message = master.recv_match(blocking=True)

                if message is None:
                    continue

                message = message.to_dict()

                if message['mavpackettype'] == 'ATTITUDE':
                    roll_rad = message['roll']
                    pitch_rad = message['pitch']
                    yaw_rad = message['yaw']

                    # Apply any settings: reverse, swap, fixed yaw, etc.
                    roll_deg = round(limit_angle(math.degrees(roll_rad)), 3)
                    pitch_deg = round(limit_angle(math.degrees(pitch_rad)), 3)
                    yaw_deg = round(math.degrees(yaw_rad), 3)

                    # Reverse options
                    if settings["reverse_roll"]:
                        roll_deg = -roll_deg
                    if settings["reverse_pitch"]:
                        pitch_deg = -pitch_deg
                    if settings["reverse_yaw"]:
                        yaw_deg = -yaw_deg

                    # Fix yaw if applicable
                    if settings["fixed_yaw_angle"] is not None:
                        yaw_deg = settings["fixed_yaw_angle"]

                    # Swap pitch and roll if enabled
                    if settings["swap_pitch_roll"]:
                        roll_deg, pitch_deg = pitch_deg, roll_deg

                    timestamp = int(time.time() * 1000)
                    send_ws_message(ws, yaw_deg, pitch_deg, roll_deg, timestamp)

            except websocket.WebSocketConnectionClosedException as e:
                print(f"WebSocket connection closed: {e}")
                break  # Exit inner loop to reconnect

            except Exception as e:
                print(f"Error: {e}")
                break  # Exit inner loop to reconnect

    except Exception as e:
        print(f"Failed to connect to WebSocket: {e}")
        time.sleep(5)

    finally:
        if ws:
            ws.close()
