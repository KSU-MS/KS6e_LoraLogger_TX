import can
import time

def send_many():
    """Sends a single message, many times."""

    # see https://python-can.readthedocs.io/en/stable/configuration.html\
    bus = can.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=500000)

    with bus as canbus:
        msg = can.Message(
            arbitration_id=196, data=[0, 25, 0, 1, 3, 1, 4, 1], is_extended_id=False
        )
        while True:
            time.sleep(.2)
            try:
                canbus.send(msg)
                print(f"Message sent on {canbus.channel_info}")
            except can.CanError:
                print("Message NOT sent")


if __name__ == "__main__":
    send_many()