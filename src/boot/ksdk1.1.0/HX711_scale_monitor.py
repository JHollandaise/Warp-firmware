"""Script for collecting and manipulating data KL03 with HX711 and Load Bar."""

import pylink
import time
import matplotlib.pyplot as plt

def main(target_device):
    """Creates an interactive terminal to the target via RTT.
    The main loop opens a connection to the JLink, and then connects
    to the target device. RTT is started, the number of buffers is presented,
    and then two worker threads are spawned: one for read, and one for write.
    The main loops sleeps until the JLink is either disconnected or the
    user hits ctrl-c.

    Args:
      target_device (string): The target CPU to connect to.

    Returns:
      Always returns ``0`` or a JLinkException.

    Raises:
      JLinkException on error.
    """
    jlink = pylink.JLink()
    print("connecting to JLink...")
    jlink.open()
    print("connecting to %s..." % target_device)
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(target_device)
    print("connected, starting RTT...")
    jlink.rtt_start()

    while True:
        try:
            num_up = jlink.rtt_get_num_up_buffers()
            num_down = jlink.rtt_get_num_down_buffers()
            print("RTT started, %d up bufs, %d down bufs." % (num_up, num_down))
            break
        except pylink.errors.JLinkRTTException:
            time.sleep(0.1)

    try:
        thread.start_new_thread(read_rtt, (jlink,))
        thread.start_new_thread(write_rtt, (jlink,))
        while jlink.connected():
            time.sleep(1)
        print("JLink disconnected, exiting...")
    except KeyboardInterrupt:
        print("ctrl-c detected, exiting...")
        pass


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))