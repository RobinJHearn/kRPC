#
# Python class to create a scrolling window in KSP for logger messages
#

import krpc
import time
import logging

logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler)


class KrpcLogStream(object):
    """Class to write log messages to a window in KSP

    When the window is full delete the earliest message
    """

    def __init__(self, conn, w=200, h=300, x=150, y=0):
        """Construct the stream object

        Parameters:
            `comm`: Communication channel to KSP
        """
        canvas = conn.ui.stock_canvas
        screen_size = canvas.rect_transform.size
        panel = canvas.add_panel()
        rect = panel.rect_transform
        rect.size = (w, h)
        rect.position = (x - (screen_size[0] / 2), y)

        self.lines = []
        self.text = panel.add_text("Your Message Here")
        self.text.rect_transform.anchor = (1, 1)
        self.text.rect_transform.position = (-98, -152)
        self.text.rect_transform.size = (w, h)

        self.text.color = (1, 1, 1)
        self.text.size = 12
        self.max_lines = h / 15

    def write(self, line):
        if len(self.lines) >= self.max_lines:
            self.lines.pop(0)
        self.lines.append(line)
        s = ""
        for l in self.lines:
            s += l
            if s[-1] != "\n":
                s += "\n"
        self.text.content = s

    def flush(self):
        pass


if __name__ == "__main__":
    stream = KrpcLogStream(krpc.connect(name="Test Logger Stream"))
    stream.write("Hello World")
    time.sleep(3)
    for i in range(1, 30):
        stream.write(f"Is this all it is {i}")
        time.sleep(1)
    stream.write("Goodbye!")
    time.sleep(3)
