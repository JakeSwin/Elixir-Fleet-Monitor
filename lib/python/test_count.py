import sys
import time

from signal import signal, SIGPIPE, SIG_DFL
signal(SIGPIPE, SIG_DFL)

count = 0

while True:
    sys.stdout.write(f"Count: {count}")
    sys.stdout.flush()
    count += 1
    time.sleep(1)
