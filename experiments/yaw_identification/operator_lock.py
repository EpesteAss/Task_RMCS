"""Only one local operator may drive experiment services at a time."""
import fcntl
from pathlib import Path


def acquire():
    lock = (Path(__file__).resolve().parent / ".operator.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise SystemExit("Another console.py/session.py is running; close it first")
    return lock
