"""Entrada aislada al CLI oficial; no cambia las tareas anteriores."""
import sys
import isaac_task  # noqa: F401
from isaaclab import cli

if __name__=='__main__':
    command=sys.argv[1]
    {'train':cli.train,'play':cli.play,'zero':cli.zero_agent}[command](sys.argv[2:])
