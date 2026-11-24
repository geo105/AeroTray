"""Usar la CLI oficial de Isaac Lab con el registro externo de AeroTray."""
import sys
import isaaclab_aerotray  # noqa: F401
from isaaclab import cli

if __name__=='__main__':
    command=sys.argv[1] if len(sys.argv)>1 else 'train'
    {'train':cli.train,'play':cli.play,'zero':cli.zero_agent,'random':cli.random_agent}[command](sys.argv[2:])
