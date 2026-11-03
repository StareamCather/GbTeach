"""Execute and verify both FAST-LIO2 modes on one captured sensor bag."""
from adapter import install_handlers, run


if __name__ == '__main__':
    install_handlers()
    run()
