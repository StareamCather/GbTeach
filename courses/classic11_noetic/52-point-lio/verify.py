"""Run both real Point-LIO publication modes and verify their evidence."""
from adapter import install_handlers, run


if __name__ == '__main__':
    install_handlers()
    run()
