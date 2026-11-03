"""Run the official lesson adapter against the actual private experiment."""
from adapter import install_handlers, run


if __name__ == '__main__':
    install_handlers()
    run()
