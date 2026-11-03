"""Run and audit the LIO-SAM loop-closure A/B experiment."""
from adapter import install_handlers, run


if __name__ == '__main__':
    install_handlers()
    run()
