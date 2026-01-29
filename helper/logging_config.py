import logging

def setup_logging(level=logging.INFO):
    root = logging.getLogger()

    # If handlers already exist, logging is already configured
    if root.handlers:
        return
    
    logging_format = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"

    logging.basicConfig(
        format=logging_format,
        level=level,
        datefmt="%H:%M:%S"
    )
    logging.getLogger().setLevel(logging.INFO)


def get_module_logger():
    import logging, sys
    spec = sys.modules[__name__].__spec__
    return logging.getLogger(spec.name if spec else __name__)

