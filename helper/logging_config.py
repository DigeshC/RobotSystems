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
