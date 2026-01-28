import logging

def setup_logging(level=logging.INFO):
    logging_format = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"

    logging.basicConfig(
        format=logging_format,
        level=level,
        datefmt="%H:%M:%S",
        force=True
    )
    logging.getLogger().setLevel(logging.INFO)
