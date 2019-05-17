#
# Decorators
#
import logging
import functools


def log_as(logger, level):
    """Decorator to set the logging level for a function then return it to the original value"""

    def decorator(func):
        """The actual decorator function"""

        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            """Wrapper to temporarily change the logging level"""
            current_level = logger.level
            logger.setLevel(level)
            f_result = func(*args, **kwargs)
            logger.setLevel(current_level)
            return f_result

        return wrapper

    return decorator


def log_critical(logger):
    """Set the logger level to critical for the decorated function"""
    return log_as(logger, logging.CRITICAL)


def log_error(logger):
    """Set the logger level to error for the decorated function"""
    return log_as(logger, logging.ERROR)


def log_warning(logger):
    """Set the logger level to warning for the decorated function"""
    return log_as(logger, logging.WARNING)


def log_info(logger):
    """Set the logger level to info for the decorated function"""
    return log_as(logger, logging.INFO)


def log_debug(logger):
    """Set the logger level to debug for the decorated function"""
    return log_as(logger, logging.DEBUG)


def singleton(cls):
    """Make a class a singleton"""
    instances = {}

    def get_instance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]

    return get_instance

