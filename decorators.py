#
# Decorator to set logger level
#
import logging
import functools


class log_as(object):
    """Decorator to set the logging level for a function then return it to the original value"""

    def __init__(self, logger, level):
        """Arguments must be passed as that is what this is for

        Parameters:
        `logger`: The logging object to set the level of
        `level`: The logging level to set
        """
        self.logger = logger
        self.level = level

    def __call__(self, func):
        """Returns a wrapper the wraps func.
        The wrapper will store the current log level then set it to the specified value
        """

        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            """Wrapper function

                Store logging level
                Set required level
                Call original function
                Restore logging level
            """
            level = self.logger.level
            self.logger.setLevel(self.level)
            f_result = func(*args, **kwargs)
            self.logger.setLevel(level)
            return f_result

        return wrapper
