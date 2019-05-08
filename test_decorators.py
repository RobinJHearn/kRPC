import unittest

import decorators

import logging


class Decorator_Tests(unittest.TestCase):
    logger_output = ""

    class stringHandler(object):
        def write(self, str):
            Decorator_Tests.logger_output = str

        def flush(self):
            pass

    logging.basicConfig(format="%(levelname)s %(message)s")
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)
    test_stream = stringHandler()
    handler = logging.StreamHandler(test_stream)
    logger.addHandler(handler)

    def test_none(self):
        Decorator_Tests.logger.info("Start of none")
        self.assertEqual(Decorator_Tests.logger_output, "Start of none\n")
        Decorator_Tests.logger_output = ""
        Decorator_Tests.logger.debug("Debug Text")
        self.assertEqual(Decorator_Tests.logger_output, "")
        Decorator_Tests.logger.info("End of none"),
        self.assertEqual(Decorator_Tests.logger_output, "End of none\n")

    @decorators.log_as(logger, logging.DEBUG)
    def test_basic(self):
        Decorator_Tests.logger.info("Start of basic")
        self.assertEqual(Decorator_Tests.logger_output, "Start of basic\n")
        Decorator_Tests.logger.debug("Debug Text")
        self.assertEqual(Decorator_Tests.logger_output, "Debug Text\n")
        Decorator_Tests.logger.info("End of basic"),
        self.assertEqual(Decorator_Tests.logger_output, "End of basic\n")

    @decorators.log_debug(logger)
    def test_debug(self):
        Decorator_Tests.logger.info("Start of debug")
        self.assertEqual(Decorator_Tests.logger_output, "Start of debug\n")
        Decorator_Tests.logger.debug("Debug Text")
        self.assertEqual(Decorator_Tests.logger_output, "Debug Text\n")
        Decorator_Tests.logger.info("End of debug"),
        self.assertEqual(Decorator_Tests.logger_output, "End of debug\n")


if __name__ == "__main__":
    unittest.main()

