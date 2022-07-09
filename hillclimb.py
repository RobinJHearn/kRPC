""" Simple hill climb routine """

import logging
import math

logger = logging.getLogger(__name__)


def hill_climb(data, score_function):
    """Modify the list of data and use the score_function to improve it

    Parameters:
    `data`: a list of data points that can be manipulated
    `score_function`: a function that can take the data list and return a score,
                      lower score is better
    """
    current_score = score_function(data)
    current_best = data.copy()
    for step in [100, 10, 1, 0.1, 0.01, 0.001]:
        improved = True
        while improved:
            # Create a list of candidate data lists with each elements adjusted by +/-step
            candidates = []
            for current in range(1, int(math.pow(2, len(data)))):
                inc = current_best.copy()
                dec = current_best.copy()
                for index in range(0, len(data)):
                    if current & (1 << index) != 0:
                        inc[index] += step
                        dec[index] -= step
                candidates += [inc, dec]
            # Score each candidate
            improved = False
            for current in candidates:
                score = score_function(current)
                if score < current_score:
                    current_score = score
                    current_best = current
                    improved = True
            logger.debug(
                "Hill Climb Results: Step=%s, Score=%s, Result=%s",
                step,
                current_score,
                current_best,
            )
    logger.info("Hill Climb Results: Score=%s, Result=%s", current_score, current_best)
    return current_best
