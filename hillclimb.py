#
# Simple hill climb routine
#
import logging
import math

import decorators

logger = logging.getLogger(__name__)


@decorators.log_debug(logger)
def hill_climb(data, score_function):
    """Modify the list of data and use the score_function to improve it

    Parameters:
    `data`: a list of data points that can be manipulated
    `score_function`: a function that can take the data list and return a score, lower score is better
    """
    current_score = score_function(data)
    current_best = data.copy()
    for step in [100, 10, 1, 0.1, 0.01, 0.001]:
        improved = True
        while improved:
            # Create a list of candidate data lists with each elements adjusted by +/-step
            candidates = []
            # for c in range(0, len(data)):
            #     inc = current_best.copy()
            #     dec = current_best.copy()
            #     inc[c] += step
            #     dec[c] -= step
            #     candidates += [inc, dec]
            for c in range(1, int(math.pow(2, len(data)))):
                inc = current_best.copy()
                dec = current_best.copy()
                for b in range(0, len(data)):
                    if c & (1 << b) != 0:
                        inc[b] += step
                        dec[b] -= step
                candidates += [inc, dec]
            # Score each candidate
            improved = False
            for c in candidates:
                score = score_function(c)
                if score < current_score:
                    current_score = score
                    current_best = c
                    improved = True
            logger.debug(
                f"Hill Climb Results: Step={step}, Score={current_score}, Result={current_best}"
            )
    logger.info(f"Hill Climb Results: Score={current_score}, Result={current_best}")
    return current_best
