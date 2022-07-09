"""Connection class to kRPC"""

import krpc


class Connection:
    """Connection to kRPC"""

    def __init__(self, logger, name="kRPC Python"):
        self.connection = krpc.connect(name=name)
        self.space_center = self.connection.space_center  # pylint: disable=no-member
        self.krpc = self.connection.krpc  # pylint: disable=no-member
        self.logger = logger

    def ksc(self):
        """Get space center reference"""
        return self.space_center

    def scene(self):
        """Return the current game scene"""
        return self.connection.krpc.current_game_scene  # pylint: disable=no-member

    def vessel(self):
        """Get active vessel if in flight otherwise return None"""
        if self.scene() == self.krpc.GameScene.flight:
            vessel = self.space_center.active_vessel  # pylint: disable=no-member
        else:
            vessel = None

        return vessel
