from carla import Actor, Transform, World
from typing_extensions import Self

import random

class EgoVehicle():
    """The ego vehicle.
    """

    def __init__(self, model: str):
        """Initialize this vehicle.
        """

        self.model = model

        self.actor = None
        self.world = None

        self.sensors = []

    
    def autopilot(self, yes: bool) -> Self:
        """Set autopilot.
        """

        self.actor.set_autopilot(yes)
        return self

    def spawn(self, world: World) -> Self:
        """Spawn this `EgoVehicle` in the provided `World`.
        """
        
        self.world = world
        self.actor = world.try_spawn_actor(
            world.get_blueprint_library().find(self.model),
            random.choice(world.get_map().get_spawn_points())
        )

        return self
    
    def attach(self, sensor: Actor, transform: Transform) -> Self:
        """Attach a sensor to this `EgoVehicle`.
        """

        self.sensors.append(sensor.spawn(self.world, transform, self.actor))
        return self
