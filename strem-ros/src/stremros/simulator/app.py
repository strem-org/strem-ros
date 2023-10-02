from carla import Location, Rotation, Transform

from stremros.simulator.ego import EgoVehicle
from stremros.simulator.runner import Runner
from stremros.simulator.sensor.camera import Camera, CameraKind

class App():
    """The simulator application.
    """

    def __init__(self, host: str, port: int):
        """Initialize simulator.
        """

        self.host = host
        self.port = port

    def run(self):
        """Run the application.
        """

        runner = Runner(self.host, self.port)

        ego = EgoVehicle("vehicle.lincoln.mkz_2020").spawn(runner.world).autopilot(True)

        ego.attach(
            Camera(CameraKind.RGB, "FrontCamera"),
            Transform(Location(z=2))
        ).attach(
            Camera(CameraKind.RGB, "BackCamera"),
            Transform(Location(z=2), Rotation(yaw=180))
        )
        
        runner.simulate(ego)
