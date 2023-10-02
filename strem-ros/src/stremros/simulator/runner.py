from carla import Client, World, CityObjectLabel, Transform
from stremros.simulator.ego import EgoVehicle
from stremros.simulator.sensor.camera import Camera

import cv2
import numpy as np

BOX_EDGES = [[0,1],[1,3],[3,2],[2,0],[0,4],[4,5],[5,1],[5,7],[7,6],[6,4],[6,2],[7,3]]

class Runner():
    """The simulation runner.
    """

    def __init__(self, host: str, port: int):
        """Initialize simulation.
        """

        self.client: Client = Client(host, port)
        self.world: World = self.client.get_world()

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05

        self.world.apply_settings(settings)

    def spawn(self, spawnable):
        """Spawn this object into the `World`.
        """

        return spawnable.spawn(self.world)
        
    def simulate(self, ego: EgoVehicle):
        """Simulate simulation.
        """

        bboxes = self.world.get_level_bbs(CityObjectLabel.TrafficLight)
        bboxes.extend(self.world.get_level_bbs(CityObjectLabel.TrafficSigns))
        bboxes.extend(self.world.get_level_bbs(CityObjectLabel.Sidewalks))
        self.world.tick()

        for sensor in ego.sensors:
            img = sensor.sample()
            img = np.reshape(np.copy(img.raw_data), (img.height, img.width, 4))

            cv2.namedWindow(f"{sensor.channel}", cv2.WINDOW_AUTOSIZE)
            cv2.imshow(f"{sensor.channel}", img)
            cv2.waitKey(1)

        while True:
            self.world.tick()

            for sensor in ego.sensors:
                img = sensor.sample()
                img = np.reshape(np.copy(img.raw_data), (img.height, img.width, 4))

                # Filter the set of bounding boxes by distance to the
                # `EgoVehicle`. This can be thought of as a camera resolution setting.
                detections = list(filter(
                    lambda bbox, ego=ego: bbox.location.distance(
                        ego.actor.get_transform().location
                    ) < 50,
                    bboxes
                ))

                for bbox in detections:
                    forward = ego.actor.get_transform().get_forward_vector()

                    if sensor.channel == "BackCamera":
                        forward = forward * -1.0

                    raycast = bbox.location - ego.actor.get_transform().location

                    if forward.dot(raycast) > 1:
                        vertices = [v for v in bbox.get_world_vertices(Transform())]

                        for edge in BOX_EDGES:
                            # Connect the vertices with edges.
                            p1 = Camera.project(
                                vertices[edge[0]],
                                Camera.intrinsic(sensor.width, sensor.height, sensor.fov),
                                np.array(sensor.actor.get_transform().get_inverse_matrix()),
                            )

                            p2 = Camera.project(
                                vertices[edge[1]],
                                Camera.intrinsic(sensor.width, sensor.height, sensor.fov),
                                np.array(sensor.actor.get_transform().get_inverse_matrix()),
                            )

                            # Draw the edges into the output of the image.
                            cv2.line(img,
                                (int(p1[0]),int(p1[1])),
                                (int(p2[0]),int(p2[1])),
                                (0,0,255, 255),
                                1
                            )

                cv2.imshow(f"{sensor.channel}", img)

                if cv2.waitKey(1) == ord('q'):
                    break

        cv2.destroyAllWindows()
