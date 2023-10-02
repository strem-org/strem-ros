from carla import Actor, Location, Transform, World
from enum import Enum
from queue import Queue
from typing import Optional
from typing_extensions import Self

import numpy as np

class CameraKind(Enum):
    RGB = 1,

class Camera():
    """A monocular camera sensor.
    """

    def __init__(self, kind: CameraKind, channel: str):
        self.channel = channel

        self.actor = None
        self.kind = kind

        self.width = None
        self.height = None
        self.fov = None

        self.intrinsic = None
        self.extrinsic = None

        self.samples: Queue = Queue()

    def spawn(self, world: World, transform: Transform, anchor: Optional[Actor]=None) -> Self:
        """Spawn `Camera` into `World` optionally attached to an anchor point.
        """

        if self.kind == CameraKind.RGB:
            blueprint = world.get_blueprint_library().find("sensor.camera.rgb")

            self.actor = world.try_spawn_actor(
                blueprint,
                transform,
                attach_to=anchor
            )

            self.width = blueprint.get_attribute("image_size_x").as_int()
            self.height = blueprint.get_attribute("image_size_y").as_int()
            self.fov = blueprint.get_attribute("fov").as_float()

            self.intrinsic = Camera.intrinsic(self.width, self.height, self.fov)

        # Attach the actor to the queue to send the samples collected
        # during simulation time.
        self.actor.listen(self.samples.put)

        return self

    def sample(self):
        return self.samples.get()
    
    def extrinsic(self):
        """Retrieve the extrinsic matrix of this `Camera`.
        """

        return np.array(self.actor.get_transform().get_inverse_matrix())
    
    def intrinsic(width: float, height: float, fov: float):
        """Construct this `Camera`'s intrinsic (i.e., projection) matrix.

        For more information, please see:
        https://carla.readthedocs.io/en/latest/tuto_G_bounding_boxes/
        """

        focal = width / (2.0 * np.tan(fov * np.pi / 360.0))

        projection = np.identity(3)
        projection[0, 0] = projection[1, 1] = focal
        projection[0, 2] = width / 2.0
        projection[1, 2] = height / 2.0

        return projection

    def project(pos: Location, intrinsic, extrinsic):
        """Project the object's `Position` onto this `Camera`.
        """

        point = np.array([pos.x, pos.y, pos.z, 1])

        # Translate the object to this `Camera`'s coordinate system.
        #
        # This involves taking the dot product of the object with the extrinsic
        # matrix of the camera, accordingly.
        point = np.dot(extrinsic, point)

        # Update the coordinate to the "standard" (i.e., [x, y, z] => [y, -z, x])
        # and remove the last component.
        #
        # There is no further definition of "standard" from:
        # https://carla.readthedocs.io/en/latest/tuto_G_bounding_boxes/
        point = [point[1], -point[2], point[0]]

        point = np.dot(intrinsic, point)

        # Normalize the result.
        point[0] /= point[2]
        point[1] /= point[2]

        return point[0:2]
