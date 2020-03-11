import carla
import pygame
try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

# Function that creates user's display. 
def make_display():
    display = pygame.display.set_mode(
        (800, 600),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption("User's camera view")
    return display

# Function that processes the image from Carla.Image class and then displays the result on the user's display.
def parse(surface,image,type_of_camera):
    if type_of_camera is 1:
        image.convert(carla.ColorConverter.Depth)
    elif type_of_camera is 2:
        image.convert(carla.ColorConverter.CityScapesPalette)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    surface.blit(image_surface,(0, 0))