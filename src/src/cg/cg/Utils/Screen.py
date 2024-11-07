import pygame
import os

max_screen_width, max_screen_height = 800, 600  # Adjust if needed

def generate_screen(maze, resolution):
    grid_size = resolution // len(maze)
    screen = pygame.display.set_mode(size=(resolution, resolution))
    pygame.display.set_caption("Culling Games")
    return screen, grid_size

def load_images_from_directory(directory="lobotomy_kaisen"):
    images = []
    for filename in os.listdir(directory):
        if filename.endswith((".png", ".jpg", ".jpeg", ".bmp", ".gif")):
            image_path = os.path.join(directory, filename)
            images.append(pygame.image.load(image_path))
    return images

# Display a random image centered on the screen
def display_image(screen, image):
    system_screen_info = pygame.display.Info()
    system_screen_width, system_screen_height = system_screen_info.current_w, system_screen_info.current_h
    image_width, image_height = image.get_size()

    # Check if the image height is larger than the system screen height
    if image_height > system_screen_height:
        # Calculate the scaling factor to resize the image while keeping aspect ratio
        scale_factor = system_screen_height / image_height
        image_width = int(image_width * scale_factor)
        image_height = system_screen_height
        image = pygame.transform.scale(image, (image_width, image_height))

    # Resize the pygame screen to match the image size and center it on the system screen
    screen = pygame.display.set_mode((image_width, image_height))
    os.environ['SDL_VIDEO_CENTERED'] = '1'  # Center the window on the system screen
    pygame.display.set_mode((image_width, image_height))

    # Center the image on the screen
    image_rect = image.get_rect(center=(image_width // 2, image_height // 2))
    screen.fill((0, 0, 0))  # Fill the screen with a black background
    screen.blit(image, image_rect)
    pygame.display.flip()
