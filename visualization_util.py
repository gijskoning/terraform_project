import pygame

pygame.init()  # start pygame
DISPLAY = pygame.display.set_mode((800, 600))  # create a window (size in pixels)
DISPLAY.fill((255, 255, 255))  # white background
WINDOW_SCALE = 400


def draw_rectangle_from_config(config, line_width=2, color=(255, 0, 0)):
    config_scaled = []
    for c in config:
        c = c * WINDOW_SCALE
        c[1] *= -1
        c += DISPLAY.get_rect().center
        config_scaled.append(c)
    pygame.draw.polygon(DISPLAY, color, config_scaled, width=line_width)