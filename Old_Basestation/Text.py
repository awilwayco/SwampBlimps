import pygame.font
from pygame.locals import Color

def getTextSurface(text, size=50, color=None):
    font = pygame.font.SysFont("Calibri",size)
    if(color==None):
        textColor = Color(0,0,0)
    else:
        textColor = color
    antialias = False
    return font.render(text,antialias,textColor)