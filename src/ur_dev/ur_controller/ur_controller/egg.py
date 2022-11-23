#!/usr/bin/env python3

from __future__ import division
import asyncio

from asciimatics.effects import Print
from asciimatics.renderers import FigletText, ImageFile
from asciimatics.effects import Cycle, Stars
from asciimatics.renderers import FigletText
from asciimatics.scene import Scene
from asciimatics.screen import Screen

from urllib.request import urlopen
from shutil import copyfileobj
from tempfile import NamedTemporaryFile

def demo_animation(img_file):
    def update_screen(end_time, loop, screen):
        screen.draw_next_frame()
        if loop.time() < end_time:
            loop.call_later(0.02, update_screen, end_time, loop, screen)
        else:
            loop.stop()

    screen = Screen.open()
    effects = [
        Print(screen, ImageFile(img_file, screen.height - 2, colours=screen.colours),0),
        Stars(screen, (screen.width + screen.height) // 2)
    ]
    screen.set_scenes([Scene(effects, 500)])

    loop = asyncio.new_event_loop()
    end_time = loop.time() + 10.0
    loop.call_soon(update_screen, end_time, loop, screen)

    loop.run_forever()
    loop.close()
    screen.close()

def demo_flashing_text():
    def update_screen(end_time, loop, screen):
        screen.draw_next_frame()
        if loop.time() < end_time:
            loop.call_later(0.05, update_screen, end_time, loop, screen)
        else:
            loop.stop()

    screen = Screen.open()
    effects = [
        Cycle(
            screen,
            FigletText("Ocado Robotics Accelerator 2022", font='big'),
            screen.height // 2 - 8),
        Cycle(
            screen,
            FigletText("DeusExCubus", font='roman'),
            screen.height // 2 + 3),
        Stars(screen, (screen.width + screen.height) // 2)
    ]
    screen.set_scenes([Scene(effects, 500)])

    loop = asyncio.new_event_loop()
    end_time = loop.time() + 7.0
    loop.call_soon(update_screen, end_time, loop, screen)

    loop.run_forever()
    loop.close()
    screen.close()

def hatch():
    url = 'https://taxime-public.s3.eu-west-1.amazonaws.com/temp/UR10e.gif'
    with urlopen(url) as fsrc, NamedTemporaryFile(delete=False) as fdst:
        copyfileobj(fsrc, fdst)

        demo_flashing_text()
        demo_animation(fdst.name)

if __name__ == "__main__":
    hatch()