import ascii_magic
import time
from pyfiglet import figlet_format
from random import choice, randint
from rich import print
from rich.highlighter import Highlighter
from rich.style import Style
from rich.color import Color
from rich.panel import Panel
from rich.align import Align

class RainbowHighlighter(Highlighter):
    def highlight(self, text):
        for index in range(len(text)):
            text.stylize(f"color({randint(16, 255)})", index, index + 1)

class OcadoHighlighter(Highlighter):
    def highlight(self, text):
        for index in range(len(text)):
            text.stylize(f"color({choice([37,255])})", index, index + 1)

def print_ocado(text):
    ocado = OcadoHighlighter()
    print(ocado(text))

def print_panel(text):
    print("\n\n")
    print(Panel(Align(text, align="center", vertical="middle"), width=64, height=10, title="DeusExCubus", subtitle="Author: Tihomir Petkov"))
    print("\n\n")

def get_star(width : int):
    line1 = "\\v/"
    line2 = ">+<"
    line3 = "/^\\"

    return [
        [line1 * width],
        [line2 * width],
        [line3 * width]
    ]

def print_stars(width : int, height : int):
    rainbow = RainbowHighlighter()
    lines = [get_star(width) for _ in range(height)]
    for l in lines:
        for layer in l:
            for sublayer in layer:
                print(rainbow(sublayer))

def print_rainbow(rows : int, columns : int, c : str):
    rainbow = RainbowHighlighter()
    for _ in range(rows):
        print(rainbow(c * columns))

def print_banner():
    # UR10e
    art = ascii_magic.from_url(
        "https://taxime-public.s3.eu-west-1.amazonaws.com/temp/ur20.png",
        columns=90,
        mode=ascii_magic.Modes.TERMINAL)
    ascii_magic.to_terminal(art)
    
    # Title
    font = 'big'
    print(figlet_format('DeusExCubus', font=font))

    time.sleep(4)