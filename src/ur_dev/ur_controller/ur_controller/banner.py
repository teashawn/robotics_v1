import ascii_magic
import time
from pyfiglet import figlet_format

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